#!/usr/bin/python3
"""Publish actual ROS-free OpenVINS output into its native RViz display."""
import argparse
import json
import math
import time
from pathlib import Path

import cv2
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    same_boot = json.loads((args.session/'session.json').read_text()).get('boot_id') == boot_id
    try:
        display_note = json.loads((args.session/'calibration/review.json').read_text()).get('display_note', '')
    except (OSError, ValueError):
        display_note = ''
    rospy.init_node('pi_openvins_display', disable_signals=True)
    pub_image = rospy.Publisher('/ov_msckf/trackhist', Image, queue_size=1)
    pub_path = rospy.Publisher('/ov_msckf/pathimu', RosPath, queue_size=1)
    pub_points = {name: rospy.Publisher('/ov_msckf/points_'+name, PointCloud2, queue_size=1)
                  for name in ('msckf', 'slam', 'aruco')}
    pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=2)
    pub_static = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
    pub_status = rospy.Publisher('/ov_msckf/status', Marker, queue_size=1, latch=True)
    origin = TransformStamped()
    origin.header.frame_id = 'world'
    origin.child_frame_id = 'global'
    origin.transform.rotation.w = 1.0
    origin.header.stamp = rospy.Time.now()
    pub_static.publish(TFMessage([origin]))
    path = RosPath()
    path.header.frame_id = 'global'
    previous = None
    last_image = None
    last_position = [0., 0., 0.]
    snap = {}
    while not rospy.is_shutdown():
        try:
            snap = json.loads((args.session/'vio/display.json').read_text())
        except (OSError, ValueError):
            pass
        age = (time.clock_gettime_ns(time.CLOCK_BOOTTIME)-snap.get('emitted_boot_ns', 0))/1e9
        fresh = same_boot and 0 <= age < 2.0
        stamp = rospy.Time.now()
        header = Header(stamp=stamp, frame_id='global')
        changed = snap.get('sequence') != previous
        rejected = False
        try:
            review = json.loads((args.session/'vio/display-review.json').read_text())
            rejected = review.get('usable_vio') is False
        except (OSError, ValueError):
            pass
        count = sum(len(snap.get(name, [])) for name in ('msckf', 'slam', 'aruco'))
        if rejected:
            state = 'TRACKING FAILED - estimate rejected'
        elif not fresh:
            state = 'STOPPED / STALE' if previous is not None else 'WAITING FOR SENSOR DATA'
        elif snap.get('image_mean_luma', 255) < 24 and snap.get('image_std_luma', 255) < 4:
            state = 'SCENE TOO DARK - turn on room lights'
        elif snap.get('zero_velocity_update'):
            state = 'STATIONARY UPDATE - moving accuracy unverified'
        elif snap.get('initialized'):
            visual_age = snap.get('last_msckf_update_age_s')
            state = (f'LIVE | {count} current 3D features' if snap.get('slam') or snap.get('aruco') or (visual_age is not None and visual_age <= 1)
                     else 'NO RECENT VISUAL CORRECTIONS - pose unreliable')
        elif snap.get('stationary_ready'):
            state = 'READY - keep both sensors fixed together'
        else:
            state = 'INITIALIZING - keep the rig still'
        if changed:
            previous = snap['sequence']
            last_image = cv2.imread(str(args.session/'vio/tracks.jpg'))
            if fresh and (snap['initialized'] or snap.get('stationary_ready')):
                p, q = snap['position'], snap['quaternion']
                last_position = p
                pose = PoseStamped(header=header)
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
                path.poses.append(pose)
                path.header.stamp = stamp
                pub_path.publish(path)
                transform = TransformStamped(header=header, child_frame_id='imu')
                transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = p
                transform.transform.rotation = pose.pose.orientation
                pub_tf.publish(TFMessage([transform]))
                for name, pub in pub_points.items():
                    pub.publish(point_cloud2.create_cloud_xyz32(header, snap.get(name, [])))
        if last_image is not None:
            pixels = last_image.copy()
            label = (f'LIVE OPENVINS | frame {previous} | age {age:.1f}s' if fresh
                     else 'STOPPED - last captured frame')
            cv2.rectangle(pixels, (0, pixels.shape[0]-86), (pixels.shape[1], pixels.shape[0]), (0,0,0), -1)
            color = (0,255,255) if fresh and not rejected else (0,100,255)
            cv2.putText(pixels, label, (10, pixels.shape[0]-51), cv2.FONT_HERSHEY_SIMPLEX, .8, color, 2)
            cv2.putText(pixels, state, (10, pixels.shape[0]-18), cv2.FONT_HERSHEY_SIMPLEX, .64, color, 2)
            msg = Image(header=Header(stamp=stamp, frame_id='cam0'), height=pixels.shape[0],
                        width=pixels.shape[1], encoding='bgr8', is_bigendian=0,
                        step=pixels.shape[1]*3, data=pixels.tobytes())
            pub_image.publish(msg)
        status = Marker(header=header, ns='status', id=0, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        # Keep the warning in the initial view even if an invalid pose diverges.
        status.pose.position.z = .45
        status.pose.orientation.w = 1
        status.scale.z = .045
        status.color.r, status.color.g, status.color.b, status.color.a = 1., .85, .2, 1.
        if rejected or 'unreliable' in state or not fresh:
            status.color.g = .2
        status.text = 'PROVISIONAL CALIBRATION\n' + (display_note+'\n' if display_note else '') + state
        pub_status.publish(status)
        time.sleep(.2)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
