#!/usr/bin/env python3
"""Keep an explicitly camera-only RViz preview live after a rejected VIO run."""
import argparse
import json
import time
from pathlib import Path

import cv2
from picamera2 import Picamera2
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--evidence', type=Path, required=True)
    args = p.parse_args()
    args.evidence.mkdir(parents=True, exist_ok=True)
    cfg = json.loads(args.config.read_text())
    width, height = cfg['main']['size']
    cv2.setNumThreads(1)
    rospy.init_node('pi_live_camera_preview', disable_signals=True)
    images = rospy.Publisher('/ov_msckf/trackhist', Image, queue_size=1)
    status = rospy.Publisher('/ov_msckf/status', Marker, queue_size=1, latch=True)
    paths = rospy.Publisher('/ov_msckf/pathimu', RosPath, queue_size=1, latch=True)
    points = [rospy.Publisher('/ov_msckf/points_'+n, PointCloud2, queue_size=1, latch=True)
              for n in ['msckf', 'slam']]
    tf = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
    tr = TransformStamped()
    tr.header.frame_id, tr.child_frame_id = 'world', 'global'
    tr.transform.rotation.w = 1
    tr.header.stamp = rospy.Time.now()
    tf.publish(TFMessage([tr]))
    header = Header(stamp=rospy.Time.now(), frame_id='global')
    paths.publish(RosPath(header=header))
    for pub in points:
        pub.publish(point_cloud2.create_cloud_xyz32(header, []))
    cam = Picamera2()
    try:
        cam.configure(cam.create_video_configuration(
            main={'size': (width, height), 'format': 'YUV420'},
            sensor={'output_size': tuple(cfg['sensor']['output_size']),
                    'bit_depth': cfg['sensor']['bit_depth']},
            controls=cfg['controls'], buffer_count=8, queue=False))
        cam.start()
        started = time.monotonic()
        last_connected = started
        count = 0
        while not rospy.is_shutdown() and time.monotonic()-started < 1800:
            if images.get_num_connections():
                last_connected = time.monotonic()
            elif time.monotonic()-last_connected > 20:
                break
            request = cam.capture_request()
            try:
                raw = request.make_array('main')[:height, :width].copy()
                metadata = request.get_metadata()
                sequence = request.request.sequence
            finally:
                request.release()
            now_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
            age = (now_ns-metadata['SensorTimestamp'])/1e9
            live = 0 <= age < 1
            pixels = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(pixels, (0, height-100), (width, height), (0,0,0), -1)
            cv2.putText(pixels, 'LIVE CAMERA ONLY' if live else 'CAMERA DELAYED',
                        (12,height-63), cv2.FONT_HERSHEY_SIMPLEX, 1.05, (0,255,255), 2)
            cv2.putText(pixels, 'VIO STOPPED: DRIFT REJECTED', (12,height-28),
                        cv2.FONT_HERSHEY_SIMPLEX, .9, (0,100,255), 2)
            stamp = rospy.Time.now()
            images.publish(Image(header=Header(stamp=stamp, frame_id='cam0'),
                height=height, width=width, encoding='bgr8', is_bigendian=0,
                step=width*3, data=pixels.tobytes()))
            marker = Marker(header=Header(stamp=stamp, frame_id='global'),
                            ns='status', id=0, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            marker.pose.orientation.w = 1
            marker.pose.position.z = .2
            marker.scale.z = .12
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .3, .2, 1.
            marker.text = 'VIO STOPPED - DRIFT REJECTED\nLive camera preview only'
            status.publish(marker)
            count += 1
            if count % 10 == 0:
                info = {'mode': 'live camera preview only; no pose estimation',
                        'sequence': sequence, 'sensor_timestamp_ns': metadata['SensorTimestamp'],
                        'emitted_boot_ns': now_ns, 'image_age_s': age, 'live': live}
                tmp = args.evidence/'preview-status.tmp.json'
                tmp.write_text(json.dumps(info, indent=2)+'\n')
                tmp.replace(args.evidence/'preview-status.json')
                cv2.imwrite(str(args.evidence/'preview-current.jpg'), pixels)
            time.sleep(.04)
    finally:
        cam.stop()
        cam.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
