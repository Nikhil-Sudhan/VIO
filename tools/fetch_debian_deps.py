#!/usr/bin/python3
"""Extract native Debian build dependencies into this project (no root/config scripts).

Uses installed apt's authenticated indexes, saves exact versions and SHA256,
then downloads/verifies archives. Does not modify the system package database.
"""
import concurrent.futures
import argparse
import hashlib
import json
import subprocess
from pathlib import Path
import apt
import apt_pkg

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ['cmake', 'libeigen3-dev', 'libceres-dev', 'libopencv-dev',
            'libboost-filesystem-dev', 'libboost-system-dev', 'libboost-thread-dev',
            'libboost-date-time-dev', 'python3-opencv', 'python3-scipy', 'python3-matplotlib']
KALIBR_PACKAGES = ['catkin', 'catkin-tools', 'python3-rosbag', 'python3-rospy',
                   'libroscpp-dev', 'libcv-bridge-dev', 'python3-cv-bridge',
                   'libboost-python-dev', 'libboost-regex-dev', 'libboost-serialization-dev',
                   'libpoco-dev', 'libv4l-dev', 'python3-igraph', 'python3-pyx']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=['openvins', 'kalibr', 'kalibr-gui', 'rviz'], default='openvins')
    args = parser.parse_args()
    manifest_path = ROOT / 'config' / ('debian-deps.lock.json' if args.profile == 'openvins' else args.profile+'-deps.lock.json')
    if manifest_path.exists():
        records = json.loads(manifest_path.read_text())
    else:
        apt_pkg.config.set('APT::Install-Recommends', 'false')
        cache = apt.Cache()
        packages={'openvins':PACKAGES,'kalibr':KALIBR_PACKAGES,'kalibr-gui':['python3-wxgtk4.0'],
                  'rviz':['rviz','python3-roslaunch','python3-rospy','rospack-tools',
                          'python3-sensor-msgs','python3-geometry-msgs','python3-nav-msgs',
                          'python3-visualization-msgs','python3-tf2-msgs']}[args.profile]
        for name in packages:
            cache[name].mark_install(auto_fix=True, auto_inst=True, from_user=True)
        records = []
        for package in sorted(cache.get_changes(), key=lambda p: p.name):
            if not (package.marked_install or package.marked_upgrade):
                continue
            version = package.candidate
            if not any(origin.trusted for origin in version.origins):
                raise RuntimeError(f'No trusted origin for {package.name}')
            records.append({'package': package.name, 'version': version.version,
                            'architecture': version.architecture, 'uri': version.uri,
                            'sha256': version.sha256, 'bytes': version.size})
        manifest_path.write_text(json.dumps(records, indent=2) + '\n')
    archives, prefix = ROOT / 'data/debs', ROOT / 'deps'
    archives.mkdir(parents=True, exist_ok=True)
    prefix.mkdir(exist_ok=True)
    print(f'{len(records)} packages, {sum(r["bytes"] for r in records)/1e6:.1f} MB', flush=True)

    def fetch(record):
        dest = archives / (record['package'] + '_' + record['version'].replace(':', '%3a') + '.deb')
        if not dest.exists() or hashlib.sha256(dest.read_bytes()).hexdigest() != record['sha256']:
            temp = dest.with_suffix('.partial')
            subprocess.run(['curl', '-4', '--fail', '--silent', '--show-error', '--location',
                            '--retry', '2', '--connect-timeout', '15', '--max-time', '180',
                            record['uri'], '-o', str(temp)], check=True)
            if hashlib.sha256(temp.read_bytes()).hexdigest() != record['sha256']:
                raise RuntimeError(f'Checksum mismatch: {record["package"]}')
            temp.replace(dest)
        return dest

    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        for i, dest in enumerate(pool.map(fetch, records), 1):
            subprocess.run(['dpkg-deb', '--extract', str(dest), str(prefix)], check=True)
            if i % 10 == 0 or i == len(records):
                print(f'Extracted {i}/{len(records)}', flush=True)
    print(f'Dependencies ready in {prefix}', flush=True)


if __name__ == '__main__':
    main()
