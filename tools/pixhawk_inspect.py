#!/usr/bin/env python3
"""Bounded bench identification/parameter backup. Never sets parameters or sends VIO."""
import argparse
import json
import math
import os
from pathlib import Path
import stat
import struct
import time

os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device', required=True, help='Verified serial device, preferably /dev/serial/by-id/...')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--seconds', type=int, default=45)
    p.add_argument('--system-id', type=int, help='Required if multiple autopilots are on the link')
    p.add_argument('--request-info', action='store_true', help='Send one standard AUTOPILOT_VERSION request after heartbeat')
    p.add_argument('--backup-parameters', action='store_true', help='Send one PARAM_REQUEST_LIST; never PARAM_SET')
    p.add_argument('--output', type=Path, required=True, help='New output directory')
    a = p.parse_args()
    if not 1 <= a.seconds <= 120 or a.baud <= 0 or (a.system_id is not None and not 1 <= a.system_id <= 255):
        p.error('Use duration 1..120 s, positive baud, and system ID 1..255')
    device = Path(a.device).resolve(strict=True)
    if not str(device).startswith('/dev/') or not stat.S_ISCHR(device.stat().st_mode):
        p.error('Expected a verified local serial character device')
    if device.name == 'ttyAMA10':
        p.error('ttyAMA10 is the Pi 5 debug UART, not the verified TELEM2 connection')
    a.output.mkdir(parents=True, exist_ok=False)
    result = dict(scope='Bench identity and optional parameter backup; no navigation integration',
                  device=str(device), baud=a.baud, started_unix_s=time.time(),
                  firmware_identified=False, parameters_modified=False, vio_transmitted=False,
                  request_info=a.request_info, request_parameters=a.backup_parameters,
                  autopilots={}, parameters={}, parameter_counts=[], error=None)
    connection = None
    target = None
    counts = set()
    indices = {}
    requested = False
    try:
        connection = mavutil.mavlink_connection(str(device), baud=a.baud, source_system=245,
                                               source_component=191, dialect='common', autoreconnect=False)
        connection.setup_logfile(str(a.output/'received.tlog'), mode='wb')
        deadline = time.monotonic()+a.seconds
        while time.monotonic() < deadline:
            message = connection.recv_match(blocking=True, timeout=.5)
            if message is None or message.get_type() == 'BAD_DATA':
                continue
            system, component = message.get_srcSystem(), message.get_srcComponent()
            kind = message.get_type()
            if kind == 'HEARTBEAT' and message.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                enum = mavutil.mavlink.enums['MAV_AUTOPILOT'].get(message.autopilot)
                result['autopilots'][str(system)] = dict(component=component, autopilot_enum=message.autopilot,
                    autopilot_name=enum.name if enum else 'UNRECOGNIZED_'+str(message.autopilot),
                    vehicle_type=message.type, base_mode=message.base_mode, custom_mode=message.custom_mode)
                if target is None and (a.system_id is None or a.system_id == system):
                    target = (system, component)
                if len(result['autopilots']) > 1 and a.system_id is None:
                    raise RuntimeError('Multiple autopilots observed; repeat with explicit --system-id')
            if target is None or (system, component) != target:
                continue
            if not requested:
                if a.request_info:
                    connection.mav.command_long_send(*target, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                                       mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
                if a.backup_parameters:
                    connection.mav.param_request_list_send(*target)
                requested = True
            if kind == 'AUTOPILOT_VERSION':
                v = int(message.flight_sw_version)
                result['firmware'] = dict(system_id=system, component_id=component, flight_sw_version_raw=v,
                    major=(v >> 24) & 255, minor=(v >> 16) & 255, patch=(v >> 8) & 255,
                    release_type=v & 255, capabilities=int(message.capabilities),
                    vendor_id=message.vendor_id, product_id=message.product_id,
                    flight_custom_version=list(message.flight_custom_version))
                result['firmware_identified'] = v != 0
            elif kind == 'PARAM_VALUE' and a.backup_parameters:
                name = message.param_id
                if isinstance(name, bytes):name=name.decode(errors='replace')
                name = name.split('\0')[0]
                counts.add(int(message.param_count))
                index = int(message.param_index)
                if 0 <= index < message.param_count:
                    indices[index] = name
                value = float(message.param_value)
                result['parameters'][name] = dict(value=value if math.isfinite(value) else None,
                    float32_bits_hex=struct.pack('<f', value).hex(), type=int(message.param_type), index=index)
    except Exception as e:
        result['error'] = str(e)
    finally:
        if connection is not None:
            if connection.logfile is not None:connection.logfile.close()
            connection.close()
        result['finished_unix_s'] = time.time()
        result['parameter_counts'] = sorted(counts)
        n = next(iter(counts)) if len(counts) == 1 else 0
        result['parameter_backup_complete'] = bool(a.backup_parameters and n > 0 and
            set(indices) == set(range(n)) and len(result['parameters']) == n and result['error'] is None)
        result['parameter_value_note'] = ('Raw MAVLink PARAM_VALUE float and MAV_PARAM_TYPE are preserved; '
                                         'do not restore by guessing integer encoding. received.tlog retains wire messages.')
        (a.output/'inspection.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(a.output/'inspection.json')
    return int(result['error'] is not None or target is None or
               (a.request_info and not result['firmware_identified']) or
               (a.backup_parameters and not result['parameter_backup_complete']))


if __name__ == '__main__':
    raise SystemExit(main())
