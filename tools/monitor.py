#!/usr/bin/env python3
"""Save Pi system load, memory, thermal and throttling evidence."""
import argparse
import json
import subprocess
import time
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--seconds',type=float,default=605)
p.add_argument('--output',type=Path,required=True)
args=p.parse_args()
deadline=time.monotonic()+args.seconds
with args.output.open('x') as f:
    while time.monotonic()<deadline:
        memory={line.split(':')[0]:line.split(':')[1].strip() for line in Path('/proc/meminfo').read_text().splitlines()}
        record={'boottime_ns':time.clock_gettime_ns(time.CLOCK_BOOTTIME),'loadavg':Path('/proc/loadavg').read_text().strip(),
                'cpu_counters':Path('/proc/stat').read_text().splitlines()[0],
                'memory':{k:memory[k] for k in ('MemTotal','MemAvailable','SwapTotal','SwapFree')},
                'temp_millic':int(Path('/sys/class/thermal/thermal_zone0/temp').read_text()),
                'throttled':subprocess.check_output(['vcgencmd','get_throttled'],text=True).strip()}
        f.write(json.dumps(record)+'\n'); f.flush(); time.sleep(2)
