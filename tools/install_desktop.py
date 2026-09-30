#!/usr/bin/env python3
"""Install the user's manual desktop shortcut, backing up any existing file."""
from datetime import datetime
from pathlib import Path
import shutil

root = Path(__file__).resolve().parents[1]
desktop = Path.home()/'Desktop'
desktop.mkdir(exist_ok=True)
target = desktop/'VIO-Demo.desktop'
if target.exists():
    backup = root/'evidence'/('VIO-Demo.desktop.backup-'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    backup.parent.mkdir(exist_ok=True)
    shutil.copy2(target, backup)
text = (root/'packaging/VIO-Demo.desktop').read_text()
text = text.replace('/home/pluto/vio-project', str(root))
target.write_text(text)
target.chmod(0o755)
(root/'tools/start_native_demo.sh').chmod(0o755)
print(target)
