"""Bounded raw capture staging; preserve RAM data if archiving fails."""
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path


class RawRecordingStorage:
    def __init__(self, destination, required_bytes, staging_root=Path('/dev/shm/vio-raw-capture')):
        self.destination = Path(destination).resolve()
        staging_root = Path(staging_root)
        staging_root.mkdir(parents=True, exist_ok=True)
        available = next(int(line.split()[1])*1024 for line in Path('/proc/meminfo').read_text().splitlines()
                         if line.startswith('MemAvailable:'))
        if shutil.disk_usage(staging_root).free < required_bytes or available < required_bytes + 512_000_000:
            raise RuntimeError('Insufficient memory for the complete bounded recording')
        self.folder = Path(tempfile.mkdtemp(prefix=self.destination.name+'-', dir=staging_root))
        self.pointer = {'mode': 'RAM capture, verified archive after sensors stop',
                        'capture_folder': str(self.folder), 'archive_folder': str(self.destination),
                        'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                        'reserved_budget_bytes': required_bytes}
        self.set_state('capturing')

    def set_state(self, state, error=None):
        self.pointer['state'] = state
        if error is not None:
            self.pointer['error'] = str(error)
        temp = self.destination/'recording-storage.tmp.json'
        temp.write_text(json.dumps(self.pointer, indent=2)+'\n')
        temp.replace(self.destination/'recording-storage.json')

    @staticmethod
    def copy_verified(source, destination):
        source_hash = hashlib.sha256()
        with source.open('rb') as src, destination.open('xb') as dst:
            while block := src.read(1024*1024):
                source_hash.update(block)
                dst.write(block)
            dst.flush()
            os.fsync(dst.fileno())
        with destination.open('rb') as copied:
            copied_hash = hashlib.file_digest(copied, 'sha256').hexdigest()
        if copied_hash != source_hash.hexdigest():
            raise RuntimeError('Archive checksum mismatch: '+str(destination))
        return {'bytes': destination.stat().st_size, 'sha256': copied_hash}

    def persist(self):
        """Call only after all acquisition/writer threads stop. Never discard a failed copy."""
        try:
            self.set_state('saving')
            manifest = {}
            paths = sorted(self.folder.rglob('*'), key=lambda p: (p.name == 'session.json', str(p)))
            for source in paths:
                relative = source.relative_to(self.folder)
                target = self.destination/relative
                if source.is_dir():
                    target.mkdir(exist_ok=True)
                elif source.is_file():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    manifest[str(relative)] = self.copy_verified(source, target)
            (self.destination/'archive-manifest.json').write_text(json.dumps({
                'scope': 'Every staged file copied and SHA-256 verified after capture',
                'files': manifest}, indent=2)+'\n')
            self.set_state('saved')
        except Exception as error:
            self.set_state('save_failed', error)
            raise
        # Only this private, verified staging directory is removed.
        shutil.rmtree(self.folder)


def capture_folder(destination):
    """Resolve an active raw recording for read-only UI status/preview."""
    destination = Path(destination)
    pointer = destination/'recording-storage.json'
    if pointer.exists():
        data = json.loads(pointer.read_text())
        if data['state'] in ('capturing', 'saving', 'save_failed'):
            return Path(data['capture_folder'])
    return destination
