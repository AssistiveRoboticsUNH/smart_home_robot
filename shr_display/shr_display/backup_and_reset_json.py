#!/usr/bin/env python3
import argparse
import json
import os
import pathlib
import shutil
import sys
import time
from datetime import datetime

LOCKFILE = "/tmp/nightly_protocol_reset.lock"

def acquire_lock():
    if os.path.exists(LOCKFILE):
        # stale lock protection: if older than 2 hours, remove
        try:
            mtime = os.path.getmtime(LOCKFILE)
            if time.time() - mtime > 2 * 3600:
                os.remove(LOCKFILE)
            else:
                print("[nightly_protocol_reset] Another instance appears to be running. Exiting.")
                sys.exit(0)
        except Exception:
            # if any error checking lock, fail safe
            sys.exit(0)
    with open(LOCKFILE, "w") as f:
        f.write(str(os.getpid()))

def release_lock():
    try:
        if os.path.exists(LOCKFILE):
            os.remove(LOCKFILE)
    except Exception:
        pass

def reset_flags(obj):
    if isinstance(obj, dict):
        for k, v in list(obj.items()):
            if k in ("provided", "confirmed"):
                obj[k] = False
            else:
                reset_flags(v)
    elif isinstance(obj, list):
        for item in obj:
            reset_flags(item)

def atomic_write_json(path: pathlib.Path, data: dict):
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    with open(tmp_path, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    os.replace(tmp_path, path)  # atomic on same filesystem

def main():
    parser = argparse.ArgumentParser(description="Backup and reset protocol_routines.json nightly.")
    parser.add_argument(
        "--src",
        default="/home/hello-robot/smarthome_ws/src/smart-home-robot/shr_display/config/protocol_routines.json",
        help="Path to protocol_routines.json.",
    )
    parser.add_argument(
        "--backup-dir",
        default="/home/hello-robot/protocol_backup",
        help="Directory to store timestamped backups.",
    )
    args = parser.parse_args()

    src_path = pathlib.Path(args.src)
    backup_dir = pathlib.Path(args.backup_dir)

    if not src_path.exists():
        print(f"[nightly_protocol_reset] ERROR: Source JSON not found: {src_path}")
        sys.exit(1)

    # Ensure backup dir exists
    backup_dir.mkdir(parents=True, exist_ok=True)

    acquire_lock()
    try:
        # Backup
        ts = datetime.now().strftime("%m_%d_%y_%H_%M")
        backup_path = backup_dir / f"{ts}.json"
        shutil.copyfile(src_path, backup_path)
        print(f"[nightly_protocol_reset] Backed up to: {backup_path}")

        # Load, reset, and write back atomically
        with open(src_path, "r") as f:
            data = json.load(f)

        reset_flags(data)
        atomic_write_json(src_path, data)
        print(f"[nightly_protocol_reset] Reset 'provided' and 'confirmed' to false in: {src_path}")

    except Exception as e:
        print(f"[nightly_protocol_reset] ERROR: {e}")
        sys.exit(1)
    finally:
        release_lock()

if __name__ == "__main__":
    main()
