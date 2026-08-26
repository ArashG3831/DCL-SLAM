#!/usr/bin/env python3
"""Low-overhead host/WSL watch for long validation runs.

The JSONL file intentionally has no ROS dependencies.  If WSL or Windows is
restarted, the last complete record remains on disk and the missing terminal
record distinguishes interruption from a normal stop.
"""
from __future__ import annotations

import datetime as dt
import json
import os
import pathlib
import subprocess
import time


POWERSHELL = "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"


def command(*args: str) -> str:
    try:
        return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT,
                                       timeout=8).strip()
    except Exception as exc:  # pragma: no cover - host-dependent
        return f"ERROR:{type(exc).__name__}:{exc}"


def snapshot(event: str) -> dict:
    record = {
        "utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "event": event,
        "pid": os.getpid(),
        "wsl_boot_id": pathlib.Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
        "uptime_s": float(pathlib.Path("/proc/uptime").read_text().split()[0]),
        "loadavg": os.getloadavg(),
    }
    record["windows"] = command(
        POWERSHELL, "-NoProfile", "-Command",
        "$o=Get-CimInstance Win32_OperatingSystem; $p=Get-Counter "
        "'\\Memory\\Pool Nonpaged Bytes','\\Memory\\Available MBytes',"
        "'\\Memory\\% Committed Bytes In Use'; [pscustomobject]@{"
        "last_boot=$o.LastBootUpTime.ToString('o'); now=(Get-Date).ToString('o');"
        "counters=@($p.CounterSamples | ForEach-Object {[pscustomobject]@{"
        "path=$_.Path; value=$_.CookedValue}})} | ConvertTo-Json -Compress")
    record["memory"] = command("free", "-b")
    record["processes"] = command(
        "ps", "-eo", "pid,ppid,stat,pcpu,pmem,etime,comm", "--sort=-pcpu")
    return record


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("--period", type=float, default=15.0)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(snapshot("START")) + "\n")
        stream.flush()
        try:
            while True:
                time.sleep(max(2.0, args.period))
                stream.write(json.dumps(snapshot("SAMPLE")) + "\n")
                stream.flush()
        except KeyboardInterrupt:
            stream.write(json.dumps(snapshot("STOP")) + "\n")
            stream.flush()


if __name__ == "__main__":
    main()
