#!/usr/bin/env python3
"""Interactive telecommand console for SIL (second TCP port)."""

from __future__ import annotations

import argparse
import socket
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from BasiliskSim.bridge.sil_protocol import pack_telecommand

# Matches SIL/cpp/command_packets.h TelecommandOpcode
TC_IDLE = 0
TC_FORCE_MODE = 1
TC_CLEAR_FORCE = 2
TC_INJECT_FAULT = 3  # arg0: 1 = ModeId replica, 2 = SOC 0.10
TC_CLEAR_FAULTS = 4
TC_SET_POINTING_TARGET = 5
TC_RESET_ESTIMATOR = 6
TC_SET_GAINS = 7
TC_SET_THRESHOLDS = 8
TC_RESET_FSW = 9

ALIASES: dict[str, tuple[int, int, int]] = {
    "idle": (TC_IDLE, 0, 0),
    "force": (TC_FORCE_MODE, 0, 0),  # use: force <mode_id>
    "unforce": (TC_CLEAR_FORCE, 0, 0),
    "inject": (TC_INJECT_FAULT, 0, 0),  # use: inject <1|2>
    "clear": (TC_CLEAR_FAULTS, 0, 0),
    "target": (TC_SET_POINTING_TARGET, 0, 0),  # use: target <0|1>
    "reset-est": (TC_RESET_ESTIMATOR, 0, 0),
    "reset-fsw": (TC_RESET_FSW, 0, 0),
}

HELP = """\
SIL telecommand console (32 B packets to sensor_receiver TC port).

Commands:
  help
  quit | exit
  <opcode> [arg0 [arg1]]     numeric (e.g. 1 2)
  force <mode>               TC_FORCE_MODE (0 Standby .. 3 Safe)
  unforce                    TC_CLEAR_FORCE (release operator override only)
  inject <1|2>               1 = ModeId replica flip, 2 = SOC 0.10
  clear                      TC_CLEAR_FAULTS (reset FDIR counters/replicas)
  target <0|1>               0 Sun, 1 Nadir
  reset-est                  TC_RESET_ESTIMATOR
  reset-fsw                  TC_RESET_FSW
  idle                       TC_IDLE
"""


def parse_line(line: str) -> tuple[int, int, int] | None:
    parts = line.strip().split()
    if not parts:
        return None
    cmd = parts[0].lower()
    if cmd in ("help", "?"):
        print(HELP, end="")
        return None
    if cmd in ("quit", "exit"):
        raise SystemExit(0)

    if cmd == "force":
        if len(parts) < 2:
            raise ValueError("usage: force <mode_id 0..3>")
        return (TC_FORCE_MODE, int(parts[1]), 0)
    if cmd == "inject":
        if len(parts) < 2:
            raise ValueError("usage: inject <1|2>")
        return (TC_INJECT_FAULT, int(parts[1]), 0)
    if cmd == "target":
        if len(parts) < 2:
            raise ValueError("usage: target <0 Sun | 1 Nadir>")
        return (TC_SET_POINTING_TARGET, int(parts[1]), 0)
    if cmd in ALIASES and len(parts) == 1:
        return ALIASES[cmd]

    opcode = int(parts[0])
    arg0 = int(parts[1]) if len(parts) > 1 else 0
    arg1 = int(parts[2]) if len(parts) > 2 else 0
    return (opcode, arg0, arg1)


def main() -> None:
    parser = argparse.ArgumentParser(description="SIL telecommand REPL")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5558)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((args.host, args.port))
    except OSError as exc:
        print(
            f"Could not connect to TC port {args.port}.\n"
            "Start sensor_receiver.exe first (it listens on that port).",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc

    print(f"Connected to TC port {args.port}. Type 'help' or 'quit'.")
    seq = 0
    try:
        while True:
            try:
                line = input("tc> ")
            except EOFError:
                break
            try:
                parsed = parse_line(line)
            except ValueError as exc:
                print(exc)
                continue
            except SystemExit:
                break
            if parsed is None:
                continue
            opcode, arg0, arg1 = parsed
            seq += 1
            packet = pack_telecommand(opcode, arg0, arg1, sequence=seq)
            sock.sendall(packet)
            print(f"sent opcode={opcode} arg0={arg0} arg1={arg1} ({len(packet)} B)")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
