"""Log telemetry from the ESP32 to a CSV file and send commands from the keyboard.

    python -m muscle_stim.log_serial --port COM5 --out session_01.csv
    python -m muscle_stim.log_serial --port /dev/ttyUSB0 --out test.csv --cmd "STATUS" --duration 120

While it runs, type a device command and press Enter (ARM, START TRAP, STOP, RESET, ...).
Type  quit  (or press Ctrl+C) to finish. Full command list: docs/protocol.md.

Line types coming from the device:
    "# ..."   status / information message (printed and saved to <out>.log, not in the CSV)
    "t_ms,..." header line (written once)
    numbers   one telemetry row, 100 Hz
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time

import serial

EXPECTED_FIELDS = 12


def _reader(ser, q, stop):
    buf = b""
    while not stop.is_set():
        try:
            chunk = ser.read(256)
        except serial.SerialException as exc:
            q.put(("error", str(exc)))
            return
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            q.put(("line", line.decode("ascii", errors="replace").strip()))


def _stdin_reader(ser, stop):
    for line in sys.stdin:
        cmd = line.strip()
        if cmd.lower() == "quit":
            stop.set()
            return
        if cmd:
            ser.write((cmd + "\n").encode("ascii"))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--out", required=True, help="CSV file for telemetry rows")
    ap.add_argument("--cmd", action="append", default=[], help="command sent once at start (repeatable)")
    ap.add_argument("--duration", type=float, default=None, help="stop after this many seconds")
    args = ap.parse_args(argv)

    stop = threading.Event()
    q: "queue.Queue[tuple[str, str]]" = queue.Queue()
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser, \
            open(args.out, "w", newline="") as fcsv, open(args.out + ".log", "w") as flog:
        time.sleep(1.5)                      # ESP32 resets when the port opens
        ser.reset_input_buffer()
        threading.Thread(target=_reader, args=(ser, q, stop), daemon=True).start()
        threading.Thread(target=_stdin_reader, args=(ser, stop), daemon=True).start()
        for c in args.cmd:
            ser.write((c + "\n").encode("ascii"))
            time.sleep(0.2)
        t0 = time.time()
        n_rows = 0
        header_seen = False
        print("Logging. Type device commands, 'quit' to stop.")
        try:
            while not stop.is_set():
                if args.duration and time.time() - t0 > args.duration:
                    break
                try:
                    kind, text = q.get(timeout=0.2)
                except queue.Empty:
                    continue
                if kind == "error":
                    print("serial error:", text)
                    break
                if text.startswith("#"):
                    print(text)
                    flog.write(f"{time.time() - t0:9.3f} {text}\n")
                    flog.flush()
                elif text.startswith("t_ms"):
                    if not header_seen:
                        fcsv.write(text + "\n")
                        header_seen = True
                elif text.count(",") == EXPECTED_FIELDS - 1:
                    fcsv.write(text + "\n")
                    n_rows += 1
                    if n_rows % 200 == 0:
                        fcsv.flush()
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            try:
                ser.write(b"STOP\n")         # never leave the stimulator running
            except Exception:
                pass
            fcsv.flush()
    print(f"Saved {n_rows} rows to {args.out}")


if __name__ == "__main__":
    main()
