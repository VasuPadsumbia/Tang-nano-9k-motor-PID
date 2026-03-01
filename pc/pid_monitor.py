#!/usr/bin/env python3
"""
=============================================================================
Script  : pid_monitor.py
Project : Tang Nano 9K – PID Motor Controller over Ethernet
File    : pc/pid_monitor.py

Purpose : PC-side dashboard for monitoring and controlling the FPGA PID motor
          controller over UDP.

Features
  - Live display of setpoint, feedback, PID output, error, and gains
  - Interactive commands: set setpoint, tune gains, enable/disable PID
  - Automatic CSV logging to data/telem_<timestamp>.csv
  - Auto-reconnect on packet loss (UDP is connectionless)

Protocol
  - FPGA IP   : 10.10.10.100  (change LOCAL_FPGA_IP below)
  - FPGA Port : 5005           (change FPGA_PORT below)
  - PC Port   : 5005           (FPGA replies here)
  - Command packet: 6 bytes  [CMD:1][VAL_HI:1][VAL_LO:1][PAD:3]
  - Telem packet : 20 bytes  [setpt:2][fb:2][pid:2][err:2][kp:2][ki:2][kd:2][stat:1][pad:5]

Command IDs
  0x01  Set setpoint   (signed 16-bit)
  0x02  Set Kp gain    (unsigned 16-bit, 8.8 fixed-point → divide by 256 for real)
  0x03  Set Ki gain
  0x04  Set Kd gain
  0x05  PID enable/disable  (0=disable, 1=enable)
  0x06  Reset PID (clear integrator)

Usage
  python pc/pid_monitor.py [--fpga-ip 10.10.10.100] [--port 5005]
=============================================================================
"""

import socket
import struct
import threading
import time
import csv
import datetime
import argparse
import os
import sys

try:
    from rich.console import Console
    from rich.table import Table
    from rich.live import Live
    from rich.panel import Panel
    from rich.layout import Layout
    from rich.text import Text
    from rich import box
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False
    print("Note: 'rich' not installed. Using plain text output.")
    print("Install with: pip install rich\n")

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DEFAULT_FPGA_IP   = "10.10.10.100"
DEFAULT_PORT      = 5005
RECV_TIMEOUT_S    = 0.5      # seconds to wait for telemetry before showing "---"
TELEM_PACKET_SIZE = 20       # bytes

# Command IDs
CMD_SETPOINT = 0x01
CMD_KP       = 0x02
CMD_KI       = 0x03
CMD_KD       = 0x04
CMD_ENABLE   = 0x05
CMD_RESET    = 0x06


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
class PIDState:
    def __init__(self):
        self.setpoint  = 0
        self.feedback  = 0
        self.pid_out   = 0
        self.error     = 0
        self.kp        = 0x0100
        self.ki        = 0x0020
        self.kd        = 0x0080
        self.status    = 0
        self.rx_count  = 0
        self.last_rx   = None
        self.lock      = threading.Lock()

    @property
    def pid_enabled(self):
        return bool(self.status & 0x01)

    @property
    def enc_dir(self):
        return bool(self.status & 0x02)

    def kp_real(self): return self.kp / 256.0
    def ki_real(self): return self.ki / 256.0
    def kd_real(self): return self.kd / 256.0

    def update_from_packet(self, data: bytes):
        if len(data) < TELEM_PACKET_SIZE:
            return False
        vals = struct.unpack_from(">hhhh HHH B", data, 0)
        with self.lock:
            self.setpoint = vals[0]
            self.feedback = vals[1]
            self.pid_out  = vals[2]
            self.error    = vals[3]
            self.kp       = vals[4]
            self.ki       = vals[5]
            self.kd       = vals[6]
            self.status   = vals[7]
            self.rx_count += 1
            self.last_rx   = datetime.datetime.now()
        return True


state = PIDState()


# ---------------------------------------------------------------------------
# UDP send helper
# ---------------------------------------------------------------------------
def send_cmd(sock, fpga_addr, cmd_id: int, value: int = 0):
    """Send a 6-byte command packet to the FPGA."""
    val = value & 0xFFFF
    packet = struct.pack(">BBBxxx", cmd_id, (val >> 8) & 0xFF, val & 0xFF)
    sock.sendto(packet, fpga_addr)


# ---------------------------------------------------------------------------
# Receiver thread
# ---------------------------------------------------------------------------
def rx_thread(sock, csv_writer, csv_fd):
    while True:
        try:
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            continue
        except OSError:
            break

        if state.update_from_packet(data):
            with state.lock:
                row = [
                    datetime.datetime.now().isoformat(timespec='milliseconds'),
                    state.setpoint, state.feedback,
                    state.pid_out, state.error,
                    state.kp, state.ki, state.kd,
                    state.status,
                ]
            csv_writer.writerow(row)
            csv_fd.flush()


# ---------------------------------------------------------------------------
# Rich display
# ---------------------------------------------------------------------------
def make_display():
    """Build a Rich renderable showing current PID state."""
    with state.lock:
        age = (datetime.datetime.now() - state.last_rx).total_seconds() \
              if state.last_rx else None
        online = (age is not None and age < 1.0)
        conn_str = (
            f"[bold green]ONLINE[/]  ({state.rx_count} packets)"
            if online else
            "[bold red]NO SIGNAL[/]  (waiting for telemetry…)"
        )

        tbl = Table(box=box.SIMPLE_HEAVY, show_header=True,
                    header_style="bold cyan", min_width=52)
        tbl.add_column("Parameter",   style="dim",         width=18)
        tbl.add_column("Value",       style="bold white",  width=12, justify="right")
        tbl.add_column("Real Unit",   style="green",       width=16)

        tbl.add_row("Setpoint",  f"{state.setpoint:+6d}",  "counts")
        tbl.add_row("Feedback",  f"{state.feedback:+6d}",  "counts")
        tbl.add_row("PID Output",f"{state.pid_out:+6d}",   "duty/65535")
        tbl.add_row("Error",     f"{state.error:+6d}",     "counts")
        tbl.add_row("─"*18,      "─"*12,                  "─"*16)
        tbl.add_row("Kp (reg)",  f"0x{state.kp:04X}",     f"{state.kp_real():.4f}")
        tbl.add_row("Ki (reg)",  f"0x{state.ki:04X}",     f"{state.ki_real():.4f}")
        tbl.add_row("Kd (reg)",  f"0x{state.kd:04X}",     f"{state.kd_real():.4f}")
        tbl.add_row("─"*18,      "─"*12,                  "─"*16)
        tbl.add_row("PID Enable",
                    "[green]YES[/]" if state.pid_enabled else "[red]NO[/]", "")
        tbl.add_row("Enc Dir",
                    "[cyan]FWD[/]" if state.enc_dir else "[yellow]REV[/]", "")

    layout = Layout()
    layout.split_column(
        Layout(Panel(conn_str, title="[bold]FPGA Connection", border_style="blue"), size=3),
        Layout(Panel(tbl, title="[bold]PID Status", border_style="cyan")),
        Layout(Panel(
            "[dim]Commands: [bold]S[/]<value>  [bold]P[/]<hex>  [bold]I[/]<hex>  "
            "[bold]D[/]<hex>  [bold]E[/] enable  [bold]X[/] disable  "
            "[bold]R[/] reset  [bold]Q[/] quit[/]\n"
            "Example: S8000  P0200  I0040  D0100",
            title="[bold]Controls", border_style="green"), size=5),
    )
    return layout


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="FPGA PID Motor Monitor")
    parser.add_argument("--fpga-ip", default=DEFAULT_FPGA_IP)
    parser.add_argument("--port",    default=DEFAULT_PORT, type=int)
    args = parser.parse_args()

    fpga_addr = (args.fpga_ip, args.port)
    print(f"[pid_monitor] FPGA: {fpga_addr[0]}:{fpga_addr[1]}")

    # Open UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", args.port))      # listen on same port
    sock.settimeout(RECV_TIMEOUT_S)

    # Open CSV log
    os.makedirs("data", exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"data/telem_{ts}.csv"
    csv_fd = open(csv_path, "w", newline="")
    csv_writer = csv.writer(csv_fd)
    csv_writer.writerow(["timestamp", "setpoint", "feedback", "pid_out",
                         "error", "kp", "ki", "kd", "status"])
    print(f"[pid_monitor] Logging to {csv_path}")

    # Start receiver
    t = threading.Thread(target=rx_thread, args=(sock, csv_writer, csv_fd),
                         daemon=True)
    t.start()

    def process_cmd(line: str):
        line = line.strip().upper()
        if not line:
            return
        try:
            if line[0] == 'S':   # Setpoint
                val = int(line[1:]) & 0xFFFF
                send_cmd(sock, fpga_addr, CMD_SETPOINT, val)
                print(f"  → Setpoint = {val}")
            elif line[0] == 'P':  # Kp
                val = int(line[1:], 16)
                send_cmd(sock, fpga_addr, CMD_KP, val)
                print(f"  → Kp = 0x{val:04X} ({val/256:.4f})")
            elif line[0] == 'I':  # Ki
                val = int(line[1:], 16)
                send_cmd(sock, fpga_addr, CMD_KI, val)
                print(f"  → Ki = 0x{val:04X} ({val/256:.4f})")
            elif line[0] == 'D':  # Kd
                val = int(line[1:], 16)
                send_cmd(sock, fpga_addr, CMD_KD, val)
                print(f"  → Kd = 0x{val:04X} ({val/256:.4f})")
            elif line[0] == 'E':  # Enable
                send_cmd(sock, fpga_addr, CMD_ENABLE, 1)
                print("  → PID ENABLED")
            elif line[0] == 'X':  # Disable
                send_cmd(sock, fpga_addr, CMD_ENABLE, 0)
                print("  → PID DISABLED")
            elif line[0] == 'R':  # Reset
                send_cmd(sock, fpga_addr, CMD_RESET, 0)
                print("  → PID RESET (integrator cleared)")
            elif line[0] == 'Q':
                raise SystemExit(0)
            else:
                print(f"  Unknown command: {line}")
        except ValueError as e:
            print(f"  Parse error: {e}")

    if RICH_AVAILABLE:
        console = Console()
        try:
            with Live(make_display(), refresh_per_second=4,
                      console=console, screen=False) as live:
                print("\nEnter commands (Q to quit):")
                while True:
                    line = input("> ")
                    process_cmd(line)
                    live.update(make_display())
        except (SystemExit, KeyboardInterrupt):
            pass
    else:
        # Plain-text fallback
        print("Enter commands (Q to quit):")
        while True:
            try:
                line = input("> ")
                process_cmd(line)
                with state.lock:
                    print(f"  State: SP={state.setpoint}  FB={state.feedback}  "
                          f"OUT={state.pid_out}  ERR={state.error}")
            except (SystemExit, KeyboardInterrupt):
                break

    csv_fd.close()
    sock.close()
    print(f"\nSession log saved: {csv_path}")


if __name__ == "__main__":
    main()
