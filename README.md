# Tang Nano 9K — PID Motor Controller over Ethernet

> **Embedded PID motor drive with UDP telemetry and control over a LAN8720 RMII Ethernet PHY**

---

## Hardware

| Component | Part |
|-----------|------|
| FPGA Board | Sipeed Tang Nano 9K (Gowin GW1NR-9) |
| Ethernet PHY | LAN8720 RMII module |
| Motor Driver | External H-bridge (L298N / DRV8833 / TB6612) |
| Motor Feedback | Quadrature encoder (A/B channels, 3.3 V or with level shifter) |

---

## Features

- **Fixed-point PID** controller with signed 16-bit I/O and anti-windup integral clamping
- **16-bit PWM** motor drive (~762 Hz) with dead-band and direction control
- **Quadrature encoder** reader (full 4× decoding, 32-bit signed position, metastability protection)
- **RMII Ethernet** stack built from scratch (ARP responder, IPv4/UDP parser, frame builder with IP checksum and CRC-32 FCS)
- **UDP control** — PC sends 6-byte command packets to update setpoint, Kp/Ki/Kd, enable/disable
- **UDP telemetry** — FPGA sends 20-byte status packets every 50 ms
- **PC dashboard** (`pc/pid_monitor.py`) with live Rich TUI and CSV logging, runs in an isolated **Python venv**
- Modular RTL organized into `comm/`, `motor/`, `sys/` subfolders
- Full testbench suite outputting VCD waveforms and CSV logs to `data/`

---

## Quick Start

### 1. Install Toolchain

```bash
pip install --user apio==0.9.*
apio install --all        # downloads Gowin toolchain
sudo apt install iverilog gtkwave python3 python3-venv
```

→ See [`docs/ide_setup.md`](docs/ide_setup.md) for VS Code extensions.

### 2. Wire Hardware

→ See [`docs/wiring_guide.md`](docs/wiring_guide.md) for full diagrams.

| Signal | FPGA Pin |
|--------|----------|
| CLK50 (LAN8720 REF_CLK) | 38 |
| RMII_TXD[0/1] | 77, 76 |
| RMII_TX_EN | 75 |
| RMII_RXD[0/1] | 73, 72 |
| RMII_CRS_DV | 71 |
| PWM_OUT → H-bridge EN | 25 |
| MOTOR_DIR → H-bridge IN1 | 26 |
| ENC_A | 27 |
| ENC_B | 28 |

### 3. Configure Network

Set PC NIC to static IP `10.10.10.10 / 255.255.255.0`. FPGA is at `10.10.10.100`.

### 4. Build and Flash

```bash
apio build
apio upload
```

### 5. Run Simulations

```bash
# PID unit test
iverilog -g2012 -o data/sim_pid tb/tb_pid_controller.v rtl/motor/pid_controller.v
vvp data/sim_pid

# PWM test
iverilog -g2012 -o data/sim_pwm tb/tb_pwm_gen.v rtl/motor/pwm_gen.v
vvp data/sim_pwm

# Full integration test (system + Ethernet communication)
iverilog -g2012 -o data/sim_top \
  tb/tb_top.v rtl/top.v \
  rtl/sys/clk_div.v \
  rtl/motor/pid_controller.v rtl/motor/motor_bridge.v \
  rtl/motor/pwm_gen.v rtl/motor/encoder_reader.v \
  rtl/comm/rmii_rx.v rtl/comm/rmii_tx.v rtl/comm/eth_crc32.v \
  rtl/comm/eth_rx_parser.v rtl/comm/eth_tx_builder.v \
  rtl/comm/arp_handler.v rtl/comm/udp_ctrl_rx.v rtl/comm/udp_telem_tx.v
vvp data/sim_top
```

Waveforms open with: `gtkwave data/sim_top.vcd`

### 6. PC Dashboard (venv)

```bash
# First time only
cd pc && bash setup_venv.sh

# Every run
bash run.sh
```

**Dashboard commands:**

| Key | Action |
|-----|--------|
| `S<val>` | Set setpoint, e.g. `S16384` |
| `P<hex>` | Set Kp, e.g. `P0100` (= 1.0) |
| `I<hex>` | Set Ki |
| `D<hex>` | Set Kd |
| `E` | Enable PID |
| `X` | Disable PID |
| `R` | Reset integrator |
| `Q` | Quit |

---

## Project Structure

```
rtl/
├── top.v                   ← Top-level
├── sys/clk_div.v           ← 1 kHz sample tick
├── motor/
│   ├── encoder_reader.v    ← Quadrature decoder
│   ├── pid_controller.v    ← Fixed-point PID
│   ├── motor_bridge.v      ← PID → PWM + DIR
│   └── pwm_gen.v           ← 16-bit PWM
└── comm/
    ├── rmii_rx/tx.v        ← RMII MAC layer
    ├── eth_crc32.v         ← CRC-32 FCS
    ├── eth_rx_parser.v     ← ETH/IP/UDP parser
    ├── eth_tx_builder.v    ← Frame builder
    ├── arp_handler.v       ← ARP responder
    ├── udp_ctrl_rx.v       ← Command decoder
    └── udp_telem_tx.v      ← Telemetry sender
tb/                         ← Testbenches
pc/                         ← Python dashboard + venv
docs/                       ← Architecture, wiring, tuning, IDE setup
data/                       ← Simulation outputs (VCD, CSV)
```

---

## Documentation

| File | Contents |
|------|----------|
| [`docs/architecture.md`](docs/architecture.md) | Block diagram, data flow, signal tables, packet formats |
| [`docs/wiring_guide.md`](docs/wiring_guide.md) | Hardware connection diagrams (LAN8720, H-bridge, encoder) |
| [`docs/pid_tuning_guide.md`](docs/pid_tuning_guide.md) | Gain format, manual tuning, Ziegler-Nichols method |
| [`docs/ide_setup.md`](docs/ide_setup.md) | VS Code extensions, APIO setup, simulation commands |

---

## LED Indicators (active LOW)

| LED | Meaning |
|-----|---------|
| LED1 | Heartbeat — toggles at ~0.75 Hz when running |
| LED2 | TX activity — flashes on every Ethernet transmit |
| LED3 | PID enabled — ON when PID is active |

---

## Future: EtherCAT

The Ethernet MAC layer (`rtl/comm/`) is designed as a clean separation from the motor control logic. The next step for EtherCAT will be:
1. Replace the UDP stack with an EtherCAT slave controller (ESC)
2. Map setpoint/feedback to EtherCAT PDOs
3. Motor bridge and PID modules remain unchanged

---

## License

MIT — see individual file headers.
