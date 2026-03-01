# Architecture Overview

## Tang Nano 9K — PID Motor Controller over Ethernet

---

## Block Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                     Tang Nano 9K  (Gowin GW1NR-9)                │
│                                                                  │
│  rtl/sys/                                                        │
│  ┌──────────────┐                                                │
│  │  clk_div     │ 50 MHz → 1 kHz sample_tick                     │
│  └──────┬───────┘                                                │
│         │ sample_tick                                            │
│  rtl/motor/                                                      │
│  ┌──────▼────────┐   position[31:16]   ┌────────────────────┐    │
│  │encoder_reader │────────────────────▶│  pid_controller    │    │
│  └───────────────┘                     │  Kp,Ki,Kd (8.8FP)  │    │
│  ENC_A/B                               │  anti-windup       │    │
│                                        └────────┬───────────┘    │
│                                                 │ pid_out[15:0]  │
│                                        ┌────────▼───────────┐    │
│                                        │  motor_bridge      │    │
│                                        │  signed→PWM+DIR    │    │
│                                        └────────┬───────────┘    │
│                                                 │                │
│                                        ┌────────▼───────────┐    │
│                                        │  pwm_gen           │    │
│                                        │  16-bit duty       │    │
│                                        └────────────────────┘    │
│                                         PWM_OUT   MOTOR_DIR      │
│                                                                  │
│  rtl/comm/                                                       │
│  ┌──────────┐  sof/eof/vld/byte  ┌──────────────┐                │
│  │ rmii_rx  │───────────────────▶│eth_rx_parser │                │
│  └──────────┘                    └──┬───────┬───┘                │
│  RMII_CRS_DV                        │       │                    │
│  RMII_RXD[1:0]               arp_req│  udp_payload_*             │
│                                     │       │                    │
│                            ┌────────▼──┐  ┌─▼──────────────┐     │
│                            │arp_handler│  │ udp_ctrl_rx    │     │
│                            │(reply buf)│  │ setpoint/gains │     │
│                            └────┬──────┘  └───────┬────────┘     │
│                                 │                 │              │
│                       ┌─────────▼─────────────────▼────────┐     │
│                       │        eth_tx_builder              │     │
│                       │  ARP priority arbiter              │     │
│                       │  IP checksum + CRC-32 FCS          │     │
│                       └─────────────────┬──────────────────┘     │
│                                         │                        │
│  ┌──────────────────┐         ┌─────────▼──────┐                 │
│  │ udp_telem_tx     │─tx_req─▶│   rmii_tx      │──▶ RMII_TX_EN   │
│  │ 50 ms interval   │         │  preamble+SFD  │──▶ RMII_TXD     │
│  └──────────────────┘         └────────────────┘                 │
└──────────────────────────────────────────────────────────────────┘
```

---

## Directory Structure

```
Tang-nano-9k-motor-PID/
├── rtl/
│   ├── top.v                  ← Top-level module
│   ├── sys/
│   │   └── clk_div.v          ← 50 MHz → 1 kHz tick
│   ├── motor/
│   │   ├── encoder_reader.v   ← Quadrature decoder
│   │   ├── pid_controller.v   ← Fixed-point PID (anti-windup)
│   │   ├── motor_bridge.v     ← Signed→PWM+DIR mapper
│   │   └── pwm_gen.v          ← 16-bit PWM (~760 Hz)
│   └── comm/
│       ├── rmii_rx.v          ← RMII receive → byte stream
│       ├── rmii_tx.v          ← Byte stream → RMII TX
│       ├── eth_crc32.v        ← CRC-32 (Ethernet FCS)
│       ├── eth_rx_parser.v    ← ARP / IPv4/UDP parser
│       ├── eth_tx_builder.v   ← Frame builder + arbiter
│       ├── arp_handler.v      ← ARP reply generator
│       ├── udp_ctrl_rx.v      ← UDP command decoder
│       └── udp_telem_tx.v     ← Periodic telemetry sender
├── tb/
│   ├── tb_pid_controller.v    ← PID unit test
│   ├── tb_pwm_gen.v           ← PWM duty cycle test
│   └── tb_top.v               ← Full integration test (incl. Ethernet)
├── pc/
│   ├── pid_monitor.py         ← Python UDP dashboard
│   ├── requirements.txt       ← Python deps (rich)
│   ├── setup_venv.sh          ← One-time venv setup
│   └── run.sh                 ← Venv-aware launcher
├── docs/
│   ├── architecture.md        ← This file
│   ├── wiring_guide.md        ← Hardware connection guide
│   ├── pid_tuning_guide.md    ← PID gain tuning reference
│   └── ide_setup.md           ← VSCode + APIO setup
├── data/                      ← Simulation outputs (VCD, CSV)
├── pinout.cst                 ← FPGA pin assignments
├── apio.ini                   ← APIO build configuration
├── requirements.txt           ← Host toolchain (APIO)
└── README.md                  ← Quick start guide
```

---

## Data Flow

### RX (PC → FPGA)
| Stage | Module | Output |
|-------|--------|--------|
| PHY dibit | `rmii_rx` | sof/eof/vld/byte |
| Frame parse | `eth_rx_parser` | arp_req or udp_payload_* |
| ARP | `arp_handler` | queues ARP reply |
| UDP | `udp_ctrl_rx` | setpoint, kp/ki/kd, enable |
| PID | `pid_controller` | pid_out |
| Drive | `motor_bridge` → `pwm_gen` | PWM_OUT, MOTOR_DIR |

### TX (FPGA → PC)
| Stage | Module | Trigger |
|-------|--------|---------|
| Timer | `udp_telem_tx` | every 50 ms (sample_tick × 50) |
| Build | `eth_tx_builder` | wraps payload + IP/UDP headers + FCS |
| Send | `rmii_tx` | preamble + SFD + frame dibits |

---

## Key Parameters

| Parameter | Default | Location | Notes |
|-----------|---------|----------|-------|
| `LOCAL_MAC` | `02:12:34:56:78:9A` | `rtl/top.v` | Locally-administered |
| `LOCAL_IP` | `10.10.10.100` | `rtl/top.v` | Set PC NIC to `10.10.10.x/24` |
| `LOCAL_PORT` | `5005` | `rtl/top.v` | UDP control + telemetry |
| `TICK_FREQ` | `1000 Hz` | `rtl/sys/clk_div.v` | PID sample rate |
| `TELEM_INTERVAL` | `50` ticks | `rtl/comm/udp_telem_tx.v` | = 50 ms |
| `ICLAMP` | `10 000 000` | `rtl/motor/pid_controller.v` | Anti-windup limit |
| `DEADBAND` | `256 / 65535` | `rtl/motor/motor_bridge.v` | ~0.4 % of full scale |

---

## Fixed-Point Gain Format (8.8)

```
Real gain = Register value / 256

0x0100 (256)  →  1.0
0x0080 (128)  →  0.5
0x0040  (64)  →  0.25
0x0200 (512)  →  2.0
```

---

## UDP Command Packet Format (PC → FPGA, 6 bytes)

| Byte | Field | Description |
|------|-------|-------------|
| 0 | CMD | 0x01=setpoint, 0x02=Kp, 0x03=Ki, 0x04=Kd, 0x05=enable, 0x06=reset |
| 1 | VAL_HI | MSB of 16-bit value |
| 2 | VAL_LO | LSB of 16-bit value |
| 3–5 | — | Reserved (send as 0x00) |

## UDP Telemetry Packet Format (FPGA → PC, 20 bytes, big-endian)

| Bytes | Field | Type |
|-------|-------|------|
| 0–1 | setpoint | signed 16-bit |
| 2–3 | feedback | signed 16-bit |
| 4–5 | pid_out | signed 16-bit |
| 6–7 | error | signed 16-bit |
| 8–9 | Kp | uint16 (8.8) |
| 10–11 | Ki | uint16 (8.8) |
| 12–13 | Kd | uint16 (8.8) |
| 14 | status | bit0=enabled, bit1=enc_dir |
| 15–19 | — | reserved |
