# Hardware Wiring Guide

## Tang Nano 9K — PID Motor Controller over Ethernet

---

## Overview

You need three physical connections to the Tang Nano 9K:
1. **LAN8720 Ethernet PHY module** (RMII)
2. **H-bridge motor driver** (L298N, DRV8833, or TB6612)
3. **Quadrature encoder** (from motor shaft)

---

## 1. LAN8720 Module → Tang Nano 9K

The LAN8720 breakout module provides RMII signals and generates the 50 MHz REF_CLK itself.

| LAN8720 Pin | Tang Nano 9K Pin | FPGA Signal | Notes |
|-------------|-----------------|-------------|-------|
| `VCC` | `3V3` | — | 3.3 V supply |
| `GND` | `GND` | — | Common ground |
| `INT/REFCLK` | `38` (CLK50) | CLK50 | 50 MHz clock – MUST be clock-capable pin |
| `TXD0` | `77` | RMII_TXD[0] | — |
| `TXD1` | `76` | RMII_TXD[1] | — |
| `TXEN` | `75` | RMII_TX_EN | — |
| `RXD0` | `73` | RMII_RXD[0] | — |
| `RXD1` | `72` | RMII_RXD[1] | — |
| `CRS_DV` | `71` | RMII_CRS_DV | — |
| `RXER` | `74` | RMII_RX_ER | Tie to GND if not wired |
| `MDC` | `68` | RMII_MDC | Held LOW (strap mode) |
| `MDIO` | `69` | RMII_MDIO | 10 kΩ pull-up on board |

```
LAN8720 Board                    Tang Nano 9K
┌──────────────┐                 ┌──────────────┐
│  VCC ────────┼─────────────────┼── 3V3        │
│  GND ────────┼─────────────────┼── GND        │
│  INT/REFCLK ─┼─────────────────┼── Pin 38     │  ← MUST be clk-capable
│  TXD0 ───────┼─────────────────┼── Pin 77     │
│  TXD1 ───────┼─────────────────┼── Pin 76     │
│  TXEN ───────┼─────────────────┼── Pin 75     │
│  RXD0 ───────┼─────────────────┼── Pin 73     │
│  RXD1 ───────┼─────────────────┼── Pin 72     │
│  CRS_DV ─────┼─────────────────┼── Pin 71     │
│  RXER ───────┼── (GND or) ─────┼── Pin 74     │
│  MDC ────────┼─────────────────┼── Pin 68     │
│  MDIO ───────┼─────────────────┼── Pin 69     │
└──────────────┘                 └──────────────┘
       │ RJ45
   Ethernet cable
   to PC / switch
```

> **Important**: pin 38 must be the `INT/REFCLK` output from the LAN8720.
> The FPGA does NOT provide this clock — the PHY generates it.

---

## 2. H-Bridge → Tang Nano 9K + Motor

### Wiring (L298N example)

| L298N Pin | Source | Notes |
|-----------|--------|-------|
| `ENA` | Tang Nano 9K pin 25 (`PWM_OUT`) | Motor A enable / speed |
| `IN1` | Tang Nano 9K pin 26 (`MOTOR_DIR`) | Direction |
| `IN2` | Inverter of `MOTOR_DIR` | See note below |
| `OUT1/OUT2` | Motor terminals A+ / A− | |
| `VCC` (logic) | `3V3` | |
| `VS` (motor) | Motor supply (7–12 V) | |
| `GND` | Common GND | |

> **IN2 inversion**: The FPGA outputs a single direction bit. If your H-bridge
> needs both IN1 and IN2 (complementary), wire through a small 74HC04 inverter,
> or use a motor driver that accepts a single DIR input (DRV8833 / TB6612).

```
Tang Nano 9K                L298N                    Motor
┌───────────┐              ┌────────┐                ┌──────┐
│ Pin 25 ───┼── PWM_OUT ──▶│ ENA   │                │      │
│ Pin 26 ───┼── MOTOR_DIR ▶│ IN1   │ OUT1 ──────────┤  M+  │
│           │  (inverter)──▶│ IN2  │ OUT2 ──────────┤  M−  │
│ 3V3 ──────┼─────────────▶│ VCC  │                └──────┘
│ GND ──────┼─────────────▶│ GND  │
└───────────┘              └────────┘
```

---

## 3. Quadrature Encoder → Tang Nano 9K

| Encoder Pin | Tang Nano 9K Pin | Notes |
|------------|-----------------|-------|
| `VCC` | `3V3` | 3.3 V encoders only. Use level shifter for 5 V |
| `GND` | `GND` | — |
| `A` | Pin 27 (`ENC_A`) | 100 Ω series resistor recommended |
| `B` | Pin 28 (`ENC_B`) | 100 Ω series resistor recommended |

> **Open-collector encoders**: add 10 kΩ pull-up from A and B to 3.3 V.
> **5 V encoders**: use a voltage divider or level shifter (e.g. TXS0102).

```
Encoder           100Ω             Tang Nano 9K
┌──────┐                          ┌──────────────┐
│  A ──┼──[ 100R ]───────────────▶│ Pin 27 (ENC_A)│
│  B ──┼──[ 100R ]───────────────▶│ Pin 28 (ENC_B)│
│ VCC ─┼───────────────────────── │ 3V3           │
│ GND ─┼───────────────────────── │ GND           │
└──────┘                          └──────────────┘
```

---

## 4. PC Network Setup

1. Set your PC NIC (wired Ethernet) to **static IP**:
   - IP: `10.10.10.10`
   - Subnet: `255.255.255.0`
   - Gateway: (leave blank)
2. Connect PC NIC → LAN8720 RJ45 (direct cable or via switch)
3. Verify with: `ping 10.10.10.100` (should reply once FPGA is running)

---

## 5. Power Checklist

| Item | Value |
|------|-------|
| FPGA supply | 5 V via USB (Tang Nano 9K USB-C) |
| LAN8720 supply | 3.3 V from FPGA 3V3 header |
| Motor supply | 7–12 V from external supply |
| Common GND | All grounds must be tied together |

> **Never** power the motor from the FPGA 3.3 V rail.
