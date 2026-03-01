# PID Tuning Guide

## Tang Nano 9K — PID Motor Controller over Ethernet

---

## Fixed-Point Gain Format (8.8)

All gains are sent as unsigned 16-bit integers using **8.8 fixed-point**:

```
Real gain = Register value / 256
```

| Register (hex) | Register (dec) | Real Gain |
|----------------|---------------|-----------|
| `0x0100` | 256 | 1.0 |
| `0x0080` | 128 | 0.5 |
| `0x0040` | 64 | 0.25 |
| `0x0200` | 512 | 2.0 |
| `0x0020` | 32 | 0.125 |
| `0x0400` | 1024 | 4.0 |

---

## Default Gains

| Gain | Default | Real |
|------|---------|------|
| Kp | `0x0100` | 1.0 |
| Ki | `0x0020` | 0.125 |
| Kd | `0x0080` | 0.5 |

---

## Manual Tuning Procedure (from PC dashboard)

### Step 1: Disable integral and derivative

Set Ki and Kd to zero:

```
> I0000
> D0000
> E        ← Enable PID
> S8000    ← Set setpoint to mid-range
```

Watch error in the dashboard. If motor oscillates, reduce Kp. If too slow, increase Kp.

### Step 2: Tune Kp

Increase `Kp` until the motor reaches the setpoint reasonably fast but without sustained oscillation:

```
> P0080    ← Kp = 0.5
> P0100    ← Kp = 1.0  (try doubling until oscillation)
> P0180    ← Kp = 1.5
```

Set Kp to **half the value that caused oscillation** (conservative start).

### Step 3: Add Ki (integral)

Ki eliminates steady-state error. Start very small to avoid integral windup:

```
> I0008    ← Ki = 0.031 (very small)
> I0010    ← Ki = 0.063
> I0020    ← Ki = 0.125 (default)
```

Increase until steady-state error is eliminated without causing slow oscillation.

> The integrator is clamped at ±10 000 000 to prevent windup.
> If you see slow oscillations, reduce Ki.

### Step 4: Add Kd (derivative)

Kd reduces overshoot and oscillation:

```
> D0040    ← Kd = 0.25
> D0080    ← Kd = 0.5
```

Increase until overshoot reduces. Stop before it causes high-frequency noise amplification.

### Step 5: Fine-tune iteratively

Use the live dashboard to observe `setpoint`, `feedback`, and `error` converging.

---

## Ziegler–Nichols Method (Quick Start)

1. Set Ki=0, Kd=0
2. Increase Kp until the motor **just starts oscillating continuously** (= Ku, ultimate gain)
3. Note the oscillation period Tu (measure from the CSV log timestamps)
4. Set gains:

| Controller | Kp | Ki | Kd |
|------------|----|----|-----|
| P only | `0.5 × Ku` | 0 | 0 |
| PI | `0.45 × Ku` | `0.54 × Ku / Tu` | 0 |
| PID | `0.6 × Ku` | `1.2 × Ku / Tu` | `0.075 × Ku × Tu` |

Convert real gain to register: `register = real_gain × 256` then send as hex.

---

## Setpoint and Feedback Scaling

- **Feedback** = `encoder_position[31:16]` (top 16 bits of 32-bit counter)
- This means **1 unit of feedback = 65536 encoder counts**
- For a 1000 CPR encoder: 65536 counts ≈ 65.5 revolutions
- **Setpoint** is in the same units (signed 16-bit)

Adjust the bit shift in `rtl/top.v` if needed:

```verilog
// Using top 16 bits (default: 1 unit = 65536 encoder counts)
wire signed [15:0] feedback = enc_position[31:16];

// Alternative: use top 8 bits (coarser, for high-resolution encoders)
// wire signed [15:0] feedback = {enc_position[31:24], 8'h00};
```

---

## Sending Commands from Dashboard

```
S<decimal>   Set setpoint    e.g. S16384  (= 0x4000)
P<hex>       Set Kp          e.g. P0100   (= 1.0)
I<hex>       Set Ki          e.g. I0020   (= 0.125)
D<hex>       Set Kd          e.g. D0080   (= 0.5)
E            Enable PID
X            Disable PID
R            Reset integrator
```

---

## Common Issues

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Motor oscillates continuously | Kp too high | Halve Kp |
| Motor never reaches setpoint | Kp too low or Ki=0 | Increase Kp, add Ki |
| Slow oscillation after settling | Ki too high (windup) | Reduce Ki, send R to clear |
| Motor jitters at rest | Dead-band too small | Increase `DEADBAND` in `motor_bridge.v` |
| Overshoot and slow settling | Kd too low | Increase Kd |
| High-frequency noise / buzzing | Kd too high | Reduce Kd |
