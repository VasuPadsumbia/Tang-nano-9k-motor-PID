# IDE & Toolchain Setup Guide

## Tang Nano 9K — PID Motor Controller over Ethernet

---

## Required Tools

| Tool | Purpose | Install Method |
|------|---------|----------------|
| **APIO** | FPGA build (Yosys + nextpnr-gowin) | `pip install apio` |
| **OSS CAD Suite** | Gowin toolchain binaries | via `apio install` |
| **Icarus Verilog** | RTL simulation | `sudo apt install iverilog` |
| **GTKWave** | Waveform viewer | `sudo apt install gtkwave` |
| **VS Code** | IDE | [code.visualstudio.com](https://code.visualstudio.com) |
| **Python 3.10+** | PC dashboard | `sudo apt install python3 python3-venv` |

---

## Step 1: Install APIO & Gowin Toolchain

```bash
# Install APIO into your user Python (not root)
pip install --user apio==0.9.*

# Download and install the Gowin toolchain + programmer
apio install --all

# Verify
apio --version
```

> **Note**: `apio install --all` automatically downloads OSS Cad Suite
> (Yosys, nextpnr-gowin, openFPGALoader). No separate Gowin IDE needed.

---

## Step 2: Install Simulation Tools

```bash
sudo apt update
sudo apt install iverilog gtkwave
iverilog -V   # verify
```

---

## Step 3: VS Code Extensions

Install these from the Extensions panel (`Ctrl+Shift+X`):

| Extension ID | Purpose |
|-------------|---------|
| `teros-hdl.teroshdl` | **TerosHDL** – Verilog lint, docs, state machine viewer |
| `mshr-k.veriloghdl` | Verilog syntax highlighting + ctags navigation |
| `apio-ide.apio-ide` | **APIO IDE** – one-click build, upload, clean |
| `ms-python.python` | Python for PC dashboard |
| `ms-python.vscode-pylance` | Python type checking |

> Install TerosHDL first — it pulls in most Verilog support automatically.

### Recommended VS Code settings (`.vscode/settings.json`):

```json
{
    "verilog.linting.linter": "iverilog",
    "verilog.linting.iverilog.arguments": "-g2012 -I rtl/",
    "teroshdl.documentation.projectName": "Tang Nano 9K PID Motor Controller",
    "python.defaultInterpreterPath": "${workspaceFolder}/pc/.venv/bin/python3"
}
```

---

## Step 4: Build the Project

```bash
cd /home/vasu-usb/Tang-nano-9k-motor-PID

# Synthesise, place & route (produces bitstream)
apio build

# Flash to Tang Nano 9K (must be connected via USB)
apio upload

# Lint / syntax check without full build
apio verify
```

---

## Step 5: Run Simulation Testbenches

All simulation outputs are saved to `data/`.

```bash
# --- Unit test: PID controller ---
iverilog -g2012 -o data/sim_pid \
  tb/tb_pid_controller.v \
  rtl/motor/pid_controller.v
vvp data/sim_pid
gtkwave data/sim_pid.vcd &

# --- Unit test: PWM generator ---
iverilog -g2012 -o data/sim_pwm \
  tb/tb_pwm_gen.v \
  rtl/motor/pwm_gen.v
vvp data/sim_pwm
gtkwave data/sim_pwm.vcd &

# --- Integration test: Top-level + Communication ---
iverilog -g2012 -o data/sim_top \
  tb/tb_top.v rtl/top.v \
  rtl/sys/clk_div.v \
  rtl/motor/pid_controller.v rtl/motor/motor_bridge.v \
  rtl/motor/pwm_gen.v rtl/motor/encoder_reader.v \
  rtl/comm/rmii_rx.v rtl/comm/rmii_tx.v rtl/comm/eth_crc32.v \
  rtl/comm/eth_rx_parser.v rtl/comm/eth_tx_builder.v \
  rtl/comm/arp_handler.v rtl/comm/udp_ctrl_rx.v rtl/comm/udp_telem_tx.v
vvp data/sim_top
gtkwave data/sim_top.vcd &
```

---

## Step 6: PC Dashboard (in venv)

```bash
# One-time setup
cd pc && bash setup_venv.sh

# Run dashboard (connects to FPGA at 10.10.10.100:5005)
bash run.sh

# Custom IP / port
bash run.sh --fpga-ip 10.10.10.100 --port 5005
```

> Set your PC NIC to static IP `10.10.10.10` / subnet `255.255.255.0`
> before launching the dashboard.

---

## Troubleshooting

| Issue | Likely Cause | Fix |
|-------|-------------|-----|
| `apio: command not found` | Not in PATH | `export PATH=$HOME/.local/bin:$PATH` |
| `apio upload` fails | USB not detected | `sudo usermod -aG plugdev $USER` then re-login |
| `iverilog: command not found` | Not installed | `sudo apt install iverilog` |
| FPGA not responding to ping | Wrong subnet | Set PC NIC to `10.10.10.x/24` |
| No telemetry in dashboard | ARP not resolved | Check LAN8720 wiring; re-run dashboard |
