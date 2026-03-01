# =============================================================================
# Makefile  : Tang Nano 9K – PID Motor Controller over Ethernet
# Purpose   : Shortcuts for running tests locally. CI uses these too.
#
# Usage:
#   make test        - Run all simulations
#   make test-pid    - Run PID unit test
#   make test-pwm    - Run PWM unit test
#   make test-top    - Run top-level integration test
#   make build       - Run apio build
#   make upload      - Run apio upload (flash to board)
#   make clean       - Remove generated files
# =============================================================================

.PHONY: all test test-pid test-pwm test-top build upload clean

all: test build

data:
	mkdir -p data

test-pid: data
	@echo "Running PID Unit Test..."
	iverilog -g2012 -o data/sim_pid tb/tb_pid_controller.v rtl/motor/pid_controller.v
	vvp data/sim_pid

test-pwm: data
	@echo "Running PWM Unit Test..."
	iverilog -g2012 -o data/sim_pwm tb/tb_pwm_gen.v rtl/motor/pwm_gen.v
	vvp data/sim_pwm

test-top: data
	@echo "Running Top-Level Integration Test..."
	iverilog -g2012 -Wall -o data/sim_top \
	  tb/tb_top.v rtl/top.v \
	  rtl/sys/clk_div.v \
	  rtl/motor/pid_controller.v rtl/motor/motor_bridge.v \
	  rtl/motor/pwm_gen.v rtl/motor/encoder_reader.v \
	  rtl/comm/rmii_rx.v rtl/comm/rmii_tx.v rtl/comm/eth_crc32.v \
	  rtl/comm/eth_rx_parser.v rtl/comm/eth_tx_builder.v \
	  rtl/comm/arp_handler.v rtl/comm/udp_ctrl_rx.v rtl/comm/udp_telem_tx.v
	vvp data/sim_top

test: test-pid test-pwm test-top

build:
	apio build

upload:
	apio upload

clean:
	rm -rf data/*.vcd data/sim_* data/*.log hardware.fs hardware.v
	apio clean
