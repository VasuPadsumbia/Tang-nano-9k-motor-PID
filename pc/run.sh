#!/usr/bin/env bash
# =============================================================================
# Script  : run.sh
# Project : Tang Nano 9K – PID Motor Controller over Ethernet
# File    : pc/run.sh
#
# Purpose : Activates the local Python venv and launches the PID monitor
#           dashboard.  Ensures the dashboard never touches the system
#           Python interpreter or any globally installed packages.
#
# Usage
#   cd pc && bash run.sh [--fpga-ip 10.10.10.100] [--port 5005]
#
# First-time setup
#   cd pc && bash setup_venv.sh
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"
ACTIVATE="${VENV_DIR}/bin/activate"

# Check venv exists
if [ ! -f "${ACTIVATE}" ]; then
    echo "ERROR: venv not found at ${VENV_DIR}"
    echo "Run setup first:  cd pc && bash setup_venv.sh"
    exit 1
fi

# Activate venv (uses the venv Python, NOT the system interpreter)
source "${ACTIVATE}"

echo "Python: $(which python3)  ($(python3 --version))"
echo "Starting PID monitor dashboard..."
echo ""

# Pass all script arguments through to pid_monitor.py
python3 "${SCRIPT_DIR}/pid_monitor.py" "$@"
