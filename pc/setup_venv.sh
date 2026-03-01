#!/usr/bin/env bash
# =============================================================================
# Script  : setup_venv.sh
# Project : Tang Nano 9K – PID Motor Controller over Ethernet
# File    : pc/setup_venv.sh
#
# Purpose : Creates an isolated Python virtual environment in pc/.venv and
#           installs all required packages.  Run ONCE before using the
#           dashboard for the first time.
#
# Usage
#   cd pc && bash setup_venv.sh
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"

echo "======================================================"
echo "  PID Monitor – Python venv setup"
echo "======================================================"

# --- Check Python 3 is available ------------------------------------------
if ! command -v python3 &>/dev/null; then
    echo "ERROR: python3 not found. Install it first:"
    echo "  sudo apt install python3 python3-venv python3-pip"
    exit 1
fi

echo "Using Python: $(python3 --version)"
echo "Creating venv at: ${VENV_DIR}"

# --- Create venv -----------------------------------------------------------
python3 -m venv "${VENV_DIR}"

# --- Activate and install --------------------------------------------------
source "${VENV_DIR}/bin/activate"

echo "Upgrading pip..."
pip install --upgrade pip --quiet

echo "Installing requirements from pc/requirements.txt..."
pip install -r "${SCRIPT_DIR}/requirements.txt"

echo ""
echo "======================================================"
echo "  Setup complete."
echo "  Run the dashboard with:  cd pc && bash run.sh"
echo "======================================================"
