#!/usr/bin/env bash
set -euo pipefail

WORKDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$WORKDIR/workspace"
VENV_PARENT_DIR="$(dirname "$WORKSPACE_DIR")"
VENV_DIR="$VENV_PARENT_DIR/.venv"

echo "[setup] Project dir: $WORKDIR"
echo "[setup] Workspace dir: $WORKSPACE_DIR"
echo "[setup] Venv dir: $VENV_DIR"
 
#  Install tmux and rosbridge-server (apt)
echo "[apt] Updating package lists..."
sudo apt-get update
sudo apt install -y libwebsocketpp-dev libboost-all-dev libssl-dev tmux ros-humble-rosbridge-server


# Determine real user (if script run via sudo, SUDO_USER is original)
if [ "$EUID" -eq 0 ] && [ -n "${SUDO_USER:-}" ]; then
  REAL_USER="$SUDO_USER"
else
  REAL_USER="$(id -un)"
fi

# Create (if needed) venv in workspace and activate it
if [ ! -d "$WORKSPACE_DIR" ]; then
  echo "[error] Workspace directory $WORKSPACE_DIR does not exist. Create it or run this script from project root."
  exit 1
fi

# Ensure python3 venv support is available (ensurepip)
if ! python3 -c "import ensurepip" >/dev/null 2>&1; then
  echo "[venv] ensurepip not available — installing python3-venv (requires sudo)"
  sudo apt-get update
  if ! sudo apt-get install -y python3-venv; then
    echo "[error] Failed to install python3-venv. Please install it manually and re-run."
    exit 1
  fi
fi

if [ ! -d "$VENV_DIR" ]; then
  echo "[venv] Creating isolated venv at $VENV_DIR (no system-site-packages)"
  if ! python3 -m venv "$VENV_DIR"; then
    echo "[venv] venv creation failed. Attempting to install python3-venv and retry..."
    sudo apt-get update
    sudo apt-get install -y python3-venv || { echo "[error] Failed to install python3-venv"; exit 1; }
    python3 -m venv "$VENV_DIR" || { echo "[error] venv creation failed after installing python3-venv"; exit 1; }
  fi
  echo "[venv] created"
else
  echo "[venv] Found existing venv at $VENV_DIR"
fi

# Activate venv for the rest of the script
# shellcheck source=/dev/null
source "$VENV_DIR/bin/activate"

echo "[venv] Activated: $(which python) ($(python --version))"

# Install pip requirements found under workspace
echo "[pip] Upgrading pip and wheel inside venv"
python -m pip install --upgrade pip wheel
echo "[pip] Ensuring compatible packaging and setuptools versions (setuptools<80, packaging>=25)"
python -m pip install --upgrade "packaging>=25.0" "setuptools<80,>=30.3"

# Find requirements files under workspace (depth 4 to be safe)
mapfile -t REQ_FILES < <(find "$WORKSPACE_DIR" -maxdepth 4 -type f -iname "requirements*.txt" 2>/dev/null | sort -u)

if [ ${#REQ_FILES[@]} -eq 0 ]; then
  echo "[pip] No requirements files found under $WORKSPACE_DIR — skipping pip installs"
else
  echo "[pip] Found requirements files:"
  for f in "${REQ_FILES[@]}"; do
    echo "  - $f"
  done

  for req in "${REQ_FILES[@]}"; do
    echo "[pip] Installing from $req"
    python -m pip install -r "$req"
  done
fi

# Final summary
echo ""
echo "[done] Setup complete. Summary:"

if [ ${#REQ_FILES[@]} -gt 0 ]; then
  echo " - Installed Python requirements from:"
  for f in "${REQ_FILES[@]}"; do echo "    $f"; done
else
  echo " - No Python requirements installed"
fi

echo "To activate this environment in a new shell run:"
echo "  source $VENV_DIR/bin/activate"
return 0 2>/dev/null || exit 0
