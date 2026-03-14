#!/usr/bin/env bash

WORKDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$WORKDIR/workspace"
VENV_PARENT_DIR="$(dirname "$WORKSPACE_DIR")"
VENV_DIR="$VENV_PARENT_DIR/.venv"

echo "[setup] Project dir: $WORKDIR"
echo "[setup] Workspace dir: $WORKSPACE_DIR"
echo "[setup] Venv dir: $VENV_DIR"
 
# ------------------------------
# Install external dependencies 
# ------------------------------
echo "[apt] Updating package lists..."
sudo apt-get update

# Read apt packages from requirements_lib.txt
LIB_REQ_FILE="$WORKDIR/requirements_lib.txt"
if [ -f "$LIB_REQ_FILE" ]; then
  echo "[apt] Installing packages from $LIB_REQ_FILE..."
  # Read non-empty, non-comment lines and install them all at once
  mapfile -t APT_PACKAGES < <(grep -v '^\s*#' "$LIB_REQ_FILE" | grep -v '^\s*$' | tr -s ' \n' '\n')
  if [ ${#APT_PACKAGES[@]} -gt 0 ]; then
    sudo apt-get install -y "${APT_PACKAGES[@]}"
  else
    echo "[apt] No packages found in $LIB_REQ_FILE"
  fi
else
  echo "[apt] $LIB_REQ_FILE not found — skipping apt installs"
fi

# Determine real user (if script run via sudo, SUDO_USER is original)
if [ "$EUID" -eq 0 ] && [ -n "${SUDO_USER:-}" ]; then
  REAL_USER="$SUDO_USER"
else
  REAL_USER="$(id -un)"
fi

# ------------------
# python enviroment
# ------------------
# Create (if needed) venv in workspace and activate it
if [ ! -d "$WORKSPACE_DIR" ]; then
  echo "[error] Workspace directory $WORKSPACE_DIR does not exist. Create it or run this script from project root."
  exit 0
fi

# Ensure python3 venv support is available (ensurepip)
if ! python3 -c "import ensurepip" >/dev/null 2>&1; then
  echo "[venv] ensurepip not available — installing python3-venv (requires sudo)"
  sudo apt-get update
  if ! sudo apt-get install -y python3-venv; then
    echo "[error] Failed to install python3-venv. Please install it manually and re-run."
    exit 0
  fi
fi

if [ ! -d "$VENV_DIR" ]; then
  echo "[venv] Creating venv at $VENV_DIR "
  if ! python3 -m venv --system-site-packages "$VENV_DIR"; then
    echo "[venv] venv creation failed. Attempting to install python3-venv and retry..."
    sudo apt-get update
    sudo apt-get install -y python3-venv || { echo "[error] Failed to install python3-venv"; exit 1; }
    python3 -m venv --system-site-packages "$VENV_DIR" || { echo "[error] venv creation failed after installing python3-venv"; exit 1; }
  fi
  echo "[venv] created "
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

# Find requirements_python.txt files under workspace (depth 4 to be safe)
mapfile -t REQ_FILES < <(find "${WORKDIR}" -maxdepth 4 -type f -iname "requirements_python.txt" 2>/dev/null | sort -u)

if [ ${#REQ_FILES[@]} -eq 0 ]; then
  echo "[pip] No requirements_python.txt files found under ${WORKDIR} — skipping pip installs"
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

# ------------------
# Clone ROS repos
# ------------------
SRC_DIR="$WORKSPACE_DIR/src"
if [ ! -d "$SRC_DIR" ]; then
  echo "[git] Creating $SRC_DIR directory..."
  mkdir -p "$SRC_DIR"
fi

VISION_OPENCV_DIR="$SRC_DIR/vision_opencv"
if [ ! -d "$VISION_OPENCV_DIR" ]; then
  echo "[git] Cloning vision_opencv (branch: humble) into $SRC_DIR..."
  git clone -b humble https://github.com/ros-perception/vision_opencv.git "$VISION_OPENCV_DIR"
  echo "[git] vision_opencv cloned successfully"
else
  echo "[git] vision_opencv already exists at $VISION_OPENCV_DIR — skipping clone"
fi

# -------------------------------------------------------------------------
# Compile ROS workspace (if compile.sh or compile.bash found in workspace)
# -------------------------------------------------------------------------

COMPILE_SCRIPT=""

if [ -f "$WORKSPACE_DIR/compile.sh" ]; then
  COMPILE_SCRIPT="$WORKSPACE_DIR/compile.sh"
fi

if [ -n "$COMPILE_SCRIPT" ]; then
  echo "[build] Found: $COMPILE_SCRIPT"
  chmod +x "$COMPILE_SCRIPT" || true

  echo "[build] Running compile script from workspace directory..."
  (
    cd "$WORKSPACE_DIR"
    # run with bash explicitly to avoid "exec format" issues
    bash "$(basename "$COMPILE_SCRIPT")"
  )

  echo "[build] Build finished."
else
  echo "[build] No compile.sh or compile.bash found in: $WORKSPACE_DIR"
  echo "[build] Skipping build step."
fi

# ---------------------------------------------
# SSL Certificate Generation for WebRTC HTTPS
# ---------------------------------------------
CERT_FILE="$WORKDIR/cert.pem"
KEY_FILE="$WORKDIR/key.pem"

echo "[ssl] Checking SSL certificate..."
if [ ! -f "$CERT_FILE" ] || [ ! -f "$KEY_FILE" ]; then
  echo "[ssl] Generating self-signed SSL certificate (valid for 365 days)..."
  openssl req -x509 -newkey rsa:4096 \
    -keyout "$KEY_FILE" \
    -out "$CERT_FILE" \
    -days 365 -nodes \
    -subj "/C=US/ST=State/L=City/O=Aesir/OU=Teleoperation/CN=localhost" \
    2>/dev/null
  
  # Set ownership to real user (in case script run with sudo)
  if [ "$EUID" -eq 0 ] && [ -n "${SUDO_USER:-}" ]; then
    chown "$REAL_USER:$REAL_USER" "$CERT_FILE" "$KEY_FILE"
  fi
  
  echo "[ssl] Certificate created"
else
  echo "[ssl] Certificate already exists"
fi

# Firewall Configuration 
echo ""
echo "[firewall] Configuring firewall rules..."
if command -v ufw >/dev/null 2>&1; then
  # Check if ufw is active
  if sudo ufw status | grep -q "Status: active"; then
    echo "[firewall] UFW is active, adding rules..."
    sudo ufw allow 8081/tcp comment 'WebRTC HTTPS Server' >/dev/null 2>&1 || echo "[firewall] Rule for 8081 already exists"
    sudo ufw allow 9090/tcp comment 'Rosbridge WebSocket' >/dev/null 2>&1 || echo "[firewall] Rule for 9090 already exists"
    echo "[firewall] Ports 8081 (HTTPS) and 9090 (WebSocket) opened"
  else
    echo "[firewall] FW is inactive."
  fi
else
  echo "[firewall] UFW not installed."
fi

# Final summary
echo ""
echo "[done] Setup complete. Summary:"

if [ -f "$LIB_REQ_FILE" ]; then
  echo " - Installed apt packages from: $LIB_REQ_FILE"
else
  echo " - No apt packages installed (requirements_lib.txt not found)"
fi

if [ ${#REQ_FILES[@]} -gt 0 ]; then
  echo " - Installed Python requirements from:"
  for f in "${REQ_FILES[@]}"; do echo "    $f"; done
else
  echo " - No Python requirements installed"
fi

echo " - Cloned vision_opencv into: $SRC_DIR"

deactivate
echo "To activate this environment in a new shell run:"
echo "  source $VENV_DIR/bin/activate"