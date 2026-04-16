#!/usr/bin/env bash

if [ "$EUID" -ne 0 ]; then
  echo "Run with sudo"
  exit 1
fi

# Detectar usuario real (no root)
REAL_USER=${SUDO_USER:-$(whoami)}
USER_HOME=$(eval echo "~$REAL_USER")

# Detectar ruta del repo (donde se ejecuta el script)
BASE_PATH=$(cd "$(dirname "$0")" && pwd)

echo "Installing for user: $REAL_USER"
echo "Home: $USER_HOME"
echo "Base path: $BASE_PATH"

# Crear servicio dinámico
sed -e "s|{{USER_HOME}}|$USER_HOME|g" \
    -e "s|{{BASE_PATH}}|$BASE_PATH|g" \
    services/robot@.service > /etc/systemd/system/robot@.service

# Copiar target
install -m 644 services/robotSystem.target /etc/systemd/system/

# Recargar systemd
systemctl daemon-reload

# Habilitar
systemctl enable robotSystem.target

echo "Installed successfully"