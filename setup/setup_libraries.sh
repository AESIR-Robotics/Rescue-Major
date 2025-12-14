#!/usr/bin/env bash
set -e

# Helpers
command_exists() { command -v "$1" >/dev/null 2>&1; }

apt_install_if_missing() {
	local pkgs=("$@")
	local missing=()
	for p in "${pkgs[@]}"; do
		if ! dpkg -s "$p" >/dev/null 2>&1; then
			missing+=("$p")
		fi
	done
	if [ ${#missing[@]} -gt 0 ]; then
		echo "Instalando paquetes apt faltantes: ${missing[*]}"
		sudo apt update
		sudo apt install -y "${missing[@]}"
	else
		echo "Paquetes apt ya instalados: ${pkgs[*]}"
	fi
}

pip_install_if_missing() {
	local pkgs=("$@")
	local missing=()
	for p in "${pkgs[@]}"; do
		if ! python3 -m pip show "$p" >/dev/null 2>&1; then
			missing+=("$p")
		fi
	done
	if [ ${#missing[@]} -gt 0 ]; then
		echo "Instalando paquetes pip faltantes: ${missing[*]}"
		python3 -m pip install "${missing[@]}"
	else
		echo "Paquetes pip ya instalados: ${pkgs[*]}"
	fi
}

cd ..
# Crear y activar venv si no existe
if [ ! -d venv ]; then
	python3 -m venv venv
fi
source venv/bin/activate

# Pip packages básicos
pip_install_if_missing em lark

# Vision/python libs
pip_install_if_missing aiohttp aiortc opencv-python av2 catkin-pkg

# vision: zbar y pyzbar
apt_install_if_missing libzbar0
pip_install_if_missing pyzbar ultralytics DateTime websockets

# Dependencias del sistema
apt_install_if_missing libwebsocketpp-dev libboost-all-dev libssl-dev tmux ros-humble-cv-bridge libopencv-dev

#Websockets