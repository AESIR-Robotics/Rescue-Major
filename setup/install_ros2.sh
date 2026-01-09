#!/usr/bin/env bash
set -e

# --- Helpers para instalaciones idempotentes ---
command_exists() { command -v "$1" >/dev/null 2>&1; }

apt_install_if_missing() {
    # usa dpkg para comprobar paquetes apt por nombre
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

# --- Detectar versión de Ubuntu ---
source /etc/os-release
UBUNTU_VERSION="$VERSION_CODENAME"

echo "Detectando versión de Ubuntu..."
echo "  VERSION_ID = $VERSION_ID"
echo "  CODENAME   = $UBUNTU_VERSION"
sleep 1

locale  # check for UTF-8

# Solo instalar locales si faltan
apt_install_if_missing locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# software-properties-common y universe
apt_install_if_missing software-properties-common
sudo add-apt-repository universe || true

# --- Función para instalar ROS2 Humble en Ubuntu 22.04 ---
install_jazzy() {
    echo "=== Instalando ROS (función install_jazzy) ==="

    if command_exists ros2; then
        echo "ros2 ya está instalado — se omite la instalación."
        return 0
    fi

    apt_install_if_missing curl
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb || true

    sudo apt update
    sudo apt upgrade -y

    # instalar el metapaquete correspondiente si falta
    apt_install_if_missing ros-jazzy-desktop

    echo "=== Instalación de ROS (install_jazzy) completada ==="
}

# --- Función para instalar ROS2 Jazzy en Ubuntu 24.04 ---
install_humble() {
    echo "=== Instalando ROS (función install_humble) ==="

    if command_exists ros2; then
        echo "ros2 ya está instalado — se omite la instalación."
        return 0
    fi

    apt_install_if_missing curl
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb || true

    sudo apt update
    sudo apt upgrade -y

    # instalar el metapaquete correspondiente si falta
    apt_install_if_missing ros-humble-desktop

    echo "=== Instalación de ROS (install_humble) completada ==="
}

# --- Lógica de selección ---
if [[ "$UBUNTU_VERSION" == "jammy" ]]; then
    install_humble

elif [[ "$UBUNTU_VERSION" == "noble" ]]; then
    install_jazzy

else
    echo "Tu Ubuntu ($UBUNTU_VERSION) no es compatible con ROS Humble (22.04) ni Jazzy (24.04)."
    echo "   Versión soportadas:"
    echo "     • Ubuntu 22.04 → ROS 2 Humble"
    echo "     • Ubuntu 24.04 → ROS 2 Jazzy"
    exit 1
fi
