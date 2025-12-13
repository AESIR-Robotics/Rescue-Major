#!/usr/bin/env bash
set -e

# --- Detectar versión de Ubuntu ---
source /etc/os-release
UBUNTU_VERSION="$VERSION_CODENAME"

echo "Detectando versión de Ubuntu..."
echo "  VERSION_ID = $VERSION_ID"
echo "  CODENAME   = $UBUNTU_VERSION"
sleep 1

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

# --- Función para instalar ROS2 Humble en Ubuntu 22.04 ---
install_jazzy() {
    echo "=== Instalando ROS 2 Humble (Ubuntu 22.04 - jammy) ==="

    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb

    sudo apt update
    sudo apt upgrade

    sudo apt install ros-jazzy-desktop

    echo "=== Instalación de ROS 2 Humble completada ==="
}

# --- Función para instalar ROS2 Jazzy en Ubuntu 24.04 ---
install_humble() {
    echo "=== Instalando ROS 2 Jazzy (Ubuntu 24.04 - noble) ==="

    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb

    sudo apt update
    sudo apt upgrade

    sudo apt install ros-humble-desktop

    echo "=== Instalación de ROS 2 Humble completada ==="
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
