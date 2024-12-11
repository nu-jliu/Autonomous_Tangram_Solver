# Autonomous Tangram Solver

**Author:** Allen Liu

This repo contains the packages necessary for a robot arm to autonomously picks up tangram pieces and solves the tangram puzzle.

## Final Video
<iframe width="800" height="560" src="https://www.youtube.com/embed/1sL6v5JUFx0?si=-zl5taqeXtQbnnAh" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Install Dependencies

*Boost*

```bash
sudo apt install libbost-all-dev
```

*OpenCV*

```bash
sudo apt install libopencv-dev
```

*ROS2 Iron*

Configure locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

Add universe repo

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add ROS2 Key

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ros dev tooks

```bash
sudo apt update && sudo apt install ros-dev-tools
```

Upgrade machine (Caution)

```bash
sudo apt update && sudo apt upgrade
```

Install ROS2 Iron Desktop

```bash
sudo apt install ros-iron-desktop-full
```

## Building all packages

```bash
export WORKSPACE=<path/to/your/workspace>

mkdir -p ${WORKSPACE}/src
cd ${WORKSPACE}
vcs import --input src < https://raw.githubusercontent.com/nu-jliu/Autonomous_Tangram_Solver/refs/heads/main/tangram.repos
```