set dotenv-load

[private]
default:
    @just --list --unsorted

[private]
alias husarnet := connect-husarnet
[private]
alias flash := flash-firmware
[private]
alias rosbot := start-rosbot

[private]
pre-commit:
    #!/bin/bash
    if ! command -v pre-commit &> /dev/null; then
        pip install pre-commit
        pre-commit install
    fi
    pre-commit run -a

# [PC/ROSbot] connect to Husarnet VPN network
connect-husarnet joincode hostname: _run-as-root
    #!/bin/bash
    if ! command -v husarnet > /dev/null; then
        echo "Husarnet is not installed. Installing now..."
        curl https://install.husarnet.com/install.sh | bash
    fi
    husarnet join {{joincode}} {{hostname}}

# [PC] Copy repo content to remote host with 'rsync' and watch for changes
sync hostname="${SYNC_HOSTNAME}" password="husarion": _install-rsync _run-as-user
    #!/bin/bash
    sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    while inotifywait -r -e modify,create,delete,move ./ --exclude='.git/' ; do
        sshpass -p "{{password}}" rsync -vRr --exclude='.git/' --delete ./ husarion@{{hostname}}:/home/husarion/${PWD##*/}
    done

# [ROSbot] flash the proper firmware for STM32 microcontroller in ROSbot XL
flash-firmware: _install-yq _run-as-user
    #!/bin/bash
    echo "Stopping all running containers"
    docker ps -q | xargs -r docker stop

    echo "Flashing the firmware for STM32 microcontroller in ROSbot"
    docker run \
        --rm -it \
        --device /dev/ttyUSBDB \
        --device /dev/bus/usb/ \
        $(yq .services.rosbot.image compose.yaml) \
        ros2 run rosbot_xl_utils flash_firmware --port /dev/ttyUSBDB
        # flash-firmware.py -p /dev/ttyUSBDB # todo

# [ROSbot] setup udevs for managing Movidius USB device permissions
oak-udev:
    #!/bin/bash
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger


# [ROSbot] start containers on a physical ROSbot XL
start-rosbot: _run-as-user
    #!/bin/bash
    mkdir -p maps
    docker compose down
    # docker compose pull
    docker compose up

# [PC] start RViz
start-pc: _run-as-user
    #!/bin/bash
    xhost +local:docker
    docker compose -f compose.pc.yaml up


# [PC/ROSbot] optimize DDS settings; Use if you experience stability issues.
dds-tunning:
    #!/bin/bash

    # https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/large_data/large_data.html#
    # https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html
    sudo sysctl -w net.core.wmem_max=12582912
    sudo sysctl -w net.core.rmem_max=12582912
    sudo sysctl -w net.core.wmem_default=16384000
    sudo sysctl -w net.core.rmem_default=16384000
    sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728     # (128 MB)
    sudo sysctl -w net.ipv4.ipfrag_time=3
    sudo sysctl -w net.ipv6.ip6frag_time=3 # 3s
    sudo sysctl -w net.ipv6.ip6frag_high_thresh=134217728 # (128 MB)
    sudo ip link set txqueuelen 500 dev hnet0
    sudo ip link set dev hnet0 mtu 1350
    # sudo ip link set dev hnet0 mtu 9000


_run-as-root:
    #!/bin/bash
    if [ "$EUID" -ne 0 ]; then
        echo -e "\e[1;33mPlease re-run as root user to install dependencies\e[0m"
        exit 1
    fi

_run-as-user:
    #!/bin/bash
    if [ "$EUID" -eq 0 ]; then
        echo -e "\e[1;33mPlease re-run as non-root user\e[0m"
        exit 1
    fi

_install-rsync:
    #!/bin/bash
    if ! command -v rsync &> /dev/null || ! command -v sshpass &> /dev/null || ! command -v inotifywait &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit 1
        fi
        apt install -y rsync sshpass inotify-tools
    fi

_install-yq:
    #!/bin/bash
    if ! command -v /usr/bin/yq &> /dev/null; then
        if [ "$EUID" -ne 0 ]; then
            echo -e "\e[1;33mPlease run as root to install dependencies\e[0m"
            exit 1
        fi

        YQ_VERSION=v4.35.1
        ARCH=$(arch)

        if [ "$ARCH" = "x86_64" ]; then
            YQ_ARCH="amd64"
        elif [ "$ARCH" = "aarch64" ]; then
            YQ_ARCH="arm64"
        else
            YQ_ARCH="$ARCH"
        fi

        curl -L https://github.com/mikefarah/yq/releases/download/${YQ_VERSION}/yq_linux_${YQ_ARCH} -o /usr/bin/yq
        chmod +x /usr/bin/yq
        echo "yq installed successfully!"
    fi