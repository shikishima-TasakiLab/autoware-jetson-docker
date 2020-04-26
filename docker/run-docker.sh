#!/bin/bash
CONTAINER_NAME="autoware"
AUTOWARE_VERSION="1.13.0"
AUTOWARE_LAUNCH="on"

PROG_NAME=$(basename $0)

function usage_exit {
  cat <<_EOS_ 1>&2
  Usage: $PROG_NAME [OPTIONS...]
  OPTIONS:
    -h, --help                      このヘルプを表示
    -l, --launch {on|off}           runtime_managerの起動（既定値：${AUTOWARE_LAUNCH}）
    -v, --version AUTOWARE_VERSION  Autowareのバージョンを指定
    -n, --name NAME                 コンテナの名前を指定
_EOS_
    cd ${CURRENT_DIR}
    exit 1
}

while (( $# > 0 )); do
    if [[ $1 == "--help" ]] || [[ $1 == "-h" ]]; then
        usage_exit
    elif [[ $1 == "--launch" ]] || [[ $1 == "-l" ]]; then
        if [[ $2 != "on" ]] && [[ $2 != "off" ]]; then
            echo "無効なパラメータ： $1 $2"
            usage_exit
        fi
        AUTOWARE_LAUNCH=$2
        shift 2
    elif [[ $1 == "--version" ]] || [[ $1 == "-v" ]]; then
        if [[ $2 == -* ]] || [[ $2 == *- ]]; then
            echo "無効なパラメータ： $1 $2"
            usage_exit
        fi
        AUTOWARE_VERSION=$2
        shift 2
    elif [[ $1 == "--name" ]] || [[ $1 == "-n" ]]; then
        if [[ $2 == -* ]] || [[ $2 == *- ]]; then
            echo "無効なパラメータ： $1 $2"
            usage_exit
        fi
        CONTAINER_NAME=$2
        shift 2
    else
        echo "無効なパラメータ： $1"
        usage_exit
    fi
done

DOCKER_IMAGE="jetson/autoware:${AUTOWARE_VERSION}"

XSOCK="/tmp/.X11-unix"
XAUTH="/tmp/.docker.xauth"

HOST_WS=$(dirname $(dirname $(readlink -f $0)))/shared_dir

DOCKER_VOLUME="-v ${XSOCK}:${XSOCK}:rw"
DOCKER_VOLUME="${DOCKER_VOLUME} -v ${XAUTH}:${XAUTH}:rw"
DOCKER_VOLUME="${DOCKER_VOLUME} -v ${HOST_WS}:/home/ros/shared_dir:rw"

DOCKER_ENV="-e XAUTHORITY=${XAUTH}"
DOCKER_ENV="${DOCKER_ENV} -e DISPLAY=$DISPLAY"
DOCKER_ENV="${DOCKER_ENV} -e TERM=xterm-256color"
DOCKER_ENV="${DOCKER_ENV} -e QT_X11_NO_MITSHM=1"

DOCKER_NET="host"

DOCKER_CMD=""
if [[ ${AUTOWARE_LAUNCH} == "on" ]]; then
    DOCKER_CMD="roslaunch /home/ros/autoware.ai/runtime_manager.launch"
fi

touch ${XAUTH}
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f ${XAUTH} nmerge -

docker run \
    --rm \
    -it \
    --gpus all \
    --privileged \
    --name ${CONTAINER_NAME} \
    --net ${DOCKER_NET} \
    ${DOCKER_ENV} \
    ${DOCKER_VOLUME} \
    ${DOCKER_IMAGE} \
    ${DOCKER_CMD}
