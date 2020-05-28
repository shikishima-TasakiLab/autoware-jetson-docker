#!/bin/bash
CONTAINER_NAME="autoware"
AUTOWARE_VERSION="1.14.0"
AUTOWARE_LAUNCH="on"

PROG_NAME=$(basename $0)
RUN_DIR=$(dirname $(readlink -f $0))

PARAM_YML="${RUN_DIR}/autoware-param/param_init.yaml"
SAVE_PATH=""

ntr_arr=( $(echo $(cat /etc/nv_tegra_release) | tr -s ',' ' ') )
MAJOR_VERSION=${ntr_arr[1]}
MINOR_VERSION=${ntr_arr[4]}

function usage_exit {
  cat <<_EOS_ 1>&2
  Usage: $PROG_NAME [OPTIONS...]
  OPTIONS:
    -h, --help                      このヘルプを表示
    -l, --launch {on|off}           runtime_managerの起動（既定値：${AUTOWARE_LAUNCH}）
    -v, --version AUTOWARE_VERSION  Autowareのバージョンを指定
    -p, --param FILE                読み込むAutowareの設定ファイルを指定
    -s, --save FILE                 Autowareの設定ファイルの保存先を指定
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
    elif [[ $1 == "--param" ]] || [[ $1 == "-p" ]]; then
        if [[ -f $2 ]]; then
            PARAM_YML=$2
        else
            echo "無効なパラメータ： $1 $2"
            usage_exit
        fi
        shift 2
    elif [[ $1 == "--save" ]] || [[ $1 == "-s" ]]; then
        if [[ -d $(dirname $(readlink -f $2)) ]]; then
            SAVE_PATH=$2
        else
            echo "無効なパラメータ： $1 $2"
            usage_exit
        fi
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

DOCKER_IMAGE="jetson/autoware:${MAJOR_VERSION,,}.${MINOR_VERSION}-${AUTOWARE_VERSION}"

XSOCK="/tmp/.X11-unix"
XAUTH="/tmp/.docker.xauth"

HOST_WS=$(dirname $(dirname $(readlink -f $0)))/catkin_ws
HOST_SH=$(dirname $(dirname $(readlink -f $0)))/shared_dir
cp ${PARAM_YML} ${RUN_DIR}/src-autoware/param.yaml

DOCKER_VOLUME="-v ${XSOCK}:${XSOCK}:rw"
DOCKER_VOLUME="${DOCKER_VOLUME} -v ${XAUTH}:${XAUTH}:rw"
DOCKER_VOLUME="${DOCKER_VOLUME} -v ${RUN_DIR}/src-autoware/param.yaml:/home/ros/autoware.ai/install/runtime_manager/lib/runtime_manager/param.yaml:rw"
DOCKER_VOLUME="${DOCKER_VOLUME} -v ${HOST_WS}:/home/ros/catkin_ws:rw"
DOCKER_VOLUME="${DOCKER_VOLUME} -v ${HOST_SH}:/home/ros/shared_dir:rw"

DOCKER_ENV="-e XAUTHORITY=${XAUTH}"
DOCKER_ENV="${DOCKER_ENV} -e DISPLAY=$DISPLAY"
DOCKER_ENV="${DOCKER_ENV} -e TERM=xterm-256color"

DOCKER_NET="host"

DOCKER_CMD=""
if [[ ${AUTOWARE_LAUNCH} == "on" ]]; then
    DOCKER_CMD="roslaunch runtime_manager runtime_manager.launch"
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

if [[ -f ${RUN_DIR}/src-autoware/param.yaml ]]; then
    if [[ ${SAVE_PATH} != "" ]]; then
        cp ${RUN_DIR}/src-autoware/param.yaml ${SAVE_PATH}
    fi
    rm ${RUN_DIR}/src-autoware/param.yaml
fi