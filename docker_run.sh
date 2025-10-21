#!/bin/bash

target="${1}"
device="${2}"
case "${target}" in
"eval")
    volume="output:/output"
    ;;
"dev")
    volume="output:/output aichallenge:/aichallenge remote:/remote vehicle:/vehicle"
    ;;
"rm")
    # clean up old <none> images
    docker image prune -f
    exit 1
    ;;
*)
    echo "invalid argument (use 'dev' or 'eval')"
    exit 1
    ;;
esac

if [ "${device}" = "cpu" ]; then
    opts=""
    echo "[INFO] Running in CPU mode (forced by argument)"
elif [ "${device}" = "gpu" ]; then
    opts="--nvidia"
    echo "[INFO] Running in GPU mode (forced by argument)"
elif command -v nvidia-smi &>/dev/null && [[ -e /dev/nvidia0 ]]; then
    opts="--nvidia"
    echo "[INFO] NVIDIA GPU detected → enabling --nvidia"
else
    opts=""
    echo "[INFO] No NVIDIA GPU detected → running on CPU"
fi

mkdir -p output

LOG_DIR="output/latest"
mkdir -p $LOG_DIR
LOG_FILE="$LOG_DIR/docker_run.log"
echo "A rocker run log is stored at : file://$LOG_FILE"

# shellcheck disable=SC2086
rocker ${opts} --x11 --devices /dev/dri --user --net host --privileged --name "aichallenge-2025-$(date "+%Y-%m-%d-%H-%M-%S")" --volume ${volume} -- "aichallenge-2025-${target}-${USER}" 2>&1 | tee "$LOG_FILE"
