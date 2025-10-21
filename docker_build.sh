#!/bin/bash

target=${1}

case "${target}" in
"eval")
    opts="--no-cache"
    ;;
"dev")
    opts=""
    ;;
*)
    echo "invalid argument (use 'dev' or 'eval')"
    exit 1
    ;;
esac

LOG_DIR="output/latest"
mkdir -p $LOG_DIR
LOG_FILE="$LOG_DIR/docker_build.log"
echo "A build log is stored at : file://$LOG_FILE"

# shellcheck disable=SC2086
docker build ${opts} --progress=plain --target "${target}" -t "aichallenge-2025-${target}-${USER}" . 2>&1 | tee "$LOG_FILE"
echo "========================================================"
echo "This log is in : file://$LOG_FILE"
echo "========================================================"
