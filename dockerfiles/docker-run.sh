#!/bin/bash

set -o pipefail

mkdir -p /run/user/$UID/bus || true

DIR=$(cd -- "$(dirname -- "$0" )" &> /dev/null && pwd)
source $DIR/id.sh 2> /dev/null

if [[ -z "$CONTAINER_ID" || $(podman image exists $CONTAINER_ID) ]]; then
  echo "image $CONTAINER_ID does not exist, building first"
  sleep 3s
  exec $DIR/docker-build.sh
  source $DIR/id.sh
fi

exec podman run                         \
  -it --rm                              \
  -v $HOME/.ssh:/root/.ssh              \
  -v $(pwd):/root/my-whisper:Z          \
  -w /root/my-whisper                   \
  $CONTAINER_ID "$@"
