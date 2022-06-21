#!/usr/bin/bash

set -e
set -o pipefail

DIR=$( cd "$( dirname "$0" )" &> /dev/null && pwd )
## registry
# source $DIR/docker-common-tt.sh

TARGET=base
podman build --target $TARGET -t $TARGET -f $DIR/Dockerfile

ID=$(podman image inspect --format="{{.Id}}" $TARGET)

cat > $DIR/id.sh << EOF
CONTAINER_ID=$ID
EOF

# podman push $ID $REGISTRY:$ID --creds=
