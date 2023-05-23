#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR/../../


# docker pull ghcr.io/commaai/openpilot-base:latest
docker pull ghcr.io/commaai/openpilot-base-cl:latest
docker build \
  -t openpilot-sim-gokart:latest \
  -f tools/sim/Dockerfile.sim .
