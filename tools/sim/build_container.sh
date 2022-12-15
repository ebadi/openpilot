#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR/../../

docker build \
  -t master-webcam-openpilot-sim:latest \
  -f tools/sim/Dockerfile.sim .
