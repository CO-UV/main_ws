#!/usr/bin/env bash
# Fast DDS Discovery Server (id 0) 실행 스크립트.
# 모든 인터페이스(0.0.0.0)의 UDP 11811 포트에서 클라이언트(메인 PC 노드 + UAV)를 받는다.
set -eo pipefail

source /opt/ros/humble/setup.bash

exec fastdds discovery -i 0 -l 0.0.0.0 -p 11811
