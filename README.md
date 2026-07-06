# main_ws

메인 PC에서 UAV(라즈베리파이)가 발행하는 카메라/상태 토픽을 수신하기 위한
ROS2 워크스페이스. Fast DDS Discovery Server + Super Client 구성을 사용한다
(핫스팟 등 멀티캐스트가 막힌 네트워크에서도 discovery가 되도록).

이 문서는 **다른 컴퓨터에 이 워크스페이스를 처음부터 그대로 재현**하기 위한
설치 가이드다. 배경 설계는 [MAIN_PC_SETUP.md](MAIN_PC_SETUP.md), 겪었던
문제와 원인 분석은 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)에 따로 정리돼 있다.

## 0. 전제 조건

- Ubuntu 22.04 + ROS 2 Humble
- UAV와 **동일한 `ROS_DOMAIN_ID`**, 같은 L2 네트워크(같은 AP/핫스팟)

## 1. 패키지 설치

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-compressed-image-transport \
  ros-humble-rqt-image-view \
  python3-opencv \
  python3-colcon-common-extensions
```

## 2. 저장소 클론 & 빌드

```bash
git clone https://github.com/CO-UV/main_ws.git ~/main_ws
cd ~/main_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

`scripts/start_discovery_server.sh`는 실행 권한이 있는 채로 커밋돼 있어서
클론하면 바로 실행 가능한 상태다 (혹시 안 되면 `chmod +x scripts/*.sh`).

## 3. `~/.bashrc` 설정 (ROS_DOMAIN_ID / Discovery Server / Super Client / SHM)

`config/bashrc.ros2.snippet`을 `~/.bashrc` 끝에 추가한다:

```bash
cat ~/main_ws/config/bashrc.ros2.snippet >> ~/.bashrc
source ~/.bashrc
```

> **주의**: `~/.bashrc`에 이미 `source /opt/ros/humble/setup.bash`가 있다면
> snippet에도 같은 줄이 있어서 중복 실행된다 (동작에는 문제없지만 신경 쓰이면
> snippet에서 그 한 줄만 지우고 병합할 것).

이 snippet이 하는 일 (자세한 배경은 TROUBLESHOOTING.md 참고):

| 설정 | 이유 |
|---|---|
| `ROS_DOMAIN_ID=23` | UAV와 반드시 동일해야 discovery 됨 |
| `ROS_DISCOVERY_SERVER` 자동 감지 (`ip route get`) | 핫스팟이라 IP가 고정이 아니라서, 터미널 열 때마다 현재 LAN IP로 자동 설정 |
| `ROS_SUPER_CLIENT=TRUE` | 이 PC의 `ros2 topic list`/`rqt` 등이 그래프 전체를 보게 함 |
| `FASTRTPS_DEFAULT_PROFILES_FILE` (`scripts/fastdds_shm_profile.xml`) | Discovery Server 모드에서도 로컬 토픽이 SHM을 쓰도록 강제 + SHM 세그먼트를 8MB로 확장 (기본 512KB는 raw 카메라 이미지보다 작아서 SHM이 조용히 UDP로 fallback됨 — 실측으로 확인된 문제, 상세는 TROUBLESHOOTING.md 3번) |

새 터미널을 열면 아래처럼 현재 설정이 바로 보인다:
```
ROS_DISCOVERY_SERVER: <이 PC의 IP>:11811  (ROS_DOMAIN_ID=23, SUPER_CLIENT=TRUE, SHM=on)
```

## 4. Fast DDS Discovery Server 상시 구동 (systemd --user)

```bash
mkdir -p ~/.config/systemd/user
cp ~/main_ws/config/fastdds-discovery-server.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now fastdds-discovery-server.service
systemctl --user status fastdds-discovery-server.service   # active (running) 인지 확인
```

로그인 세션이 없어도(재부팅 직후 등) 서비스가 자동 기동되게 하려면 linger를
켠다 (sudo 필요):
```bash
sudo loginctl enable-linger "$USER"
```

서버가 제대로 뜨면 `ss -ulnp | grep 11811`로 UDP 11811 포트가 리스닝 중인
것을 확인할 수 있다.

## 5. 동작 확인

```bash
ros2 daemon stop
ros2 daemon start
ros2 topic list        # /uav/... 와 /main/uav/... 가 보이면 정상
```

`ros2 daemon`은 최초 실행 시점의 환경을 캐싱하므로, `.bashrc`를 새로 적용한
직후나 IP가 바뀐 직후에는 위 두 줄로 재시작해줘야 한다.

## 6. UAV(라즈베리파이) 쪽 맞추기

이 메인 PC의 IP는 핫스팟 환경이라 유동적이다. UAV의 `ROS_DISCOVERY_SERVER`도
**이 PC가 현재 감지한 IP:11811**과 동일해야 한다. 아무 터미널에서나:
```bash
echo $ROS_DISCOVERY_SERVER
```
로 확인한 값을 UAV 쪽 환경변수에 반영한다. (UAV는 SUPER_CLIENT일 필요는
없고 일반 CLIENT로 충분.)

## 7. 카메라 수신 노드 실행

```bash
cd ~/main_ws
source install/setup.bash
ros2 launch uav_camera_receiver uav_camera_receiver.launch.py
```

압축된 컬러/뎁스 영상을 각각 `/main/uav/camera/color/image_raw`,
`/main/uav/camera/depth/image_rect_raw`로 압축 해제해서 재발행한다.

## 8. 수신 확인 체크리스트

```bash
ros2 topic hz /main/uav/camera/color/image_raw       # ~15Hz 나와야 정상
ros2 topic hz /main/uav/camera/depth/image_rect_raw
rqt_image_view /main/uav/camera/color/image_raw       # 색상 정상인지 육안 확인
```

`ros2 topic hz`가 몇 초씩 끊기거나 색상이 초록/자홍으로 깨져 보이면
[TROUBLESHOOTING.md](TROUBLESHOOTING.md)에 동일 증상과 원인 분석이 정리돼
있으니 먼저 확인할 것.

## 문서 구성

- **README.md** (이 문서) — 새 머신에 처음부터 재현하는 설치 가이드
- **[MAIN_PC_SETUP.md](MAIN_PC_SETUP.md)** — 워크스페이스/노드 구조 설계 배경
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** — 색상 깨짐, 프레임 드롭 등
  실제로 겪은 문제의 원인 분석과 해결 기록 (같은 증상 재발 시 먼저 확인)

## 저장소 구조

```
main_ws/
├── config/
│   ├── bashrc.ros2.snippet              # ~/.bashrc에 추가할 환경변수 블록
│   └── fastdds-discovery-server.service # systemd --user 유닛 템플릿
├── scripts/
│   ├── start_discovery_server.sh        # Discovery Server 실행 스크립트
│   └── fastdds_shm_profile.xml          # SHM 전송 강제 + 세그먼트 8MB 프로파일
└── src/
    └── uav_camera_receiver/             # 압축 해제 노드 (color/depth)
```
