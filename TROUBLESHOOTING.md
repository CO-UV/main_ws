# main_ws 트러블슈팅 기록

이 문서는 `main_ws` 구성 중 겪은 문제들과 해결 과정을 정리한 기록이다.
설정 자체의 절차는 [MAIN_PC_SETUP.md](MAIN_PC_SETUP.md)를 참고하고,
이 문서는 "왜 그렇게 했는지" / "무엇이 문제였는지"에 집중한다.

## 1. Fast DDS Discovery Server + Super Client 구성

### 왜 필요했나
기본(멀티캐스트) discovery는 이 VM 환경에서 동작하지 않았다
(같은 도메인이어도 서로를 못 찾음). 그래서 Discovery Server를 메인 PC에
직접 띄우고, 메인 PC의 도구들(`ros2 topic list`, `rqt` 등)이 그래프
전체를 볼 수 있도록 SUPER_CLIENT로 설정했다.

### 구성
- 서버 실행 스크립트: `scripts/start_discovery_server.sh`
  (`fastdds discovery -i 0 -l 0.0.0.0 -p 11811`)
- 상시 구동: `~/.config/systemd/user/fastdds-discovery-server.service`
  (`systemctl --user enable --now fastdds-discovery-server.service`)
- `~/.bashrc`에 환경변수:
  ```bash
  export ROS_DOMAIN_ID=23
  export ROS_DISCOVERY_SERVER=<메인 PC IP>:11811
  export ROS_SUPER_CLIENT=TRUE
  ```

### 핫스팟 유동 IP 문제
메인 PC가 핫스팟에 붙어서 IP가 고정이 아니었다. `ROS_DISCOVERY_SERVER`에
IP를 하드코딩하는 대신, 터미널을 열 때마다 `ip route get 1.1.1.1`로
현재 LAN IP를 자동 감지해서 채우도록 했다 (`~/.bashrc`).
서버 자체는 `0.0.0.0`에 바인딩돼 있어서 IP가 바뀌어도 그대로 잘 뜬다.
단, **이미 열려 있던 터미널**은 IP가 바뀌어도 갱신되지 않으므로
`source ~/.bashrc` + `ros2 daemon stop/start`가 필요하다.

새 터미널을 열면 아래처럼 현재 값을 바로 보여준다:
```
ROS_DISCOVERY_SERVER: 10.194.146.49:11811  (ROS_DOMAIN_ID=23, SUPER_CLIENT=TRUE, SHM=on)
```

## 2. 컬러 이미지 색상이 초록/자홍색으로 깨지는 문제

### 증상
`/main/uav/camera/color/image_raw`를 `rqt_image_view`로 보면 전체적으로
초록/자홍색으로 색이 깨져 보임. 구조·윤곽선·밝기는 정상, 색상만 이상.

### 최초 가설 (틀림)
카메라가 보고하는 YUYV 포맷의 실제 바이트 순서가 YVYU(U/V 스왑)일
것으로 추정하고 `cv2.cvtColor` 플래그를 `COLOR_YUV2BGR_YUYV` →
`COLOR_YUV2BGR_YVYU`로 변경했으나 **증상 동일하게 재현됨** (가설 기각).

### 실제 원인
`v4l2_mjpeg_node.py`가 `v4l2-ctl`을 서브프로세스로 띄우고 stdout raw
파이프를 고정 크기(`width*height*2`바이트)로 직접 읽어서 프레임을
파싱하고 있었음. 이 방식은 프레임 경계 동기화를 보장하지 않아서,
스트림 시작 직후나 버퍼 드랍 시점에 한 번이라도 읽기 위치가 프레임
경계와 어긋나면 그 이후 모든 프레임이 서로 다른 두 프레임의 조각이
섞인 채로 영구적으로 밀려서 읽히게 됨 (초록/자홍 줄무늬로 나타남).

- raw 프레임을 파일로 통째로 캡처해서 오프라인으로 디코드하면 정상
  → 채널 순서 자체는 문제 아님
- 실제 ROS 노드가 라이브 파이프에서 읽을 때만 재현 → 파이프 동기화
  문제로 확인

### 조치
`v4l2_mjpeg_node.py`를 subprocess + 수동 파이프 파싱 방식에서
`cv2.VideoCapture`(OpenCV V4L2 백엔드) 사용으로 전면 재작성. OpenCV가
V4L2 프레임 동기화를 내부적으로 처리해주기 때문에 이 구조적 버그가
원천 제거됨. 부수적으로 `v4l2-ctl` 서브프로세스 관리, stderr 드레인
스레드 등 관련 코드도 단순화됨.

### 검증
- 라이브로 발행되는 프레임을 직접 캡처해서 확인 → 색상 정상 (초록/자홍
  없음, 줄무늬 없음)
- `ros2 topic hz`로 15Hz 안정적으로 발행되는 것 확인

### 교훈
- "색이 이상하다"고 다 YUV 성분 순서(YUYV/YVYU/UYVY) 문제는 아니다.
  구조·윤곽선은 멀쩡한데 색만 깨지는 증상은 **프레임 동기화가 어긋나서
  서로 다른 두 프레임 조각이 섞이는 경우**에도 똑같이 나타날 수 있다.
- 플래그를 바꿔서 증상이 그대로면, 원인을 채널 순서 쪽에서 계속 찾지
  말고 한 단계 아래(파이프/프레임 경계 동기화)를 의심할 것.
- 의심되는 스트림을 파일로 통째로 캡처해서 오프라인 디코드해보는 게
  "라이브 파싱 문제"와 "데이터 자체 문제"를 가르는 데 효과적이었다.

## 3. Raw 이미지 토픽이 15fps인데 실제로는 0.5~3Hz로 뚝뚝 끊기는 문제

### 증상
UAV → main_ws로 오는 압축(`/uav/.../compressed`) 토픽은 안정적으로
15Hz인데, main_ws가 압축 해제해서 재발행하는 raw 토픽
(`/main/uav/camera/color/image_raw`)은 평균 0.5~1Hz, 심하면 프레임 간격이
몇 초씩 벌어짐.

### 잘못 짚었던 가설들 (기록으로 남김 — 나중에 비슷한 증상 만나면 순서대로 배제)

1. **UDP 커널 버퍼 부족설**
   `netstat -su`에서 `receive buffer errors`가 실제로 있었고
   (`net.core.rmem_max` 기본값 212992바이트로 너무 작음), 이건 진짜 문제라
   `sysctl`로 버퍼를 키우긴 했음. 하지만 이것만으로는 해결 안 됨
   (드롭 카운터는 안 늘어나는데도 여전히 느림) → 부분적 원인이었을 뿐
   **진짜 원인은 아니었음**.
2. **RELIABLE QoS heartbeat 지연설**
   대용량 메시지 + RELIABLE이 fragment 유실 시 heartbeat 주기까지
   기다린다는 가설로 `qos_profile_sensor_data`(BEST_EFFORT)로 바꿔봤지만
   **변화 없음** → 기각.
3. **CPU 부족설 (2 vCPU VM이라 감당 못 함)**
   `top`으로 직접 확인해보니 관련 프로세스 CPU 사용률이 10~30% 수준으로
   전혀 높지 않았음 → **명백히 틀린 가설**. (사용자가 htop으로 직접
   확인하고 지적해서 잡아낸 오진단)

### 진짜 원인
`free -h`에서 `shared` 메모리 사용량이 거의 0에 가까웠던 것에 착안해
Fast DDS의 SHM(공유메모리) 전송이 로컬 토픽에 안 쓰이고 있는지 확인:

- Discovery Server(Client/SUPER_CLIENT) 모드로 참가자를 구성하면
  기본적으로 로컬(같은 머신) 토픽도 SHM이 아니라 UDP로만 오간다.
- SHM 전송을 명시적으로 켜는 XML 프로파일(`scripts/fastdds_shm_profile.xml`)을
  적용했더니 0.3~0.5Hz → 2.8~5Hz로 크게 개선됐지만, 여전히 15Hz는
  안 나왔음.
- `/dev/shm`을 직접 열어보니 Fast DDS의 **SHM 세그먼트 기본 크기가
  512KB(정확히는 549,408바이트)**였는데, raw 컬러 이미지 한 장은
  640×480×3 = **921,600바이트**로 이 세그먼트보다 컸다.
  → **이미지가 세그먼트에 안 들어가서 SHM을 아예 못 타고 UDP로
  fallback되고 있었던 것**이 진짜 원인.

### 해결
`fastdds_shm_profile.xml`의 SHM transport에 `segment_size`를 8MB로
키움:
```xml
<transport_descriptor>
  <transport_id>shm_transport</transport_id>
  <type>SHM</type>
  <segment_size>8388608</segment_size>
</transport_descriptor>
```
→ `ros2 topic hz /main/uav/camera/color/image_raw`가 **정확히 15Hz**로
안정화됨 (std dev 0.01s 수준).

### 교훈
- "CPU가 낮은데도 느림"은 CPU 병목이 아니라는 확실한 신호였다 —
  htop/top으로 실측 없이 "VM이라 느릴 것"이라고 짐작한 게 오진단의 원인.
- Fast DDS SHM 전송은 메시지 크기가 세그먼트 크기보다 크면 조용히
  다른 transport로 넘어간다 (에러를 던지지 않음) — 그래서 겉보기엔
  "SHM을 켰는데 왜 안 되지"처럼 보인다. 큰 센서 메시지(카메라 이미지 등)를
  로컬에서 주고받을 땐 `segment_size`를 실제 메시지 크기보다 넉넉히
  크게 잡아야 한다.
- `/dev/shm`을 직접 들여다보면 (`ls -la /dev/shm`) 세그먼트 크기를
  바로 확인할 수 있어서 진단에 유용했다.

## 4. Depth raw 토픽이 1.1Hz밖에 안 나오는 문제 (해결)

### 증상
컬러와 달리 뎁스는 SHM 세그먼트 문제를 고치고도 여전히 1.1Hz.
확인해보니 **UAV에서 오는 압축 뎁스 토픽 자체가 이미 1.1Hz**로 들어옴
(`/uav/camera/depth/image_rect_raw/compressed`). main_ws 쪽 문제가 아니라
UAV 쪽(`v4l2_depth_node`) 원인으로 확인됨.

### 잘못 짚었던 가설
라즈베리파이 CPU 자체가 부족해서 `depth_fps`를 낮게 잡을 수밖에 없다고
판단하고 있었음 → **틀림**. `htop`으로 보면 4코어 중 여유가 있었고, 근본
원인은 CPU 총량 부족이 아니라 코드 안의 낭비였음.

### 실제 원인
`v4l2_depth_node.py`가 `publish_raw=false`(기본 설정)임에도 **매 프레임
쓰지도 않는 raw `sensor_msgs/Image` 메시지를 만들고 있었음**. 그중
`image.data = frame`(614KB `bytes`를 메시지 필드에 대입) 한 줄이 이
Pi에서 **프레임당 ~555ms**나 걸림 — rclpy가 생성한 `uint8[]` 필드
setter가 이 정도 크기에서 비정상적으로 느린 것으로 보임. 실제 처리
비용(raw 프레임 읽기 ~6ms, PNG 인코딩 ~55ms, publish ~45ms)은 이 낭비에
비하면 미미했음.

단계별(`_read_exact_frame` → 메시지 구성 → encode → publish)로 타이밍
로그를 직접 찍어서 어디서 시간이 새는지 실측으로 확인 — 추측이 아니라
측정으로 찾아냄.

### 해결
`publish_raw`가 켜져 있을 때만 raw `Image` 메시지를 만들도록 수정.
header는 `std_msgs/Header`로 한 번만 만들어서 raw/compressed/camera_info가
공유하도록 정리 (`src/uav_camera_streamer/uav_camera_streamer/v4l2_depth_node.py`,
`uav_ws` 저장소).

### 검증
- `depth_fps:=5.0` 그대로: 1.1~1.4Hz → **4.97~5.13Hz**로 정상화
- `depth_fps:=15.0`으로 한계치 테스트: **~7.7~8Hz**까지 나옴 (당시 코드의
  자연스러운 상한선 — read+encode+publish 합쳐서 프레임당 ~125ms)
- 참고로 raw Z16 캡처 자체는 `v4l2-ctl`로 따로 재보면 640x480에서
  **60fps**까지도 나온다 (드랍 1개 수준). 즉 8Hz는 센서/USB 한계가 아니라
  순전히 소프트웨어 파이프라인(read→encode→publish를 한 스레드에서 순서대로
  처리) 한계였음.

### 추가 개선 시도: 파이프라인화 (부분 성공)
읽기를 별도 스레드로 분리해서 항상 최신 프레임을 준비해두고, publish를
`ThreadPoolExecutor`로 넘겨서 인코딩과 겹치게 만들면 이론상
`1/max(encode, publish)` ≈ 15~18Hz까지 오를 것으로 기대했음.

실제로는 **~9Hz**까지만 올랐다 (8Hz → 9Hz, read 대기시간 제거분 정도만
이득). 원인은 **Python GIL** — `cv2.imencode`는 GIL을 풀어주지만
`rclpy`의 `publish()`(직렬화 + rmw 전달)는 GIL을 오래 붙들고 있어서,
스레드를 나눠도 인코딩과 발행이 실제로 동시에 실행되지 않고 GIL을
번갈아 가지면서 순서대로 처리됨. 스레드로는 넘겼지만 진짜 병렬 실행이
아니었던 것.

기본값을 `depth_fps:=9.0`으로 설정하고 여기서 멈춤. 더 올리려면:
- 해상도를 낮춰서 인코딩 자체를 가볍게 만들거나 (예: 424x240)
- 멀티프로세싱(별도 OS 프로세스)으로 GIL을 아예 우회해야 함 — 다만
  프레임 데이터를 프로세스 간에 넘기는 IPC 비용이 이득을 상쇄할 수 있어서
  아직 시도 안 함.

### 교훈
- "CPU가 느려서 fps를 못 올린다"고 짐작하기 전에 `htop`/실측으로 확인할
  것. 이번에도 실제 병목은 CPU 총량이 아니라 안 쓰는 메시지를 만드는
  낭비 코드였음 (섹션 3의 "CPU 부족설" 오진단과 같은 패턴).
- **발행하지 않는 메시지를 만드는 데도 비용이 든다** — 특히 큰 배열
  필드(`uint8[]`, `Image.data` 등)를 rclpy 메시지에 대입하는 연산은
  생각보다 느릴 수 있다. `if publisher is not None:` 가드가 있어도,
  메시지 자체를 그 가드 **밖에서** 미리 만들어버리면 가드는 아무 의미가
  없다.
- 단계별 타이밍 로그를 직접 찍어보는 게 제일 빠른 진단 방법이었다
  (섹션 2의 "라이브 캡처해서 오프라인 디코드"만큼 효과적).
- **파이썬 스레드로 넘긴다고 다 병렬로 겹치는 건 아니다.** GIL을 실제로
  풀어주는 호출(`cv2.imencode` 같은 C 확장)인지 확인 없이 "무거운 작업을
  스레드로 빼면 겹치겠지"라고 가정하면 틀릴 수 있다 — `rclpy.publish()`가
  그 경우였다. 진짜 병렬이 필요하면 멀티프로세싱을 고려해야 한다.

## 5. 뎁스 이미지가 갑자기 등고선 무늬로 깨져 보이는 문제 (하드웨어 원인)

### 증상
코드를 전혀 안 건드렸는데도(섹션 4의 파이프라인 이전 검증된 버전 그대로)
뎁스 이미지(`/main/uav/camera/depth/image_rect_raw`)가 갑자기 등고선/
terracing 무늬로 나오기 시작함. 직전까지는 정상적인 이미지가 계속
나오고 있었음.

### 잘못 짚었던 가설
- 코드는 안 바뀌었는데 결과가 바뀌어서 처음엔 소프트웨어 쪽(파이프라인
  코드 잔재, 포트 꼬임)을 의심했음 — 파일 타임스탬프로 확인해보니 코드는
  정말 안 바뀌어 있었음.
- 등고선 무늬 자체가 RealSense 스테레오 뎁스의 정상적인 quantization/
  terracing 현상(매끈하고 무늬 없는 표면에서 흔히 보임)과 겉보기엔
  비슷해서, "그냥 정상적인 센서 특성"으로 결론지을 뻔했음.

### 실제 원인
`dmesg`를 확인해보니 **RealSense 카메라가 세션 도중 두 번 USB 연결이
완전히 끊겼다가 재연결됨**:
```
[13881s] usb 2-1: USB disconnect, device number 2
[13884s] usb 2-1: new SuperSpeed USB device ... RealSense Depth Camera 435if
[14635s] usb 2-1: USB disconnect, device number 3
[14642s] usb 2-1: new SuperSpeed USB device ... RealSense Depth Camera 435if
```
이 재연결 때문에 `/dev/video4`(컬러)가 `/dev/video5`로 번호가 바뀌기도
했음. USB가 완전히 끊겼다 붙으면 카메라 내부 펌웨어 상태(IR 프로젝터
on/off, 자동노출 보정 등)가 초기화되는데, V4L2/UVC 레벨에서만 접근하는
우리 코드는 이런 RealSense 고유 상태를 제어/복원하지 못한다. 재연결 후
IR 프로젝터가 꺼진 채로 초기화되면 무늬 없는 표면에서 스테레오 매칭이
실패해서 등고선 무늬가 나타난 것으로 추정됨.

재연결을 유발한 정확한 원인은 특정하지 못했지만, 디버깅 중 V4L2
디바이스를 매우 짧은 간격으로 반복 강제종료(`kill -9`)/재시작한 것이
유력한 원인으로 추정됨.

### 해결
카메라 USB 케이블을 물리적으로 뽑았다 다시 꽂아서 완전히 전원 재시작.
소프트웨어 레벨에서는 이 상태를 복구할 방법이 없다 (v4l2-ctl/UVC
인터페이스로는 RealSense 펌웨어 내부 상태에 접근 불가).

### 교훈
- 카메라 관련 이상 증상이 보이면 **코드 diff부터 보지 말고 `dmesg`로 USB
  재연결 여부부터 확인**할 것. "코드를 안 바꿨는데 결과가 바뀌었다"는
  소프트웨어 바깥(하드웨어/USB) 원인의 강한 신호다.
- V4L2 디바이스를 짧은 시간에 너무 여러 번 강제 종료/재시작하면 USB
  재연결을 유발할 수 있어 보인다 — 테스트할 때 장치를 완전히 정리하고
  최소 1초 이상 텀을 두는 습관이 필요.
- `/dev/videoN` 번호는 USB 재연결이 일어나면 바뀔 수 있다 — launch
  인자로 디바이스 경로를 하드코딩해두면 재연결 후 실행이 조용히
  실패하거나 엉뚱한 장치를 열 수 있으니, 실행 전 `v4l2-ctl --list-devices`로
  항상 확인할 것.
- RealSense를 순수 V4L2/UVC로 열면 IR 프로젝터/자동노출 등 고유 제어를
  건드릴 수 없다 — 재연결 후 화질이 이상하면 소프트웨어보다 먼저 물리적
  재연결(전원 재시작)을 시도해볼 것.

## 관련 파일

- `scripts/start_discovery_server.sh` — Discovery Server 실행
- `scripts/fastdds_shm_profile.xml` — SHM 전송 강제 활성화 (segment_size 8MB)
- `src/uav_camera_receiver/uav_camera_receiver/image_decompressor_node.py` —
  퍼블리셔 QoS를 `qos_profile_sensor_data`(BEST_EFFORT)로 설정
- `~/.bashrc` — `ROS_DOMAIN_ID`, `ROS_DISCOVERY_SERVER`(자동 IP 감지),
  `ROS_SUPER_CLIENT`, `FASTRTPS_DEFAULT_PROFILES_FILE` 설정
- `~/.config/systemd/user/fastdds-discovery-server.service` — Discovery
  Server 상시 구동
- (`uav_ws` 저장소) `src/uav_camera_streamer/uav_camera_streamer/v4l2_depth_node.py` —
  `publish_raw=false`일 때 안 쓰는 raw `Image` 메시지를 만들지 않도록 수정
  (섹션 4 참고)
