# 메인 PC 워크스페이스(main_ws) 구성 가이드

이 문서는 `uav_ws`(라즈베리파이 UAV 측)가 발행하는 토픽을 메인 PC에서 수신하기 위해
**메인 PC 쪽 워크스페이스(`main_ws`)를 어떻게 구성해야 하는지** 정리한 가이드다.
이 저장소(`uav_ws`)와는 별개의 워크스페이스이며, 이 문서는 참고용으로 `uav_ws`
저장소에 같이 보관한다.

받아야 하는 토픽 목록과 메시지 형식 자체는 [README.md](README.md)의
[메인 PC에서 각 토픽 수신하는 법](README.md#메인-pc에서-각-토픽-수신하는-법) 절을 따른다.
이 문서는 그걸 실제로 동작시키기 위한 **환경 설정 + 워크스페이스/노드 구조**를 다룬다.

> **참고**: 이 문서 작성 이후, 실제로는 멀티캐스트 discovery가 막힌 네트워크(핫스팟)
> 환경이라 아래 2번 항목과 달리 **Fast DDS Discovery Server + Super Client 구성을
> 사용 중**이다. 새 머신에 지금 실제로 쓰는 설정 그대로 적용하려면 이 문서 대신
> [README.md](README.md)를 따라갈 것. 이 문서는 초기 설계 배경 참고용으로 남겨둔다.

## 1. 사전 조건

메인 PC와 라즈베리파이(UAV)가 아래 조건을 동일하게 맞춰야 discovery가 된다.

```text
ROS 2 Humble (동일 배포판)
동일 ROS_DOMAIN_ID
동일 RMW 구현체 (기본 rmw_fastrtps_cpp)
같은 L2 네트워크 (같은 AP) — UDP 멀티캐스트가 라우터를 못 넘는 경우가 많음
```

메인 PC 패키지 설치 (Ubuntu 22.04 + Humble 기준):

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-compressed-image-transport \
  python3-opencv \
  python3-colcon-common-extensions
```

## 2. 환경변수 설정 (오늘 겪었던 문제들 기준)

`~/.bashrc`에 두 머신 모두 동일하게:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=23   # UAV와 반드시 동일한 값
```

Discovery Server를 쓰지 않는다면 `ROS_DISCOVERY_SERVER`는 아예 설정하지 말 것 (설정해두고
서버가 안 떠 있으면 discovery 자체가 실패한다 — 오늘 겪은 문제).

**주의할 점 (오늘 트러블슈팅에서 확인된 것들):**

- `.bashrc`를 고쳐도 **이미 열려 있던 터미널에는 반영되지 않는다.** 새 터미널을 열거나
  `source ~/.bashrc`를 다시 실행해야 한다.
- `ros2 daemon`은 **최초 실행 시점의 환경(도메인/디스커버리 서버)을 캐시한 채로 계속
  떠 있는다.** 환경변수를 바꾼 뒤에 `ros2 topic list`가 비어 보이면 대부분 이 문제다:
  ```bash
  ros2 daemon stop
  ros2 daemon start
  ```
- 그래도 안 보이면 UAV 쪽에서 실제로 노드가 떠 있는지, 같은 AP/서브넷에 있는지부터
  확인할 것 (`ros2 node list`, `ping <라즈베리파이 IP>`).

## 3. `main_ws` 워크스페이스 구조

프로젝트 설계상 메인 PC는 UAV의 압축 영상을 압축 해제해서 내부적으로
`/main/uav/...` 네임스페이스로 다시 발행하고, 그 뒤에 Visual SLAM / ArUco 노드가
그 raw 이미지를 구독하는 구조를 권장한다. 패키지 하나로 시작하면 충분하다.

```text
main_ws/
└── src/
    └── uav_camera_receiver/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── uav_camera_receiver
        ├── launch/
        │   └── uav_camera_receiver.launch.py
        └── uav_camera_receiver/
            ├── __init__.py
            └── image_decompressor_node.py
```

```bash
mkdir -p ~/main_ws/src
cd ~/main_ws/src
ros2 pkg create uav_camera_receiver --build-type ament_python \
  --dependencies rclpy sensor_msgs cv_bridge
```

## 4. 압축 해제 노드

컬러/뎁스 둘 다 같은 노드 클래스를 파라미터만 바꿔서 두 번 띄운다
(`uav_ws`의 `camera_health_node`와 동일한 패턴).

`uav_camera_receiver/uav_camera_receiver/image_decompressor_node.py`:

```python
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class ImageDecompressorNode(Node):
    def __init__(self):
        super().__init__("image_decompressor_node")

        self.declare_parameter("input_topic", "/uav/camera/color/image_raw/compressed")
        self.declare_parameter("output_topic", "/main/uav/camera/color/image_raw")
        self.declare_parameter("encoding", "bgr8")  # depth는 "16UC1"

        self.encoding = self.get_parameter("encoding").value
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            CompressedImage, self.get_parameter("input_topic").value, self._on_compressed, 10
        )

    def _on_compressed(self, msg):
        flag = cv2.IMREAD_UNCHANGED if self.encoding == "16UC1" else cv2.IMREAD_COLOR
        decoded = cv2.imdecode(np.frombuffer(msg.data, np.uint8), flag)
        if decoded is None:
            self.get_logger().warning("Failed to decode compressed frame", throttle_duration_sec=2.0)
            return

        image = self.bridge.cv2_to_imgmsg(decoded, encoding=self.encoding)
        image.header = msg.header
        self.publisher.publish(image)


def main(args=None):
    rclpy.init(args=args)
    node = ImageDecompressorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

`setup.py`의 `entry_points`에 등록:

```python
entry_points={
    "console_scripts": [
        "image_decompressor_node = uav_camera_receiver.image_decompressor_node:main",
    ],
},
```

`package.xml`에 실행 의존성 추가 (`ros2 pkg create --dependencies`로 만들었으면 이미 있음):

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>cv_bridge</exec_depend>
<exec_depend>python3-opencv</exec_depend>
```

## 5. Launch 파일

`launch/uav_camera_receiver.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="uav_camera_receiver",
                executable="image_decompressor_node",
                name="color_image_decompressor",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/uav/camera/color/image_raw/compressed",
                        "output_topic": "/main/uav/camera/color/image_raw",
                        "encoding": "bgr8",
                    }
                ],
            ),
            Node(
                package="uav_camera_receiver",
                executable="image_decompressor_node",
                name="depth_image_decompressor",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/uav/camera/depth/image_rect_raw/compressed",
                        "output_topic": "/main/uav/camera/depth/image_rect_raw",
                        "encoding": "16UC1",
                    }
                ],
            ),
        ]
    )
```

`CameraInfo`는 압축되지 않은 그대로 오므로 굳이 다시 발행할 필요 없이,
Visual SLAM/ArUco 노드가 `/uav/camera/color/camera_info` /
`/uav/camera/depth/camera_info`를 직접 구독하면 된다.

## 6. 빌드 및 실행

```bash
cd ~/main_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch uav_camera_receiver uav_camera_receiver.launch.py
```

## 7. 상태/heartbeat 토픽 구독 (필요할 때)

`std_msgs/String` JSON이라 별도 압축 해제가 필요 없다. 임무 관리 노드에서
그냥 파싱해서 쓰면 된다:

```python
import json
from std_msgs.msg import String

def on_status(msg: String):
    payload = json.loads(msg.data)
    ok = payload["ok"]
```

대상 토픽: `/uav/heartbeat`, `/uav/camera/color/status`, `/uav/camera/depth/status`,
`/uav/network/status`, `/uav/system/status` — 필드 목록은 README 참고.

## 8. 수신 확인 체크리스트

```bash
ros2 topic list                                    # /uav/... 와 /main/uav/... 둘 다 보여야 함
ros2 topic hz /main/uav/camera/color/image_raw      # 압축 해제된 raw 컬러
ros2 topic hz /main/uav/camera/depth/image_rect_raw # 압축 해제된 raw 뎁스
rqt_image_view /main/uav/camera/color/image_raw
```

## 9. 다음 단계 (참고)

전체 시스템 설계 문서(`/home/ubuntu/UAV_UGV_프로젝트_현황_상세정리.md`) 기준으로,
`/main/uav/camera/color/image_raw`는 이후 `/main/vslam_node`(Visual SLAM)와
`/main/aruco_detector`가 구독하는 입력이 된다. 이 가이드의 범위는 UAV 토픽을
받아서 raw 이미지로 만들어주는 부분까지이고, SLAM/ArUco 노드 자체는 별도 패키지로
구성하면 된다.
