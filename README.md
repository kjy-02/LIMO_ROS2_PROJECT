# LIMO · 객체 위치를 지도에 표시하기

SLAM과 3차원 좌표 변환을 공부한 뒤 LIMO 로봇에 적용한 프로젝트입니다. 기존 SLAM·Nav2에 YOLOv3 객체 검출을 연결하고, **쿼터니언을 회전행렬로 변환하는 계산을 직접 구현해 객체의 위치를 지도 좌표로 표현**했습니다.

| 항목 | 내용 |
| :--- | :--- |
| 목표 | SLAM 적용, YOLO 통합, 3차원 좌표 변환 구현 |
| 환경 | LIMO / Ubuntu 20.04 / ROS 2 Foxy / RealSense D435i / 2D LiDAR |
| 팀 구성 | 2인 |
| 본인 담당 | 객체 검출·좌표 변환·마커 표시 코드 구현 및 통합 주도 |
| 활용한 구성 | LIMO 드라이버, Cartographer, Nav2·AMCL, RealSense 드라이버, YOLOv3 사전학습 모델 |

![실험 결과 자료](https://github.com/user-attachments/assets/edeef7d7-1086-413c-8314-af90c7ee190b)

[구현 코드](code.py) · [설치 및 실행](#설치-및-실행) · [좌표 변환 상세](#좌표-변환-상세)

## 처리 흐름

```text
RGB 영상 → YOLOv3 → 객체 박스 중심 픽셀
                         + 깊이 영상 / CameraInfo
                         ↓
                  카메라 기준 3차원 위치
                         ↓ 고정 회전행렬 + 카메라 장착 오프셋
                  로봇 기준 위치 (base_link)
                         ↓ AMCL 위치 + 쿼터니언 → 회전행렬
                  지도 기준 위치 (map)
                         ↓
                  RViz 구형 마커 표시
```

SLAM·Nav2는 기존 패키지를 사용했습니다. 이 저장소의 [code.py](code.py)는 객체 검출, 깊이 기반 위치 계산, 좌표 변환, 마커 발행을 담당합니다. 객체 좌표 계산에는 TF 조회 대신 행렬 연산을 직접 사용했습니다.

## 설치 및 실행

아래는 **Ubuntu 20.04 / ROS 2 Foxy** 기준 안내입니다. 로봇에 이미 설치된 드라이버와 워크스페이스가 있다면 해당 환경을 우선 사용합니다. 각 실행 명령은 새 터미널에서 ROS 및 로봇 워크스페이스를 source한 뒤 실행합니다.

### 1. 필요한 패키지 준비

패키지는 아래 공식 저장소·설치 안내에서 받습니다.

| 필요한 구성 | 공식 다운로드·문서 | 용도 |
| :--- | :--- | :--- |
| ROS 2 Foxy | [Ubuntu 설치 안내](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) | ROS 실행 환경 |
| LIMO ROS 2 | [AgileX 공식 저장소 · foxy](https://github.com/agilexrobotics/limo_ros2/tree/foxy) | `limo_bringup`, 로봇 구동·SLAM 실행 파일 |
| Nav2 | [공식 저장소](https://github.com/ros-navigation/navigation2) · [설치 안내](https://docs.nav2.org/lyrical/getting_started/quickstart/quickstart/#installation) | AMCL, 지도 저장·불러오기 |
| RealSense | [공식 Foxy 지원 안내 · ros2-legacy](https://github.com/realsenseai/realsense-ros/tree/ros2-legacy) | RGB·깊이·카메라 내부 파라미터 |
| YOLOv3 | [Darknet 공식 모델 안내](https://pjreddie.com/darknet/yolo/) | 사전학습 가중치 및 모델 구성 |

```bash
sudo apt update
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup \
  ros-foxy-nav2-map-server ros-foxy-cartographer-ros \
  ros-foxy-cv-bridge ros-foxy-realsense2-camera \
  python3-opencv python3-numpy python3-colcon-common-extensions
```

`nav2_map_server`는 Nav2에 포함된 ROS 패키지입니다. 별도 프로젝트인 `nav2_map`를 내려받을 필요는 없습니다. 위 Nav2 문서는 최신 배포판도 다루므로 설치 패키지명은 이 프로젝트 환경인 `foxy`에 맞춥니다.

LIMO 패키지를 새로 구성하는 경우:

```bash
source /opt/ros/foxy/setup.bash
mkdir -p ~/limo_ws/src
cd ~/limo_ws/src
git clone --branch foxy https://github.com/agilexrobotics/limo_ros2.git
cd ~/limo_ws
# LiDAR 등 하드웨어별 드라이버는 제조사 안내에 따라 먼저 준비
colcon build --symlink-install
source ~/limo_ws/install/setup.bash
```

LIMO에 배포된 패키지 버전에 따라 launch 파일 이름과 의존성이 다를 수 있습니다. 사용 중인 로봇의 설치 환경은 다음 명령으로 확인합니다.

```bash
ros2 pkg prefix limo_bringup
ls "$(ros2 pkg prefix limo_bringup)/share/limo_bringup/launch"
```

### 2. 프로젝트와 YOLO 모델 준비

```bash
git clone https://github.com/kjy-02/LIMO_ROS2_PROJECT.git
cd LIMO_ROS2_PROJECT
```

다음 파일을 내려받아 준비합니다. 모델 파일은 이 저장소에 포함되어 있지 않습니다.

- [yolov3.weights](https://data.pjreddie.com/files/yolov3.weights)
- [yolov3.cfg](https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg)
- [coco.names](https://github.com/pjreddie/darknet/blob/master/data/coco.names)

`code.py`의 `weights`, `cfg`, `names`를 실제 파일 경로로 수정합니다. 현재 코드는 개발 당시의 `/home/agilex/limo21/src/` 경로를 사용합니다.

### 3. SLAM으로 지도 만들기

로봇과 SLAM을 각각 실행하고, 로봇을 이동시켜 지도를 작성합니다.

```bash
# 터미널 1: 로봇 구동
ros2 launch limo_bringup limo_start.launch.py

# 터미널 2: SLAM
ros2 launch limo_bringup cartographer.launch.py

# 터미널 3: 지도 저장
ros2 run nav2_map_server map_saver_cli -f ~/map
```

저장한 `map.yaml`과 지도 이미지 파일은 함께 보관합니다. 이후 위치 추정 단계에서는 지도 작성용 SLAM을 종료합니다.

### 4. 저장한 지도에서 위치 추정

로봇 구동을 유지한 상태에서 지도와 AMCL을 실행합니다. 공식 `foxy` 브랜치의 [limo_localization.launch.py](https://github.com/agilexrobotics/limo_ros2/blob/foxy/limo_bringup/launch/limo_localization.launch.py)는 map server와 AMCL을 함께 실행합니다.

```bash
ros2 launch limo_bringup limo_localization.launch.py \
  map:="$HOME/map.yaml" use_sim_time:=false
```

RViz의 **2D Pose Estimate**로 초기 위치를 지정하고 `/amcl_pose`가 들어오는지 확인합니다. 실제 로봇 기준으로 `use_sim_time:=false`를 사용합니다. 로봇에 설치된 Nav2 launch가 이미 map server·AMCL을 실행한다면 중복 실행하지 않습니다.

### 5. 카메라와 객체 좌표 계산 실행

```bash
# 터미널 1: 카메라
ros2 launch realsense2_camera rs_launch.py

# 터미널 2: 프로젝트 폴더에서 실행
python3 code.py
```

현재 저장소는 단일 Python 파일 구성이므로 `ros2 run` 대신 Python으로 실행합니다. 코드가 사용하는 토픽은 다음과 같습니다.

| 방향 | 토픽 | 데이터 |
| :--- | :--- | :--- |
| 입력 | `/camera/color/image_raw` | RGB 영상 |
| 입력 | `/camera/depth/image_rect_raw` | 깊이 영상 |
| 입력 | `/camera/color/camera_info` | 카메라 내부 파라미터 |
| 입력 | `/amcl_pose` | 지도 기준 로봇 위치·자세 |
| 출력 | `/detected_objects` | `visualization_msgs/msg/Marker` |

카메라 드라이버 버전에 따라 토픽 이름이 달라질 수 있습니다. `ros2 topic list`로 확인하고 구독 토픽 또는 remap을 맞춥니다. RGB와 깊이 영상의 정렬 조건은 아래 ‘현재 구현의 범위’도 확인합니다.

RViz에서 **Fixed Frame = map**, **Map**, **TF**, **Marker**를 추가하고 Marker 토픽을 `/detected_objects`로 선택합니다.

## 좌표 변환 상세

<details>
<summary><strong>1. 픽셀과 깊이 → 카메라 좌표</strong></summary>

검출 박스 중심 픽셀 $(u,v)$ 주변의 최대 $5 \times 5$ 깊이 영역에서 중앙값 $Z$를 구합니다. 카메라 내부 파라미터 $f_x,f_y,c_x,c_y$는 CameraInfo에서 읽습니다.

$$
X = \frac{(u-c_x)Z}{f_x}, \qquad
Y = \frac{(v-c_y)Z}{f_y}, \qquad
P_{\mathrm{cam}} = \begin{bmatrix}X\\Y\\Z\end{bmatrix}
$$

코드: `compute_depth_point()`

</details>

<details>
<summary><strong>2. 카메라 좌표 → 로봇 좌표</strong></summary>

카메라의 오른쪽·아래·전방 축을 로봇의 전방·왼쪽·위쪽 축으로 바꾸고 장착 오프셋을 더합니다.

$$
P_{\mathrm{base}} = R_{\mathrm{cam\to base}} P_{\mathrm{cam}} + t_{\mathrm{cam}}
$$

$$
R_{\mathrm{cam\to base}} =
\begin{bmatrix}
0&0&1\\
-1&0&0\\
0&-1&0
\end{bmatrix},
\qquad
t_{\mathrm{cam}} =
\begin{bmatrix}0.1\\0\\0.1\end{bmatrix}\ \mathrm{m}
$$

위 값은 코드에 고정된 가정입니다. 실제 장착 위치·자세가 달라지면 수정해야 합니다.

코드: `R_cam_to_base`, `camera_offset`, `transform_to_map()`

</details>

<details>
<summary><strong>3. 쿼터니언 → 회전행렬 → 지도 좌표</strong></summary>

AMCL에서 받은 로봇 자세 $q=(x,y,z,w)$를 정규화한 뒤 회전행렬로 바꿉니다. 이 변환을 라이브러리 호출 대신 직접 구현했습니다.

$$
R(q)=
\begin{bmatrix}
1-2(y^2+z^2)&2(xy-wz)&2(xz+wy)\\
2(xy+wz)&1-2(x^2+z^2)&2(yz-wx)\\
2(xz-wy)&2(yz+wx)&1-2(x^2+y^2)
\end{bmatrix}
$$

AMCL의 지도 기준 위치 $t_{\mathrm{robot}}$를 더해 객체의 지도 좌표를 구합니다.

$$
P_{\mathrm{map}}=R(q)P_{\mathrm{base}}+t_{\mathrm{robot}}
$$

코드: `quat_to_rot_matrix()`, `transform_to_map()`

쿼터니언의 크기가 0이면 코드에서는 경고를 출력하고 단위행렬을 사용합니다.

</details>

## 실험 자료

기존 실험의 지도 작성 자료와 결과 화면입니다.

<details>
<summary><strong>지도 작성 자료 보기</strong></summary>

<img src="https://github.com/user-attachments/assets/0d149542-7da0-4602-ac31-a485a3dda75b" alt="지도 작성 실험 자료 1" width="420">

<img src="https://github.com/user-attachments/assets/e0e25ca5-a246-4563-90c3-fabff38e21cc" alt="지도 작성 실험 자료 2" width="420">

![생성한 지도](https://github.com/user-attachments/assets/b6880b29-fdf0-42c7-bd79-04c3387e89f7)

</details>

<details>
<summary><strong>추가 결과 화면과 출력 로그 보기</strong></summary>

<img src="https://github.com/user-attachments/assets/55dd5e25-dcf3-42b3-99ab-5d48c2759246" alt="추가 실험 결과 화면" width="600">

```text
YOLO detected 1 objects
Map coords: [0.29899437 0.67378386 0.30389404]
Marker published at map=[0.29899437 0.67378386 0.30389404]
```

기존 실행 로그 예시이며 위치 정확도의 측정 결과는 아닙니다.

</details>

## 현재 구현의 범위

- 깊이와 AMCL 자세는 각각 가장 최근에 받은 값을 사용하며, RGB와의 시각 동기화는 구현하지 않았습니다.
- RGB 픽셀을 깊이 영상에 그대로 적용합니다. 정확한 투영을 위해 RGB·깊이 정렬 및 내부 파라미터의 일치 여부를 확인해야 합니다.
- 깊이 단위는 값의 크기로 추정하고, 카메라 장착 변환은 고정값을 사용합니다. 깊이 인코딩·단위와 실제 장착값 확인이 필요합니다.
- 마커는 검출 결과를 표시하는 용도입니다. 객체별 지속 추적·중복 통합을 수행하는 지도는 구현하지 않았습니다.

이 프로젝트에서는 학습한 좌표 변환을 실제 ROS 데이터에 연결하는 데 초점을 두었습니다. 위치 오차와 반복 측정 정확도는 별도로 평가하지 않았습니다.
