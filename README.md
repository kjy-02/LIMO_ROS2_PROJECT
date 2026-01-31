# LIMO ROS2 Object Mapping Project

이 프로젝트는 LIMO 로봇을 활용하여 실내 환경의 지도를 작성(SLAM)하고, YOLO v3 객체 인식을 통해 탐지된 사물의 위치를 좌표 변환하여 맵 상에 마커(Marker)로 시각화하는 자율주행/로봇 비전 프로젝트입니다.
https://github.com/realsenseai/realsense-ros/tree/ros2-development 카메라 패키지 다운 참고

## 프로젝트 개요 (Project Overview)
자율주행 로봇이 주행 중 마주치는 장애물이나 특정 객체를 인식하고, 그 객체가 지도상 어디에 위치하는지 파악하는 것은 매우 중요합니다. 이 프로젝트는 다음과 같은 주요 기능을 수행합니다.

1. SLAM (Simultaneous Localization and Mapping): Lidar 센서를 이용하여 주변 환경의 2D 지도를 작성합니다.
2. Object Detection (YOLO v3): 카메라를 통해 들어오는 영상에서 실시간으로 객체(사람, 사물 등)를 인식합니다.
3. Coordinate Transformation: 카메라 좌표계(2D 이미지)에서 인식된 객체의 위치를 로봇 좌표계를 거쳐 지도 좌표계(Map Frame)로 변환합니다.
4. Visualization: 변환된 좌표를 바탕으로 RViz 상의 지도 위에 해당 객체의 위치를 마커로 표시합니다.
## 주요 기능 및 기술
Platform: AgileX LIMO (NVIDIA Jetson Nano 기반)
OS / Middleware: Ubuntu 20.04 / ROS2 Foxy
SLAM: Cartographer / Gmapping
Object Detection: YOLO v3
Navigation: Nav2 (Navigation 2 Stack)
Sensors: 2D Lidar, RGB-D Camera (RealSense D435i)
## 맵 만들기 (SLAM Mapping)
1. **리모 구동 (Bringup)**

    ```bash
    ros2 launch limo_bringup limo_start.launch.py
    ```

2. **SLAM 실행 (Catographer)**

   ```bash
   ros2 launch limo_bringup cartographer.launch.py
   ```

3. **맵 저장 (Map Saver)**

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/map
   ```

<img width="400" height="400" alt="2" src="https://github.com/user-attachments/assets/0d149542-7da0-4602-ac31-a485a3dda75b" />
<img width="400" height="400" alt="1" src="https://github.com/user-attachments/assets/e0e25ca5-a246-4563-90c3-fabff38e21cc" />

![map (1)](https://github.com/user-attachments/assets/b6880b29-fdf0-42c7-bd79-04c3387e89f7)


## 객체 인신 및 마커 생성 (Object Detection & Marker)
1. **리모 구동**
    
    ```bash
    ros2 launch limo_bringup limo_start.launch.py
    ```
    
2. **지도 불러오기**
    
    ```bash
    ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/agilex/map.yaml
    ```
    
3. **Localization 실행 (AMCL).**
    
    ```bash
    ros2 launch limo_bringup limo_nav2_ackmann.launch.py map:=/home/agilex/map.yaml use_sim_time:=true
    ```
    
4. **Realsense 카메라 구동**
    
    ```bash
    ros2 launch realsense2_camera rs_launch.py
    ```
    
5. **YOLO+Depth→Map 마커 코드 실행**
    
    ```bash
    ros2 run limo_ros2_project code.py
    ```
    
6. **RViz 설정**
    - Fixed Frame: `map`
    - Display → `Map`, `TF`, `Marker`


- 좌표 변환 과정 (**픽셀 + depth → 카메라 → 로봇(base_link) → 맵(map))**
    
    ### 로그
    
    ```
    [CAM] X=0.122, Y=-0.214, Z=1.456
    [BASE] [ 1.556      -0.12196973  0.3135729 ]
    [MAP]  [0.03616281 0.76370386 0.3135729 ]
    ```
    
    ## **1. 픽셀 + 깊이 → 카메라 좌표계 (P_cam)**
    
    YOLO가 잡은 박스 중심 픽셀 좌표를 **`(u, v)`**라고 하고,
    
    해당 픽셀에서 뽑은 깊이(거리)를 **`Z`**라고 하면,
    
    - 카메라 내부 파라미터:
        - **`fx = msg.k[0]`**
        - **`fy = msg.k[4]`**
        - **`cx = msg.k[2]`**
        - **`cy = msg.k[5]`**
  <img width="367" height="255" alt="image" src="https://github.com/user-attachments/assets/0445d335-3242-48ed-bf25-a889e4a41f43" />

    
    ```cpp
    //코드 구현
    X = (u - self.cx) * Z / self.fx
    Y = (v - self.cy) * Z / self.fy
    P_cam = np.array([X, Y, Z])
    ```
    
    ## **2. 카메라 좌표계 → 로봇 기준(base_link) (P_base)**
    
    ```python
    self.R_cam_to_base = np.array([
        [0, 0, 1],   # Z_cam → X_base
        [-1, 0, 0],  # X_cam → -Y_base
        [0, -1, 0]   # Y_cam → -Z_base
    ], dtype=float)
    
    self.camera_offset = np.array([0.1, 0.0, 0.1])
    ```

  <img width="913" height="379" alt="image" src="https://github.com/user-attachments/assets/78e8bb7b-64a5-4690-9f9d-81585970de98" />

    ```cpp
    //코드 구현
    P_base = self.R_cam_to_base.dot(P_cam) + self.camera_offset
    ```
    
   ## **3. 로봇 기준(base_link) → 맵 기준(map) (P_map)**
  
    AMCL에서 들어온 로봇 포즈:
  
    ```python
    pos = np.array([p.x, p.y, p.z])     # 로봇의 map 좌표
    q   = (q.x, q.y, q.z, q.w)          # 로봇의 회전 (쿼터니언)
    R_map_base = self.quat_to_rot_matrix(*q)
    ```
    
    여기서 **`R_map_base`**는 **쿼터니언을 회전행렬로 바꾼 것**,**`pos`**는 맵 좌표계에서 로봇의 위치
    
    <img width="1012" height="284" alt="image" src="https://github.com/user-attachments/assets/6357c578-826a-4f10-918e-883012d8c0cc" />
    
    ```cpp
    //코드 구현
    P_map = R_map_base.dot(P_base) + pos
    ```

  - 결과 출력
    
    ### 로그
    📦 YOLO detected 1 objects
    
    🌐 Map coords: [0.29899437 0.67378386 0.30389404]
    
    📍 Marker published at map=[0.29899437 0.67378386 0.30389404]

    
![1000031519](https://github.com/user-attachments/assets/edeef7d7-1086-413c-8314-af90c7ee190b)
<img width="400" height="300" alt="1000031513" src="https://github.com/user-attachments/assets/55dd5e25-dcf3-42b3-99ab-5d48c2759246" />


    
