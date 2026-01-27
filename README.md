# LIMO_ROS2_PROJECT

https://github.com/realsenseai/realsense-ros/tree/ros2-development 카메라 패키지 다운 참고

## 맵 만들기
1. **리모 가동**

    ```bash
    ros2 launch limo_bringup limo_start.launch.py
    ```

2. **맵 작성**

   ```bash
   ros2 launch limo_bringup cartographer.launch.py
   ```

3. **맵 저장**

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/map
   ```

## 메인 프로젝트
1. **리모 가동**
    
    ```bash
    ros2 launch limo_bringup limo_start.launch.py
    ```
    
2. **지도 불러오기**
    
    ```bash
    ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/agilex/map01.yaml
    ```
    
3. **Localization 실행 (AMCL).**
    
    ```bash
    ros2 launch limo_bringup limo_nav2_ackerman.launch.py map:=/home/agilex/map01.yaml use_sim_time:=true
    ```
    
4. **Realsense 카메라 실행**
    
    ```bash
    ros2 launch realsense2_camera rs_launch.py
    ```
    
5. **YOLO+Depth→Map 마커 코드 실행**
    
    ```bash
    /bin/python /home/agilex/limo21/src/code.py
    ```
    
6. **RViz 실행**
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
