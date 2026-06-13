# Vision-Based ArUco Landing Code Explanation

이 문서는 `marker_recognition.py`와 `landing.cpp` 두 개의 ROS 2 노드가 어떻게 연동되어 비전 기반 자동 착륙을 수행하는지 정리한 문서이다.

현재 시스템의 핵심 구조는 다음과 같다.

```text
SIYI A8 mini camera + ArUco marker detection
        ↓
marker_recognition.py
        ↓  /landing/coordinates
landing.cpp
        ↓  /fmu/in/offboard_control_mode
        ↓  /fmu/in/trajectory_setpoint
        ↓  /fmu/in/vehicle_command
PX4 Offboard control
```

---

## 1. 전체 시스템 개요

이 시스템은 카메라 영상에서 ArUco 마커를 검출하고, 마커 중심이 카메라 화면 중심으로부터 얼마나 떨어져 있는지 계산한 뒤, 그 상대 위치 오차를 이용해 드론이 마커 중심으로 접근하도록 제어한다.

역할은 크게 두 노드로 나뉜다.

| 파일 | 노드 이름 | 역할 |
|---|---|---|
| `marker_recognition.py` | `marker_recognition` | 카메라 영상 수신, ArUco 검출, 거리센서 고도 측정, 마커 상대좌표 계산, `/landing/coordinates` publish |
| `landing.cpp` | `landing` | `/landing/coordinates`를 받아 PX4 Offboard setpoint 생성, 위치/속도 기반 접근, 하강 제어, 최종 NAV_LAND 명령 |

---

## 2. `marker_recognition.py` 설명

`marker_recognition.py`는 비전 인식 및 센서 처리 노드이다. 주요 기능은 다음과 같다.

1. SIYI A8 mini RTSP 영상 수신
2. ArUco marker 검출
3. 화면 중심과 마커 중심 사이의 pixel error 계산
4. 고도값을 이용해 pixel error를 meter 단위 위치 오차로 변환
5. Kalman filter로 좌표를 필터링
6. 현재 카메라 yaw 장착 방향을 고려해 카메라 좌표계를 기체 body 좌표계로 변환
7. `/landing/coordinates` topic으로 publish
8. LIDAR-Lite v3HP 거리센서 값을 PX4 distance sensor topic으로 publish
9. SIYI A8 mini zoom 제어
10. 디버그 영상 `/landing/video/compressed` publish

---

### 2.1 카메라 영상 수신

카메라는 SIYI A8 mini RTSP stream을 GStreamer pipeline으로 받아온다.

```python
rtspsrc location=rtsp://192.168.0.20:8554/main.264
```

현재 pipeline은 H.265 stream을 기준으로 작성되어 있다.

```python
rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! video/x-raw,format=BGR ! appsink
```

OpenCV의 `cv2.VideoCapture(..., cv2.CAP_GSTREAMER)`를 사용해서 프레임을 읽는다.

---

### 2.2 ArUco marker 검출

ArUco dictionary는 다음을 사용한다.

```python
cv2.aruco.DICT_4X4_50
```

마커 검출은 `_detect_first_tag()`에서 수행된다. 검출 안정성을 위해 한 가지 영상만 사용하는 것이 아니라 다음 세 가지 전처리를 순서대로 시도한다.

```text
1. raw gray image
2. CLAHE 적용 image
3. gamma correction image
```

각 전처리 결과에 대해 ArUco를 검출하고, 여러 개가 검출될 경우 `_select_largest_marker_center()`에서 면적이 가장 큰 마커를 선택한다. 선택된 마커의 네 꼭짓점 평균값을 이용해 마커 중심 좌표 `(cx, cy)`를 계산한다.

---

### 2.3 pixel error 계산

카메라 이미지에서 화면 중심은 다음과 같이 계산된다.

```python
height, width = frame.shape[:2]
cx0 = width / 2.0
cy0 = height / 2.0
```

마커 중심이 `(cx, cy)`일 때 pixel error는 다음과 같다.

```python
dx_px = cx - cx0
dy_px = cy0 - cy
```

여기서 의미는 다음과 같다.

| 값 | 의미 |
|---|---|
| `dx_px > 0` | 마커가 화면 오른쪽에 있음 |
| `dx_px < 0` | 마커가 화면 왼쪽에 있음 |
| `dy_px > 0` | 마커가 화면 위쪽에 있음 |
| `dy_px < 0` | 마커가 화면 아래쪽에 있음 |

---

### 2.4 pixel 좌표를 meter 좌표로 변환

카메라 calibration matrix의 초점거리 `fx`, `fy`와 고도 `z`를 이용해 pixel error를 meter 단위로 변환한다.

```python
raw_x_m = dx_px / fx * z
raw_y_m = dy_px / fy * z
```

현재 코드에서는 1x zoom과 4x zoom에 대해 각각 다른 camera matrix와 distortion coefficient를 사용한다.

```python
_CAMERA_MATRIX_1X
_DIST_COEFFS_1X
_CAMERA_MATRIX_4X
_DIST_COEFFS_4X
```

현재 zoom factor가 4x에 가까우면 4x calibration을 사용하고, 그렇지 않으면 1x calibration을 사용한다.

```python
if abs(self._current_zoom_factor - 4.0) < 0.2:
    return self._camera_matrix_4x, self._dist_coeffs_4x

return self._camera_matrix_1x, self._dist_coeffs_1x
```

---

### 2.5 Kalman filter

`TargetKalman2D` 클래스는 2D 위치와 속도를 추정하는 Kalman filter이다.

상태벡터는 다음과 같다.

```text
x = [x_m, y_m, vx_mps, vy_mps]^T
```

측정값은 다음과 같다.

```text
z = [raw_x_m, raw_y_m]^T
```

마커가 정상적으로 검출되면 `update()`를 통해 측정값을 반영하고, 마커가 잠깐 사라지면 `predict_only()`로 예측값을 유지한다.

주의할 점은 `target_predict_timeout` 동안 마커가 사라져도 예측 좌표가 계속 publish될 수 있다는 것이다. 실제 비행 중 마커를 잃은 상태에서 너무 오래 예측값을 사용하면 드론이 잘못된 방향으로 계속 이동할 수 있다. 따라서 실비행에서는 `target_predict_timeout`을 너무 길게 두지 않는 것이 좋다.

예시:

```bash
ros2 param set /marker_recognition target_predict_timeout 0.5
```

---

### 2.6 현재 카메라 yaw 장착 방향 보정

현재 코드에서 가장 중요한 부분이다.

현재 기체에서는 카메라 yaw가 기체 yaw와 동일하게 정렬되어 있지 않고, **기체 yaw 기준 반시계방향으로 90도 돌아가 있는 상태**를 가정한다.

즉, 카메라 좌표계와 기체 body 좌표계의 관계는 다음과 같다.

```text
camera yaw = body yaw - 90 deg
```

비전에서 계산된 필터링 좌표는 카메라 기준이다.

```text
self.x_m = camera right(+)
self.y_m = camera forward(+)
```

하지만 `landing.cpp`는 `/landing/coordinates`를 다음과 같은 body 기준 좌표로 받는다고 가정한다.

```text
point.x = body right(+)
point.y = body forward(+)
```

따라서 Python 노드에서 publish하기 전에 좌표계를 변환한다.

현재 적용된 변환은 다음과 같다.

```python
body_right_m = -float(self.y_m)
body_forward_m = float(self.x_m)

msg.point.x = body_right_m
msg.point.y = body_forward_m
msg.point.z = float(self._altitude)
```

좌표 변환 의미는 다음과 같다.

```text
body_forward =  camera_right
body_right   = -camera_forward
```

즉, 마커가 카메라 화면 오른쪽에 있으면 기체 기준으로는 전방 오차가 있는 것이고, 마커가 카메라 화면 위쪽에 있으면 기체 기준으로는 왼쪽 방향 오차가 있는 것으로 해석된다.

---

### 2.7 매우 중요한 주의사항: 카메라 yaw 정렬이 바뀌면 이 부분을 수정해야 함

현재 `marker_recognition.py`는 **카메라가 기체 yaw 기준 반시계방향 90도 돌아가 있는 장착 상태**를 코드에 직접 반영한 버전이다.

따라서 나중에 카메라와 기체 yaw가 동일하게 정렬되면, 현재의 변환을 그대로 사용하면 안 된다.

카메라 yaw와 기체 yaw가 같아지는 경우에는 `/landing/coordinates` publish 부분을 아래처럼 바꿔야 한다.

```python
# 카메라 yaw와 기체 yaw가 동일하게 정렬된 경우
# self.x_m = body right(+)
# self.y_m = body forward(+)

msg.point.x = float(self.x_m)
msg.point.y = float(self.y_m)
msg.point.z = float(self._altitude)
```

즉, 현재 코드의 이 부분:

```python
body_right_m = -float(self.y_m)
body_forward_m = float(self.x_m)

msg.point.x = body_right_m
msg.point.y = body_forward_m
```

를 아래처럼 단순화해야 한다.

```python
msg.point.x = float(self.x_m)
msg.point.y = float(self.y_m)
```

정리하면 다음과 같다.

| 카메라 장착 상태 | `/landing/coordinates` publish |
|---|---|
| 현재 상태: 카메라 yaw가 기체 기준 반시계 90도 | `point.x = -self.y_m`, `point.y = self.x_m` |
| 추후 상태: 카메라 yaw와 기체 yaw가 동일 | `point.x = self.x_m`, `point.y = self.y_m` |

---

### 2.8 `/landing/coordinates` topic

최종적으로 `marker_recognition.py`는 다음 topic을 publish한다.

```text
/landing/coordinates
```

메시지 타입은 다음과 같다.

```text
geometry_msgs/msg/PointStamped
```

현재 코드에서 publish되는 의미는 다음과 같다.

```text
point.x = body right(+), [m]
point.y = body forward(+), [m]
point.z = altitude, [m]
```

이 convention은 `landing.cpp`와 반드시 일치해야 한다.

---

### 2.9 LIDAR altitude 처리

LIDAR-Lite v3HP는 I2C bus를 통해 값을 읽는다.

```python
I2C_BUS = 7
LIDAR_ADDR = 0x62
```

측정된 거리값은 cm 단위로 읽고, meter로 변환한다.

```python
distance_m = distance_cm / 100.0
```

이후 LIDAR 장착 높이를 빼서 실제 기체 기준 고도를 계산한다.

```python
raw_altitude = distance_m - self._lidar_altitude
```

기체 roll/pitch가 너무 크면 LIDAR 값이 지면 수직 거리가 아닐 가능성이 높으므로 attitude gate를 적용한다.

```python
lidar_attitude_gate_deg = 20.0
```

roll 또는 pitch가 gate를 넘으면 현재 raw altitude를 바로 쓰지 않고 이전 altitude를 유지한다.

---

### 2.10 SIYI A8 mini gimbal / zoom 제어

코드에는 SIYI A8 mini에 UDP packet을 보내는 `SiyiA8MiniUDP` 클래스가 포함되어 있다.

주요 기능은 다음과 같다.

| 기능 | 설명 |
|---|---|
| `absolute_zoom()` | 지정한 zoom factor로 SIYI A8 mini zoom 제어 |
| `_check_auto_zoom_by_altitude_direct_udp()` | 고도에 따라 1x ↔ 4x 자동 zoom 전환 |
| `_set_gimbal_pitch_down()` | PX4 gimbal manager command로 pitch -90도 명령 |

고도가 `auto_zoom_threshold_m` 이상이면 4x zoom으로 전환하고, 낮아지면 다시 1x로 복귀한다.

---

## 3. `landing.cpp` 설명

`landing.cpp`는 PX4 Offboard 제어 노드이다. 이 노드는 `/landing/coordinates`에서 받은 body 기준 상대 위치 오차를 이용해 드론을 ArUco marker 중심으로 접근시키고, 조건이 만족되면 하강 및 최종 착륙 명령을 보낸다.

---

### 3.1 주요 subscribe topic

| Topic | Type | 역할 |
|---|---|---|
| `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` | 현재 기체 위치와 자세 quaternion 수신 |
| `/fmu/out/vehicle_land_detected` | `px4_msgs/msg/VehicleLandDetected` | 착륙 여부 확인 |
| `/landing/coordinates` | `geometry_msgs/msg/PointStamped` | 비전 노드에서 계산한 body 기준 마커 위치 오차 수신 |

`/landing/coordinates`를 받을 때 의미는 다음과 같다.

```cpp
desired_x_ = msg->point.x;   // body right(+), [m]
desired_y_ = msg->point.y;   // body forward(+), [m]
acc_alt_ = -msg->point.z;    // existing convention
```

여기서 `desired_x_`와 `desired_y_`는 이미 Python 노드에서 body 기준으로 변환되어 들어온 값이다.

---

### 3.2 주요 publish topic

| Topic | Type | 역할 |
|---|---|---|
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | PX4에 현재 어떤 setpoint를 사용할지 전달 |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | 위치 또는 속도 setpoint 전달 |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | ARM, OFFBOARD mode, NAV_LAND 등 명령 전달 |
| `/mission_mode` | `std_msgs/msg/String` | 현재 mission state를 다른 노드에 알림 |

---

### 3.3 Mission state

`landing.cpp`는 세 가지 mission state를 사용한다.

```cpp
enum Mission {
  FLIGHT,
  LANDING,
  FINISHED,
};
```

| State | 의미 |
|---|---|
| `FLIGHT` | 시작 waypoint로 이동하는 단계 |
| `LANDING` | 비전 기반 정밀 접근 및 하강 단계 |
| `FINISHED` | 착륙 완료 또는 종료 단계 |

`start_param`이 0이면 시작하자마자 바로 `LANDING`으로 진입한다.

```cpp
if (start_mode_ == 0 && mission_mode_ == FLIGHT) {
  mission_mode_ = LANDING;
}
```

`start_param`이 1이면 `start_x`, `start_y`, `start_z` 위치로 먼저 이동한 뒤 착륙을 시작한다.

---

### 3.4 Offboard 진입 절차

PX4 Offboard mode는 mode 변경 전에 setpoint stream이 먼저 들어오고 있어야 안정적으로 진입한다. 그래서 코드에서는 처음부터 setpoint를 publish하면서 `preflight_setpoint_count_`를 증가시킨다.

일정 횟수 이상 setpoint를 보낸 뒤 OFFBOARD mode를 요청한다.

```cpp
publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
```

이후 ARM 명령을 보낸다.

```cpp
publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
```

---

### 3.5 Landing mode

착륙 제어 방식은 두 가지이다.

```cpp
enum LandingMode {
  POSITION_XY_VELOCITY_Z = 0,
  VELOCITY_XYZ = 1,
};
```

| 값 | 모드 | 설명 |
|---|---|---|
| `0` | `POSITION_XY_VELOCITY_Z` | x/y는 위치 setpoint, z는 velocity setpoint |
| `1` | `VELOCITY_XYZ` | x/y/z 모두 velocity setpoint |

현재 기본값은 velocity 기반이다.

```cpp
int land_mode_ = VELOCITY_XYZ;
```

---

## 4. Velocity 기반 착륙 제어

`land_mode_ == VELOCITY_XYZ`일 때 사용된다.

---

### 4.1 위치 오차 계산

Python 노드에서 들어온 좌표는 이미 body 기준이다.

```cpp
const float ex = valid_xy ? desired_x_ : 0.0f;  // body right(+)
const float ey = valid_xy ? desired_y_ : 0.0f;  // body forward(+)
const float err_dist = std::sqrt(ex * ex + ey * ey);
```

즉:

```text
ex = 오른쪽 방향 오차
ed = 전방 방향 오차
err_dist = 수평 거리 오차
```

실제 코드 변수명은 `ey`이고, 전방 방향 오차를 의미한다.

---

### 4.2 tanh 기반 속도 포화

거리 오차가 커질수록 속도 명령이 증가하지만, `max_xy_` 이상 커지지 않도록 `tanh()`를 사용한다.

```cpp
v_close = max_xy_ * std::tanh(tanh_gain_ * err_dist);
v_close = clamp_range(v_close, tanh_min_xy_, max_xy_);
```

의미는 다음과 같다.

| 파라미터 | 의미 |
|---|---|
| `max_xy_` | x/y 최대 접근 속도 |
| `tanh_gain_` | 오차에 대한 속도 증가 민감도 |
| `tanh_min_xy_` | deadband 밖에서 적용할 최소 접근 속도 |
| `deadband_m_` | 이 거리 이하에서는 x/y 제어를 거의 0으로 둠 |

이후 방향 단위벡터를 곱해서 body 기준 속도를 만든다.

```cpp
v_right = clamp_symmetric(v_close * ux, max_xy_);
v_forward = clamp_symmetric(v_close * uy, max_xy_);
```

그리고 body FRD 속도를 NED 좌표계로 변환한다.

```cpp
Eigen::Vector3f v_body(v_forward, v_right, 0.0f);
Eigen::Vector3f v_ned = body_frd_to_ned(v_body);
```

최종적으로 PX4에 velocity setpoint를 보낸다.

```cpp
msg.position = {nan_, nan_, nan_};
msg.velocity = {v_ned[0], v_ned[1], descent_mps};
```

---

## 5. Position 기반 착륙 제어

`land_mode_ == POSITION_XY_VELOCITY_Z`일 때 사용된다.

이 모드에서는 x/y 방향을 속도 명령으로 직접 주는 대신, 현재 위치에서 조금 이동한 target position을 계산해 PX4에 보낸다. 하지만 z축 하강은 여전히 velocity 기반이다.

---

### 5.1 atan 기반 position step

거리 오차가 커질수록 더 큰 position step을 만들지만, 너무 큰 위치 이동 명령이 들어가지 않도록 `atan()`으로 포화시킨다.

```cpp
xy_step =
  position_step_max_m_ *
  (2.0f / static_cast<float>(M_PI)) *
  std::atan(atan_position_gain_ * err_dist);

xy_step = clamp_range(xy_step, position_step_min_m_, position_step_max_m_);
```

방향 단위벡터를 곱해 body 기준 step을 만든다.

```cpp
const float step_right = xy_step * ux;
const float step_forward = xy_step * uy;
```

그리고 body FRD 기준 위치 step을 NED로 변환해 현재 위치에 더한다.

```cpp
Eigen::Vector3f target_body_frd = Eigen::Vector3f(step_forward, step_right, 0.0f);
Eigen::Vector3f target_ned = current_ned + body_frd_to_ned(target_body_frd);
```

최종적으로 x/y position setpoint와 z velocity setpoint를 같이 보낸다.

```cpp
msg.position = {target_ned[0], target_ned[1], nan_};
msg.velocity = {nan_, nan_, descent_mps};
```

---

## 6. 고도 하강 로직

z축 하강 속도는 `select_descent_speed()`에서 결정된다.

```cpp
float LandingTest::select_descent_speed(float alt_m, bool valid_xy) const
```

현재 코드는 마커가 valid하고, 일정 횟수 이상 alignment가 유지되어야 하강을 시작한다.

```cpp
if (!valid_xy || hold_counter_ < align_need_) {
  return 0.0f;
}
```

하강 속도는 고도에 따라 세 단계로 나뉜다.

```cpp
if (alt_m > 2.0f) {
  return descent_high_mps_;
}

if (alt_m > 0.8f) {
  return descent_mid_mps_;
}

return descent_low_mps_;
```

기본 의미는 다음과 같다.

| 고도 | 하강 속도 |
|---|---|
| 2.0 m 초과 | `descent_high_mps_` |
| 0.8 m 초과 ~ 2.0 m 이하 | `descent_mid_mps_` |
| 0.8 m 이하 | `descent_low_mps_` |

---

## 7. Alignment 판단

수평 오차가 허용 범위 안에 들어오면 aligned 상태로 판단한다.

```cpp
const bool aligned =
  valid_xy &&
  std::fabs(desired_x_) < tol_m_ &&
  std::fabs(desired_y_) < tol_m_;
```

aligned 상태가 연속으로 유지되면 `hold_counter_`가 증가한다.

```cpp
hold_counter_ = aligned ? hold_counter_ + 1 : 0;
```

`hold_counter_ >= align_need_`가 되어야 하강 속도가 0이 아닌 값으로 설정된다.

---

## 8. Target lost 처리

마커 좌표가 finite하지 않으면 target lost로 판단한다.

```cpp
if (!valid_xy) {
  lost_count_++;
  hold_counter_ = 0;
}
```

`lost_count_`가 `lost_abort_`보다 커지면 PX4를 수동 position mode로 전환하고 mission을 종료한다.

```cpp
publish_vehicle_command(
  VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
  1.0f,
  3.0f);

mission_mode_ = FINISHED;
```

주의할 점은 Python 노드에서 Kalman prediction을 계속 publish하면 C++ 노드 입장에서는 target lost가 아닌 것처럼 보일 수 있다는 것이다. 따라서 prediction timeout을 너무 길게 설정하지 않는 것이 중요하다.

---

## 9. 최종 NAV_LAND 조건

드론이 충분히 낮은 고도까지 내려오고, 마커 중심 정렬이 유지되면 PX4에 NAV_LAND 명령을 보낸다.

```cpp
if (
  valid_xy &&
  hold_counter_ >= align_need_ &&
  acc_alt_ > low_enough_ &&
  !nav_land_sent_) {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
}
```

현재 코드에서는 `acc_alt_ = -msg->point.z`로 저장하고 있으므로, 고도 관련 부호 convention을 수정할 때는 이 조건도 함께 확인해야 한다.

---

## 10. 좌표계 전체 정리

현재 시스템의 좌표계 흐름은 다음과 같다.

```text
[카메라 영상]
마커 pixel center
        ↓
[pixel error]
dx_px = cx - cx0
dy_px = cy0 - cy
        ↓
[카메라 기준 meter 좌표]
self.x_m = camera right(+)
self.y_m = camera forward(+)
        ↓
[현재 장착 상태 보정: camera yaw = body yaw - 90 deg]
body_right   = -camera_forward
body_forward =  camera_right
        ↓
[/landing/coordinates]
point.x = body right(+)
point.y = body forward(+)
point.z = altitude
        ↓
[landing.cpp]
desired_x_ = body right(+)
desired_y_ = body forward(+)
        ↓
[PX4 setpoint]
body FRD → NED 변환 후 trajectory_setpoint publish
```

---

## 11. 지상 검증 방법

실제 비행 전에 반드시 `/landing/coordinates`를 echo하면서 좌표 방향을 확인해야 한다.

```bash
ros2 topic echo /landing/coordinates
```

현재 카메라가 반시계 90도 돌아간 상태에서 기대되는 결과는 다음과 같다.

| 마커 위치 | 기대되는 `/landing/coordinates` 변화 |
|---|---|
| 화면 오른쪽 | `point.y` 양수 증가, 즉 body forward |
| 화면 왼쪽 | `point.y` 음수 증가, 즉 body backward |
| 화면 위쪽 | `point.x` 음수 증가, 즉 body left |
| 화면 아래쪽 | `point.x` 양수 증가, 즉 body right |

이 방향이 맞으면 Python 노드의 yaw 보정은 정상이다.

---

## 12. 자주 조정할 파라미터

### `marker_recognition.py`

| Parameter | 의미 | 권장 확인 사항 |
|---|---|---|
| `frame_rate` | 카메라 처리 주기 | 실제 카메라 FPS와 맞추기 |
| `lidar_altitude` | LIDAR 장착 높이 | 실측값과 맞춰야 altitude 오차 감소 |
| `lidar_attitude_gate_deg` | roll/pitch 허용각 | 너무 작으면 고도값이 자주 hold됨 |
| `auto_zoom_threshold_m` | zoom 전환 고도 | 1x/4x calibration과 함께 확인 |
| `target_predict_timeout` | 마커 lost 후 예측 유지 시간 | 실비행에서는 짧게 권장 |
| `target_kf_process_var` | Kalman process noise | 반응성 조정 |
| `target_kf_measurement_var` | Kalman measurement noise | 필터링 강도 조정 |

### `landing.cpp`

| Parameter | 의미 | 영향 |
|---|---|---|
| `land_param` | 착륙 제어 방식 | 0: 위치 기반, 1: 속도 기반 |
| `max_xy_` | 최대 수평 접근 속도 | 클수록 빠르지만 overshoot 가능 |
| `tanh_gain_` | 속도 증가 민감도 | 클수록 공격적 접근 |
| `tanh_min_xy_` | 최소 수평 속도 | 너무 크면 중심 근처에서 흔들림 가능 |
| `tol_m_` | aligned 판단 거리 | 너무 크면 덜 맞은 상태에서 하강 |
| `align_need_` | alignment 유지 count | 클수록 안정적이지만 하강 시작 느림 |
| `deadband_m_` | x/y 제어 deadband | 중심 근처 미세 움직임 억제 |
| `descent_high_mps_` | 높은 고도 하강 속도 | 빠른 하강 |
| `descent_mid_mps_` | 중간 고도 하강 속도 | 중간 접근 |
| `descent_low_mps_` | 낮은 고도 하강 속도 | 착지 직전 안정성 |
| `lost_abort_` | target lost 허용 count | 너무 크면 lost 상황에서 오래 움직임 |

---

## 13. 추천 테스트 순서

1. 드론 프로펠러 제거
2. `marker_recognition.py` 실행
3. `/landing/video/compressed`로 마커 검출 확인
4. `/landing/coordinates` echo 확인
5. 마커를 화면 오른쪽/왼쪽/위/아래로 움직이며 body 좌표 변환 확인
6. `landing.cpp` 실행 전 `desired_x_`, `desired_y_` 방향이 맞는지 확인
7. 저고도 hover 상태에서 `max_xy_`를 작게 두고 접근 테스트
8. 속도 방향이 맞으면 `max_xy_`, `tanh_gain_`, `tol_m_`, `align_need_` 튜닝
9. 마지막에 하강 속도 튜닝

---

## 14. 핵심 요약

- `marker_recognition.py`는 카메라 영상에서 ArUco marker를 검출하고, 마커 중심의 상대 위치를 계산한다.
- 현재 Python 노드는 **카메라 yaw가 기체 yaw 기준 반시계방향 90도 돌아간 장착 상태**를 반영해 `/landing/coordinates`를 body 기준으로 publish한다.
- 현재 변환은 `point.x = -self.y_m`, `point.y = self.x_m`이다.
- 나중에 카메라 yaw와 기체 yaw가 동일하게 정렬되면 이 변환을 제거하고 `point.x = self.x_m`, `point.y = self.y_m`로 바꿔야 한다.
- `landing.cpp`는 `/landing/coordinates`가 이미 body 기준이라고 가정하고, 이를 이용해 PX4 Offboard setpoint를 생성한다.
- x/y 접근은 velocity 기반 또는 position 기반으로 선택할 수 있고, z축 하강은 항상 velocity 기반이다.
- 마커 lost 상황에서 Kalman prediction이 너무 오래 유지되면 위험할 수 있으므로 `target_predict_timeout`을 실비행에 맞게 짧게 조정하는 것이 좋다.
