# 上位机装甲板三维模型重投影测试方案

## 1. 测试目标

定位以下链路中的具体错误：

1. 当前装甲板角点、PnP 或相机标定错误。
2. 机器人中心到其它装甲板的几何模型错误。
3. `yaw`、装甲板索引、旋转方向或坐标系定义错误。
4. `r`、`r1/r2`、`dz`、大小装甲板尺寸配置错误。
5. EKF、图像和重投影之间的时间戳或预测补偿错误。
6. 上下位机传输中的单位、延迟、丢帧或角度回绕问题。

本方案的原则是：一次只改变一个变量，所有测试都保存原始数据，能够离线重复执行。

## 2. 测试前固定版本和环境

记录以下信息，不允许测试过程中改变：

```text
上位机 commit：
下位机 commit：
相机型号和序列号：
图像分辨率：
检测帧率：
相机曝光/增益：
装甲板模型版本：
相机内参文件：
畸变模型：普通 pinhole / fisheye
```

测试期间关闭自动参数热更新、自动重新标定和自动切换装甲板尺寸。

## 3. 每帧必须保存的数据

建议使用 `frame_id` 关联图像和 CSV。每帧一行，至少包含以下字段。

### 3.1 图像和检测数据

```text
frame_id
image_timestamp_us
image_width, image_height
raw_image_path
display_image_path
crop_x, crop_y, crop_width, crop_height
scale_x, scale_y
undistort_enabled
armor_id
armor_type              # small / large
armor_num               # 3 / 4 / 其他
armor_index             # 当前板在环形模型中的索引
detector_confidence
corner_0_u, corner_0_v
corner_1_u, corner_1_v
corner_2_u, corner_2_v
corner_3_u, corner_3_v
```

角点顺序必须明确写入文档，例如：

```text
corner_0: 左上
corner_1: 右上
corner_2: 右下
corner_3: 左下
```

### 3.2 相机和 PnP 数据

```text
K[0..8]
D[0..n]
K_used_for_projection[0..8]
D_used_for_projection[0..n]
object_width
object_height
rvec_x, rvec_y, rvec_z
tvec_x, tvec_y, tvec_z
reprojection_rmse_px
reprojection_corner_error_0_px ... reprojection_corner_error_3_px
armor_normal_cam_x, armor_normal_cam_y, armor_normal_cam_z
```

### 3.3 三维模型数据

```text
car_center_x, car_center_y, car_center_z
armor_center_x, armor_center_y, armor_center_z
armor_yaw
car_yaw
yaw_offset
yaw_sign
rotation_direction       # +1 / -1
r, r1, r2
dz
model_armor_width
model_armor_height
```

每块生成的装甲板都保存：

```text
model_armor_id
model_center_x/y/z
model_normal_x/y/z
projected_center_u/v
projected_corner_0_u/v ... projected_corner_3_u/v
point_depth_z
```

### 3.4 状态和时序数据

```text
measurement_timestamp_us
ekf_state_timestamp_us
reprojection_timestamp_us
predict_dt_s
half_horizon_dt_s
latency_compensation_s
ekf_car_x/y/z
ekf_yaw
ekf_yaw_speed
ekf_yaw_acc
```

下位机侧同时记录：

```text
rx_dwt_count
command_age_us
vision_generation
vision_active
command_stale
yaw_ref_deg
yaw_speed_ff_rad_s
pitch_ref_deg
pitch_speed_ff_rad_s
```

当前协议中的 `Vision_Recv_s` 没有上位机采样时间戳，因此必须额外记录上位机发送时间和下位机接收时间，否则无法精确计算传输延迟。

## 4. 统一误差定义

### 4.1 当前装甲板四点重投影误差

```text
e_corner = sqrt((u - u_hat)^2 + (v - v_hat)^2)
RMSE = sqrt(sum(e_corner^2) / 4)
```

### 4.2 生成装甲板中心误差

如果其它装甲板也能被检测到：

```text
e_center = distance(projected_model_center, detected_armor_center)
```

### 4.3 方向误差

```text
e_angle = abs(projected_armor_axis_angle - detected_armor_axis_angle)
```

建议阈值：

```text
当前板静止重投影 RMSE：<= 3 px
生成板中心误差：<= max(5 px, 装甲板宽度的 5%)
生成板方向误差：<= 3 deg
静止帧连续 100 帧不得发生 ID/解跳变
```

阈值应根据相机分辨率和装甲板像素尺寸调整，但测试过程中必须保持一致。

## 5. 测试数据集

至少准备以下 6 组数据，每组保存原始图像和完整 CSV。

| 数据集 | 目标状态 | 用途 | 最少帧数 |
|---|---|---|---:|
| S0 | 目标静止，画面中心 | 基准测试 | 300 |
| S1 | 目标静止，画面左侧 | 检查主点和裁剪 | 300 |
| S2 | 目标静止，画面右侧 | 检查主点和裁剪 | 300 |
| S3 | 目标慢速旋转 | 检查时序 | 500 |
| S4 | 目标快速旋转 | 放大时序错误 | 500 |
| S5 | 不同距离：近/中/远 | 检查半径、深度和尺度 | 每档 300 |

旋转数据至少包含 0、1、2、4 rad/s 四档速度。速度应从陀螺仪或编码器实测，不要只使用发送目标值。

## 6. 测试 T1：只验证当前装甲板 PnP

### 操作

1. 关闭其它装甲板生成。
2. 关闭 EKF 预测、半时间段预测和延迟补偿。
3. 使用当前帧检测到的四个角点执行 PnP。
4. 用同一个 `rvec/tvec` 投影回同一张图像。
5. 输出四点误差、RMSE、`tvec.z` 和装甲板法向量。

### 通过标准

```text
tvec.z > 0
静止数据的 RMSE <= 3 px
四个角点没有固定方向的系统误差
连续 100 帧 rvec 不发生突然翻转
```

### 失败时检查

```text
角点顺序是否为左上、右上、右下、左下
object_points 和 image_points 是否一一对应
small/large 装甲板尺寸是否选反
projectPoints 是否使用了和输入图像匹配的 K、D
去畸变图是否又使用了原始畸变参数投影
是否误用 fisheye 参数调用普通 projectPoints
```

T1 失败时，禁止继续修改机器人中心和其它装甲板模型；先修复 PnP/标定链路。

## 7. 测试 T2：只验证三维环形模型

### 操作

1. 取 T1 的当前装甲板 PnP 结果。
2. 暂时固定 `car_center`、`car_yaw`、`r`、`dz`，不使用 EKF 预测。
3. 按当前模型生成其余装甲板。
4. 将机器人中心、每块装甲板中心和法向量画到图像上。
5. 对所有参数使用同一帧、同一坐标系。

### 必须检查的公式

环形布局应明确采用哪一种：

```text
theta_i = car_yaw + rotation_direction * 2*pi*i/N + yaw_offset
```

或使用当前装甲板反推机器人中心：

```text
p_car = p_armor - Rz(theta_k) * [r_k, 0, dz_k]^T
```

### 通过标准

```text
当前板模型中心与 PnP 中心误差 <= 3 px
可见其它装甲板中心误差 <= 5 px 或装甲板宽度的 5%
相邻板角度差接近 360/N deg
所有 point_depth_z > 0
```

### 误差判定

```text
所有板同方向固定偏移       -> yaw_offset 或坐标轴零点
左右互换/镜像               -> rotation_direction 或坐标系手性
相差一整块板               -> armor_index 或 N
水平方向距离不对            -> r/r1/r2
上下装甲板交替偏差          -> dz 或高低板类型
```

## 8. 测试 T3：参数枚举，不凭肉眼调参

对同一批静止帧自动枚举以下组合，并按全部可见装甲板的中心误差排序：

```text
N                  = 3, 4
rotation_direction = +1, -1
yaw_offset          = 0, +pi/2, -pi/2, pi
armor_index        = 0, 1, 2, 3
center_formula     = armor - offset, armor + offset
```

每组参数输出：

```text
valid_frame_count
mean_center_error_px
p95_center_error_px
mean_angle_error_deg
current_armor_error_px
```

如果某一组参数在 S0、S1、S2 都稳定优于其它组合，说明是确定性的几何定义错误；不要继续用 EKF 参数掩盖该问题。

## 9. 测试 T4：机器人中心一致性

对每一块可检测装甲板分别反算机器人中心：

```text
p_car_from_i = p_armor_i - Rz(theta_i) * [r_i, 0, dz_i]^T
```

计算不同装甲板反算中心之间的差：

```text
delta_car_ij = norm(p_car_from_i - p_car_from_j)
```

### 通过标准

```text
同一帧内不同装甲板反算的机器人中心差 <= 2 cm（按实际尺寸调整）
中心到各装甲板的水平距离接近对应 r_i
机器人中心到装甲板中心的方向与装甲板法向关系符合机械结构
```

如果不同装甲板反算出的中心明显分散，优先检查 `r_i`、`dz_i`、yaw 方向和大小板尺寸。

## 10. 测试 T5：去掉所有时间补偿

对 S0 和 S3/S4 分别运行两次：

```text
Run A:
predict_dt = 0
half_horizon_dt = 0
latency_compensation = 0

Run B:
恢复线上实际参数
```

### 判定

```text
Run A 静止仍错误       -> 几何、PnP 或标定问题
Run A 静止正确         -> 几何链路基本正确
Run A 旋转偏差较小，Run B 变差 -> 预测重复或延迟补偿错误
偏差约为 omega * dt    -> 时间差可由误差反推
```

例如实测 `omega=4 rad/s`，图像与状态相差 30 ms，理论角度误差约为：

```text
delta_theta = 4 * 0.030 = 0.12 rad = 6.87 deg
```

将实际误差除以实测角速度，可估算有效时间错位：

```text
estimated_dt = measured_angle_error / measured_omega
```

## 11. 测试 T6：时间戳和重复预测

每帧打印：

```text
image_timestamp
measurement_timestamp
ekf_state_timestamp
reprojection_timestamp
predict_dt
half_horizon_dt
latency_compensation
command_age_us
```

检查公式是否被重复使用：

```text
EKF 状态是否已经预测到当前时刻
重投影是否再次使用相同的 dt
图像延迟是否已经包含在 predict_dt 中
下位机是否又按接收延迟进行了一次传播
```

下位机当前会用接收时间作为轨迹锚点，并对轨迹进行最多 4 ms 的传播；视觉数据超过 10 ms 会被判定为过期。测试时要把 `command_age_us`、`vision_generation` 和 `command_stale` 一起记录。

## 12. 测试 T7：相机分辨率、裁剪和去畸变

分别对原始图像和显示图像做投影验证。

若运行图像由原图缩放得到：

```text
fx_new = scale_x * fx
fy_new = scale_y * fy
cx_new = scale_x * cx
cy_new = scale_y * cy
```

若运行图像经过裁剪：

```text
cx_new = cx_new - crop_x
cy_new = cy_new - crop_y
```

若图像已经去畸变：

```text
使用去畸变后的 K_new
投影时畸变参数设为 0
```

### 误差特征

```text
中心准确、边缘变差       -> K/D 或裁剪缩放错误
整幅图整体平移           -> cx/cy 或 crop 偏移错误
误差随半径非线性增加     -> 畸变模型不匹配
原图正确、显示图错误     -> 显示图坐标变换漏乘 scale/offset
```

## 13. 测试 T8：单位、协议和下位机执行链路

固定发送一组可预测的数据：

```text
yaw     = 0.10 rad
yaw_vel = 1.00 rad/s
yaw_acc = 0.00 rad/s^2
pitch   = 0.00 rad
```

预期在 10 ms 后：

```text
yaw_ref = 0.11 rad = 6.302 deg
```

检查：

```text
上位机发送的是 rad 还是 deg
速度是否为 rad/s
角度是否在 [-pi, pi] 回绕
下位机 vision_generation 是否连续增加
command_stale 是否异常置 1
```

当前下位机轨迹接口内部按弧度接收，最终才转换为电机角度；因此不能把角度制 yaw 直接写入视觉轨迹字段。

## 14. 最终问题定位表

| 现象 | 首要怀疑 |
|---|---|
| 当前板四角 RMSE 大 | 角点顺序、尺寸、PnP、K/D |
| 当前板准确，其它板固定角度错 | `yaw_offset`、yaw 定义、索引、N |
| 其它板左右镜像 | 坐标系手性或旋转正负号 |
| 方向正确但距离错误 | 机器人中心、r/r1/r2 |
| 上下板交替偏差 | dz、大小板分类、高度坐标 |
| 静止准确，旋转越来越错 | 时间戳、预测或延迟补偿 |
| 画面边缘误差大 | 分辨率、裁剪、主点、畸变 |
| yaw 接近 pi 时跳变 | 角度 unwrap |
| 下位机偶发停止跟随 | CRC、丢帧、command_age_us、过期判定 |
| 固定速度下误差恒定 | 单位转换或固定时间偏移 |

## 15. 推荐执行顺序

```text
第 1 步：完成 T1，只确认当前板 PnP 和标定
第 2 步：完成 T2，关闭预测验证几何模型
第 3 步：完成 T3，枚举 N、索引、旋转方向和 yaw 偏置
第 4 步：完成 T4，验证机器人中心、半径和 dz
第 5 步：完成 T5/T6，恢复预测并定位时间错位
第 6 步：完成 T7，验证裁剪、缩放和畸变
第 7 步：完成 T8，验证单位和下位机传输
```

在 T1 未通过前，不修改 `r`、`dz` 或 EKF 参数；在 T2/T4 未通过前，不修改时间补偿参数。


