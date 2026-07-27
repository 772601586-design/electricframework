# 底盘跟随模式自转修复记录

## 问题现象

- 当前车只有在枪管接近 `0` 位时切入底盘模式才正常。
- 当枪管不在 `0` 位时切入 `CHASSIS_FOLLOW_GIMBAL_YAW`，底盘会把当前固定偏角当成跟随误差，进入后立刻自转。
- 本质原因是原逻辑直接使用当前 `yaw` 相对配置零位的 `offset_angle` 作为底盘跟随误差，没有“切模式时自校正”。

## 修改文件

- `application/cmd/robot_cmd.c`

## 修改内容

### 1. 保留原始 yaw 偏角

在 `CalcOffsetAngle()` 中，不再只把计算结果直接覆盖到 `chassis_cmd_send.offset_angle`，而是先保存到新的静态变量：

- `chassis_raw_offset_angle`

对应位置：

- `application/cmd/robot_cmd.c:116`

这样可以区分：

- 原始机械偏角
- 发给底盘控制的补偿后偏角

### 2. 新增角度归一化函数

新增：

- `NormalizeAngle180(float angle)`

作用：

- 把角度限制到 `[-180, 180]`
- 避免做零点补偿后角度越界

对应位置：

- `application/cmd/robot_cmd.c:145`

### 3. 新增底盘跟随自校正逻辑

新增：

- `UpdateChassisFollowAlignment(void)`

逻辑如下：

- 记录上一次底盘模式 `last_chassis_mode`
- 在第一次进入 `CHASSIS_FOLLOW_GIMBAL_YAW` 时，把当前 `chassis_raw_offset_angle` 记为 `follow_zero_offset`
- 在跟随模式下，发送给底盘的角度改为：

`NormalizeAngle180(chassis_raw_offset_angle - follow_zero_offset)`

这意味着：

- 切入跟随模式时，当前枪管朝向会被当作这一次的临时零位
- 底盘不会因为“当前不是机械零位”而立刻自转找零
- 后续 yaw 再变化时，底盘仍然可以继续正常跟随

对应位置：

- `application/cmd/robot_cmd.c:154`

### 4. 在发送控制前应用补偿

在 `RefereeHandler()` 开头调用：

- `UpdateChassisFollowAlignment();`

这样可以保证在控制消息发给底盘前，`chassis_cmd_send.offset_angle` 已经是补偿后的值。

对应位置：

- `application/cmd/robot_cmd.c:542`

## 修复后的预期效果

- 枪管不在 `0` 位时切入底盘跟随模式，不会再立即原地自转。
- 当前朝向会被自动视为本次进入底盘模式的参考零位。
- 在底盘跟随模式下继续转动云台，底盘仍能按相对偏角正常跟随。

## 建议实车验证

1. 先把枪管停在非 `0` 位。
2. 切入底盘跟随模式，确认底盘不会立刻自转。
3. 再手动转动 yaw，确认底盘仍然跟随。
4. 多次在 `零力 <-> 跟随`、`小陀螺 <-> 跟随` 之间切换，确认每次进入跟随都会重新取当前朝向为参考零位。
