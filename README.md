# New Functions List

> 命名规则: 标准数据类型(如 `uint32_t`)将数据类型小写简写前置(如 `uiXXX`).

## Sensors Function

---

- `ping_distance`: 返回测得距离, 单位为 cm. 返回值为浮点数, 理论精度高于 `ping_cm`.
- `gyro_get`: 获取角速度和偏向角. 需要更新的指针 `fGyro`, `fAngle`. 放在 `gyro_get(fGyro, fAngle)`.
