# 0x24 reject_reason 编码表

0x24 是 H723 上报给小电脑的发射状态，不是开火命令。

## 字段

| 字段 | 说明 |
|---|---|
| can_fire | 1 = 当前允许视觉请求开火；0 = 不允许。只代表许可，不代表击发。 |
| reject_reason | can_fire=0 的原因编码，见下表。 |
| bullet_speed_mmps | 弹速，mm/s；0 表示未知。 |
| launcher_state | 0 未安装，1 已安装未锁，2 首发未确认，3 首发已确认，255 未知或故障。 |

## 编码

| 编码 | 名称 | 类别 | 含义 |
|---|---|---|---|
| 0 | NONE | 正常 | 无拒绝原因 |
| 1 | FRICTION_NOT_READY | 许可条件 | 摩擦轮未达到目标转速或未稳定 |
| 2 | HEAT_LIMIT | 许可条件 | 热量达到上限或冷却中 |
| 3 | TRIGGER_NOT_READY | 许可条件 | 拨盘未就绪或正在复位 |
| 4 | TRIGGER_FAULT | 许可条件 | 拨盘堵转、离线或状态异常 |
| 5 | RC_OFFLINE | 许可条件 | 遥控链路失联 |
| 6 | REFEREE_NOT_READY | 许可条件 | 裁判系统未接入或未给出最终开火权限 |
| 7 | SYSTEM_FAULT | 许可条件 | 整车安全状态异常 |
| 8 | NO_AMMO | 诊断 | 无弹，仅上报，不参与 can_fire 判断 |
| 9 | LAUNCHER_NOT_INSTALLED | 诊断 | 发射机构未安装，仅上报 |
| 10 | VISION_NOT_ACTIVE | 诊断 | 视觉未处于自瞄模式，仅上报 |
| 255 | UNKNOWN | 诊断 | 原因未知 |

## 判定顺序

1. 裁判系统未接入时，can_fire 固定 0，reject_reason 填 6。
2. 裁判系统接入后，按摩擦轮、热量、拨盘、RC 在线依次判定；任一不满足则填对应编码。
3. 全部通过后 can_fire=1，reject_reason=0。
4. 无弹、发射机构未安装、视觉未激活只做诊断，不阻止 can_fire。
5. 最终是否击发仍由 H723 的 shoot_task 决定，0x24 不直接控制扳机。

## 当前占位

发射机构与裁判系统未接入，固件发送：

```text
can_fire = 0
reject_reason = 6
bullet_speed_mmps = 0
launcher_state = 255
```

对应代码位置：`USER/Task/Transmission/transmission_task.c` 的 `TRANS_SHOOT_*_PLACEHOLDER` 宏。
