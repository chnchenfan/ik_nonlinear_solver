# 接口说明

## 构造

```python
from nonlinear_ik import NonlinearPinocchioIK, NonlinearIKConfig

solver = NonlinearPinocchioIK(
    urdf_path="model.urdf",   # 机械臂 URDF(纯运动学即可);缺省用自带示例模型
    ee_frame="F_Link",        # 末端 frame 名;缺省对应自带示例模型
    config=None,              # NonlinearIKConfig;None 用默认
)
```

关节限位从 URDF 读取,构造后可用属性 `nq`、`q_lower`、`q_upper`、`q_center` 访问。

## 主接口

### `solve(target_position, target_rotation=None, q_init=None) -> (q, ok)`

求解关节命令。用于控制循环:每周期给当前关节角与目标,取回应发布的关节命令。

| 参数 | 说明 |
|---|---|
| `target_position` | 目标末端位置 `(3,)`,base 坐标系,单位 m;也可传 `(n, 3)` 给每步一个目标 |
| `target_rotation` | 可选目标姿态 `(3,3)` 旋转矩阵;`None` 则只跟踪位置 |
| `q_init` | 当前/初始关节角 `(nq,)`,用于热启动;`None` 用 neutral 位姿 |

返回 `(q, ok)`:`q` 为关节命令(已钳限位),`ok` 表示解可用。求解耗时、迭代数、
避障 margin 等详细诊断在调用后从 `solver.last_info` 读取。

### `forward(q) -> (position, rotation)`

正运动学。给定关节角,返回末端位置 `(3,)` 与旋转矩阵 `(3,3)`。

## 高级接口(可选)

| 方法 / 属性 | 返回 | 用途 |
|---|---|---|
| `step(target, q_current, warm_start=None, target_rotation=None)` | `(q, ok, info)` | 直接拿第一步命令 + 完整诊断 `info` |
| `solve_horizon(target, q_init=None, warm_start=None, target_rotation=None)` | `(q_N, ok, info)` | 求解多步关节序列,返回末端一步 + 诊断 |
| `last_info` | `dict` | 最近一次 `solve()` 的诊断 |
| `last_plan()` | `(n, nq)` 或 None | 上一轮多步序列副本 |

`info` / `last_info` 常用字段:`solve_time`(求解耗时 s)、`nit`(迭代数)、
`terminal_error`(末端到目标误差)、`min_obstacle_margin`、`obstacle_violation_max`。

## 配置 `NonlinearIKConfig`

按需覆盖,未列出的用默认即可。

**核心**

| 字段 | 默认 | 说明 |
|---|---|---|
| `horizon_steps` | 8 | 求解的关节序列步数;主接口只取第一步 |
| `control_dt` | 0.02 | 控制周期(s),用于理解时间尺度 |
| `max_joint_step` | 0.08 | 相邻两步每关节最大变化(rad),等效速度约束 |
| `success_tolerance` | 2e-3 | 末端误差收敛阈值(m) |

**代价权重**(定义求解偏好:末端更准、关节更平滑、姿态不过偏)

| 字段 | 默认 |
|---|---|
| `position_weight` / `terminal_position_weight` | 50 / 500 |
| `orientation_weight` / `terminal_orientation_weight` | 0 / 0 |
| `joint_delta_weight` / `joint_delta_change_weight` | 0.08 / 0.25 |
| `current_posture_weight` / `center_posture_weight` | 0.01 / 0.002 |

**求解后端**

| 字段 | 默认 | 说明 |
|---|---|---|
| `acados_qp_solver` | `FULL_CONDENSING_QPOASES` | QP 后端;推荐 qpOASES。也可 `PARTIAL_CONDENSING_HPIPM`(用它须把 `acados_qp_solver_iter_max` 调小,如 50) |
| `acados_nlp_solver_type` | `SQP_RTI` | 实时迭代 SQP |
| `acados_qp_solver_iter_max` | 500 | QP 内层最大迭代数 |
| `acados_qp_warm_start` | 1 | 0 冷启动 / 1 primal / 2 primal+dual |
| `acados_build_dir` | `/tmp/windylab_acados_mpc_ik` | 代码生成/编译缓存目录 |

## 可选能力:避障约束

默认关闭(`obstacle_avoidance_enabled=False`),此时就是最小逆解。需要时置为 `True`
并由调用方传入避障几何(本库不内置具体机型的几何):

| 字段 | 说明 |
|---|---|
| `obstacle_avoidance_enabled` | 总开关 |
| `obstacle_safety_margin` | 安全余量(m) |
| `obstacle_ll100_urdf_path` | 解析避障几何用的参考 URDF |
| `obstacle_keypoint_frames` | 参与避障的 frame 名序列 |
| `obstacle_front_propeller_joints` | 前方障碍(如桨叶)对应关节 |
| `body_obstacle_enabled` / `body_obstacle_boxes` | 机身盒式障碍 |
| `capsule_points` / `capsule_*_radii` | 胶囊体采样点与半径(最精确的臂体建模) |

避障几何的坐标/半径标定与具体机械臂和安装环境相关,需按实际平台配置。
