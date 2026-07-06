# 非线性末端逆解求解器(Pinocchio + acados/qpOASES)

一个独立、可复用的机械臂末端位置逆解(IK)求解器。用 Pinocchio 从 URDF 建模,用
acados + qpOASES 做**非线性优化**求解关节角——适合大位移、强非线性、带约束的情形。

对外接口与常见的逐点 IK 求解器一致,可最小改动替换:

```python
from nonlinear_ik import NonlinearPinocchioIK

solver = NonlinearPinocchioIK(urdf_path="model.urdf", ee_frame="F_Link")
q, ok = solver.solve(target_position)   # 目标末端位置 (3,), base 系, m
pos, rot = solver.forward(q)            # 正运动学验证
```

## 特性

- **接口简洁**:主用法就是 `solve(target) -> (q, ok)` 和 `forward(q) -> (pos, rot)`。
- **非线性求解**:acados SQP_RTI + FULL_CONDENSING_QPOASES,单步毫秒级。
- **能力可开关**:默认是最小逆解;需要时可通过配置启用多步序列、末端姿态、避障约束
  等高级能力(见 [docs/API.md](docs/API.md)),不用时完全不影响简洁接口。
- **不绑定机型**:URDF、末端 frame、避障几何都由调用方传入;自带一个示例模型便于开箱试跑。

## 目录结构

```
.
├── nonlinear_ik/            # 求解器库(纯 Python,不依赖 ROS)
│   ├── __init__.py     # 导出 NonlinearPinocchioIK / NonlinearIKConfig
│   └── solver.py       # 求解器与 acados 后端
├── models/             # 示例模型(ll100_lh420_nmpc.urdf,纯运动学,无 mesh)
├── examples/           # 可运行示例
│   ├── 01_minimal_solve.py
│   └── 02_trajectory_tracking.py
├── docs/
│   ├── DEPLOY.md       # 依赖安装与环境配置
│   └── API.md          # 接口详解与配置项
├── requirements.txt
└── pyproject.toml
```

## 快速开始

1. 装依赖(pinocchio / casadi / acados),见 [docs/DEPLOY.md](docs/DEPLOY.md)。
2. 配好 acados 环境变量后跑最小示例:

```bash
export ACADOS_SOURCE_DIR=$HOME/code/acados
export LD_LIBRARY_PATH=$HOME/code/acados/lib:$LD_LIBRARY_PATH
python3 examples/01_minimal_solve.py
```

首次求解会触发 acados 代码生成 + 编译(数十秒~几分钟),之后复用缓存的 `.so`,单步毫秒级。

## 接口速览

| 方法 | 说明 |
|---|---|
| `solve(target_position, target_rotation=None, q_init=None) -> (q, ok)` | 求关节命令,主接口 |
| `forward(q) -> (position, rotation)` | 正运动学 |
| `step(target, q_current, ...) -> (q, ok, info)` | 高级:返回第一步命令 + 完整诊断 |
| `solve_horizon(target, ...) -> (q_N, ok, info)` | 高级:返回多步序列末端一步 + 诊断 |
| `last_info` | 属性:最近一次 `solve()` 的诊断(求解耗时/迭代/避障 margin 等) |

详见 [docs/API.md](docs/API.md)。
