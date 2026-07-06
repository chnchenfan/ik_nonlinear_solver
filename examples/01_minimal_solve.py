#!/usr/bin/env python3
"""最小示例:构造求解器,给一个可达目标点,解出关节命令。

接口与 base 版 PinocchioIK 一致:solve(target) -> (q, ok)、forward(q) -> (pos, rot)。
运行前需装好依赖(见 docs/DEPLOY.md):pinocchio、casadi、acados(含编译库)。
首次求解会触发 acados 代码生成 + 编译(数十秒~几分钟),之后复用缓存。
"""

from pathlib import Path
import sys

import numpy as np

# 未 pip 安装时,直接从源码目录导入本包。
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nonlinear_ik import NonlinearPinocchioIK, NonlinearIKConfig

HERE = Path(__file__).resolve().parents[1]
URDF = str(HERE / "models" / "ll100_lh420_nmpc.urdf")
EE_FRAME = "F_Link"  # 该 URDF 的末端 frame


def main() -> int:
    config = NonlinearIKConfig(
        horizon_steps=6,
        control_dt=0.02,          # 50Hz
        position_weight=200.0,
        terminal_position_weight=1200.0,
    )
    solver = NonlinearPinocchioIK(urdf_path=URDF, ee_frame=EE_FRAME, config=config)
    print(f"模型自由度 nq={solver.nq}, 末端 frame={EE_FRAME}")

    # 从关节中位出发,取当前末端位置,设一个小位移作为目标。
    q0 = solver.q_center.copy()
    p0, _R0 = solver.forward(q0)
    target = p0 + np.array([0.03, 0.0, -0.03])
    print(f"当前末端 = {np.round(p0, 4)}, 目标 = {np.round(target, 4)}")

    # 接口与 base 一致:solve(target, q_init=当前关节角) -> (q, ok)。
    q, ok = solver.solve(target, q_init=q0)
    p, _ = solver.forward(q)
    info = solver.last_info  # 求解耗时/迭代/避障 margin 等详细诊断

    # ok 反映内部预测序列末端是否达标;下面的"到目标距离"是本步命令 q(受 max_joint_step
    # 单步步长限制)到目标的距离。单次冷启动通常一步走不到位,需多步滚动逼近(见 02 示例的稳态精度)。
    print(f"求解成功={ok}, 求解耗时={info.get('solve_time', float('nan'))*1e3:.2f} ms, 迭代={info.get('nit')}")
    print(f"关节命令 q = {np.round(q, 4)}")
    print(f"命令末端 = {np.round(p, 4)}, 到目标距离 = {np.linalg.norm(p - target)*1e3:.1f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
