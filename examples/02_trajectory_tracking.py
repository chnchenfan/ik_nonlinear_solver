#!/usr/bin/env python3
"""轨迹跟踪示例:在控制循环里逐点求解,让末端跟踪一条圆轨迹。

演示控制循环用法:每周期 solve(target, q_init=当前关节角) -> (q, ok),把 q 作为
下一步命令(实际部署中发给机械臂/仿真,并作下次热启动)。这里离线迭代,用于快速看跟踪精度。
"""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nonlinear_ik import NonlinearPinocchioIK, NonlinearIKConfig

HERE = Path(__file__).resolve().parents[1]
URDF = str(HERE / "models" / "ll100_lh420_nmpc.urdf")
EE_FRAME = "F_Link"


def main() -> int:
    config = NonlinearIKConfig(
        horizon_steps=6,
        control_dt=0.02,
        position_weight=200.0,
        terminal_position_weight=1200.0,
    )
    solver = NonlinearPinocchioIK(urdf_path=URDF, ee_frame=EE_FRAME, config=config)

    # 以当前末端为参考,在 y-z 平面画一个半径 5cm 的圆。
    q = solver.q_center.copy()
    center, _ = solver.forward(q)
    radius = 0.05
    n_steps = 200

    errors = []
    for i in range(n_steps):
        theta = 2.0 * np.pi * i / n_steps
        target = center + np.array([0.0, radius * np.cos(theta), radius * np.sin(theta)])
        q, ok = solver.solve(target, q_init=q)   # q 既是下一步命令,也作下次热启动
        p, _ = solver.forward(q)
        errors.append(np.linalg.norm(p - target))

    errors = np.array(errors)
    steady = errors[n_steps // 4:]              # 跳过前 1/4 启动暂态,看稳态跟踪
    print(f"跟踪 {n_steps} 步、半径 {radius*100:.0f}cm 圆")
    print(f"稳态末端误差:均值 {steady.mean()*1e3:.2f} mm,最大 {steady.max()*1e3:.2f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
