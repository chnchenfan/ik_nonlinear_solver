"""基于 Pinocchio 的末端位置逆解求解器,面向非线性情况(acados + qpOASES)。

接口:
- solve(target_position, target_rotation=None, q_init=None) -> (q, ok)
- forward(q) -> (position, rotation)

核心导出:
- NonlinearPinocchioIK: 求解器类,给定目标末端位置与当前关节角求关节命令。
- NonlinearIKConfig:  求解器配置(权重、限位、可选避障几何等)。
"""

from .solver import MpcPinocchioIK as NonlinearPinocchioIK
from .solver import MpcIkConfig as NonlinearIKConfig

__all__ = ["NonlinearPinocchioIK", "NonlinearIKConfig"]
__version__ = "0.1.0"
