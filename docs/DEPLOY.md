# 部署说明

## 系统与依赖概览

- Python 3.8+,Linux(acados 代码生成需要 C 编译器 gcc/cc)。
- 依赖分三块:
  1. **numpy / casadi** —— pip 直接装。
  2. **pinocchio** —— 机器人运动学库(从 URDF 建模、正运动学、雅可比)。
  3. **acados** —— 非线性优化求解框架,含 C 库 `libacados`、Python 接口 `acados_template`、
     以及 qpOASES 后端。这是本库求解能力的核心。

## 1. numpy / casadi

```bash
pip install -r requirements.txt
```

## 2. pinocchio

三种方式任选其一:

```bash
pip install pin                                  # pip
# 或 conda install pinocchio -c conda-forge      # conda
# 或 sudo apt install ros-humble-pinocchio       # 随 ROS 2 Humble 安装
```

验证:

```bash
python3 -c "import pinocchio; print('pinocchio', pinocchio.__version__)"
```

## 3. acados(含 qpOASES 后端)

从源码编译,**编译时启用 qpOASES**:

```bash
git clone https://github.com/acados/acados.git ~/code/acados
cd ~/code/acados
git submodule update --recursive --init
mkdir -p build && cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j4
pip install -e ~/code/acados/interfaces/acados_template
```

> **ARM64 / Jetson**:blasfeo 需指定 CPU target,例如
> `cmake -DACADOS_WITH_QPOASES=ON -DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57 ..`;
> 报指令不支持则退到 `-DBLASFEO_TARGET=GENERIC`。

## 4. 运行环境变量(每次运行前)

```bash
export ACADOS_SOURCE_DIR=$HOME/code/acados
export LD_LIBRARY_PATH=$HOME/code/acados/lib:$LD_LIBRARY_PATH
```

建议写进 `~/.bashrc` 或专用 env 脚本,避免每次手动设。

## 5. 验证安装

```bash
python3 examples/01_minimal_solve.py
```

首次运行会触发 acados 代码生成 + 编译(数十秒~几分钟),看到 `求解成功=True` 即部署成功。
之后复用缓存的 `.so`,单步求解毫秒级。

## 常见问题

| 现象 | 排查 |
|---|---|
| `import pinocchio` 失败 | 确认装了 pinocchio 的 Python 绑定(见第 2 步) |
| 找不到 `libacados.so` / 运行报链接错误 | 确认 `ACADOS_SOURCE_DIR` 与 `LD_LIBRARY_PATH` 已导出 |
| 首次求解很慢 | 正常,acados 在做 codegen + 编译;缓存目录由 `config.acados_build_dir` 决定(默认 `/tmp/windylab_acados_mpc_ik_*`) |
| 改了 URDF/时域/后端后行为异常 | 这些是结构量,会触发重新编译;清掉旧 `acados_build_dir` 再跑 |
