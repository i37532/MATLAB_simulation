# 四旋翼前向模型：最小运行包

本目录保留运行 `drone_model.slx` 所需的文件，供学习四旋翼前向动力学和 Simulink 仿真使用。

## 文件

- `drone_model.slx`：四旋翼模型，包含姿态、位置、电机和动力学模块。
- `InitParameters.m`：模型的 `InitFcn` 回调会调用此脚本，设置电机、机体、推力和惯量参数。
- `icon/FlightGear.png`：模型内 FlightGear 遮罩图引用的图像。FlightGear 模块在此模型中已注释，不参与仿真计算；保留图像可保证模型打开时图标引用有效。
- `run_drone_model.m`：将 MATLAB 当前目录临时切换到本目录、打开模型并启动仿真。

`.slxc`、`slprj/` 是 Simulink 生成的缓存，`.slx.r2017b` 是转换前的旧版备份，不是运行依赖，未放入此最小包。

## 运行

需要安装 MATLAB 和 Simulink。在 MATLAB 命令窗口运行：

```matlab
cd('本目录的完整路径')
run_drone_model
```

例如 Windows：

```matlab
cd('C:\Users\13501\Desktop\多旋翼无人机建模与仿真\matlab_simulation\forward_simulation')
run_drone_model
```

也可以手动打开 `drone_model.slx` 并点击 **运行**。手动运行前，先将 MATLAB 当前文件夹切换到本目录，使 `InitParameters.m` 和 `icon/` 的相对路径可用。

此目录是最小前向模型，不包含原工作副本中其他控制方案的 `.slx` 文件，也不包含仿真生成缓存。
