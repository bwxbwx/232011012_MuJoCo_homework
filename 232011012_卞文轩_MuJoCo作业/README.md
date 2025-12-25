# MuJoCo MPC 汽车仪表盘项目

## 项目信息
- **学号**: 232011012
- **姓名**: 卞文轩
- **班级**: 计科2301班
- **完成日期**: 2025年12月25日

## 项目概述
1. **车辆状态数据获取**：通过 MuJoCo MPC 仿真获取汽车的实时速度、转速、加速度等关键状态参数。
2. **仪表盘绘制与设计**：设计一个模拟真实汽车仪表盘的界面，包括速度表、转速表和其他辅助信息，指针能够根据实时状态动态变化。
3. **数据可视化**：实现数值与指针的实时同步更新，使实验过程中车辆状态可视化，直观呈现控制效果。
4. **系统整合与调试**：将 MuJoCo 仿真、MPC 控制算法和仪表盘可视化整合为一个可运行实验程序

## 环境要求
- 操作系统: Ubuntu 24.03
- 编译器: gcc 11.3.0
- CMake: 3.22.1
- 编译和运行

### 编译步骤
```bash
cd mujoco_mpc
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE:STRING=Release -G Ninja -DCMAKE_C_COMPILER:STRING=clang-16 -DCMAKE_CXX_COMPILER:STRING=clang++-16 -DMJPC_BUILD_GRPC_SERVICE:BOOL=ON
cmake --build . --config=Release
```



### 运行
```bash
./bin/mjpc --task simplecar
```



## 功能说明

### 已实现功能
- [x] 速度表
- [x] 转速表
- [x] 数字显示（油量、温度）

