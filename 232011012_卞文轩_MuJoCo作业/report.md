# MuJoCo MPC 汽车仪表盘 - 作业报告

## 一、项目概述

### 1.1 作业背景

随着自动化和智能驾驶技术的发展，车辆仪表盘不仅仅是信息显示的界面，更是驾驶交互的重要环节。在 **MuJoCo MPC**仿真环境中，设计一个可视化的汽车仪表盘有助于直观展示车辆的状态，如速度、加速度和转向角等数据，从而更好地理解 MPC 控制下车辆的行为和性能。

本次实验的背景是通过 MuJoCo 模拟汽车运动轨迹，并结合 MPC 控制算法，实现一个**可视化汽车仪表盘**，让用户可以实时观察车辆状态，提高对控制算法效果的理解和分析能力。

### 1.2 实现目标

本次实验的主要目标包括：

1. **车辆状态数据获取**：通过 MuJoCo MPC 仿真获取汽车的实时速度、转速、加速度等关键状态参数。
2. **仪表盘绘制与设计**：设计一个模拟真实汽车仪表盘的界面，包括速度表、转速表和其他辅助信息，指针能够根据实时状态动态变化。
3. **数据可视化**：实现数值与指针的实时同步更新，使实验过程中车辆状态可视化，直观呈现控制效果。
4. **系统整合与调试**：将 MuJoCo 仿真、MPC 控制算法和仪表盘可视化整合为一个可运行实验程序，并保证数据更新与显示流畅、准确。

### 1.3 开发环境

- **操作系统**：Ubuntu 24.03
- **编程语言**：C++17
- **开发工具**：VS Code
- **仿真工具**：MuJoCo 2.3
- **构建工具**：CMake + Ninja
- **控制算法**：MPC（Model Predictive Control）
- **依赖库**：
  - `mjpc`：MuJoCo 控制任务框架
  - `Eigen`：线性代数计算
  - `OpenGL / ImGui`：用于仪表盘绘制和实时渲染

## 二、技术方案

### 2.1 系统架构

- 系统架构图

  ```md
  +-----------------------+
  |      MuJoCo物理引擎    |
  |  - 车辆动力学仿真       |
  |  - 传感器数据生成       |
  +-----------------------+
            |
            v
  +-----------------------+
  |   SimpleCar任务模块    |
  |  - ResidualFn残差计算  |
  |  - TransitionLocked   |
  |  - 仪表盘数据计算       |
  +-----------------------+
            |
            v
  +-----------------------+
  |    OpenGL渲染模块      |
  |  - 仪表盘绘制          |
  |  - 指针、刻度、数字显示  | 
  |  - 转速表和速度表       |
  +-----------------------+
  
  ```

- 模块划分

  1. **仿真控制模块（MPC + MuJoCo）**
     - 负责车辆动力学仿真和控制输入计算
     - 使用 MPC 算法预测未来轨迹并生成控制指令
  2. **数据处理模块**
     - 从 MuJoCo 仿真中获取车辆状态数据（速度、转速、加速度等）
     - 对原始数据进行滤波和转换，使其适合仪表盘显示
  3. **可视化渲染模块（OpenGL / ImGui）**
     - 负责绘制仪表盘界面，包括速度表、转速表和其他状态信息
     - 根据实时状态更新指针位置、刻度和数值显示
  4. **系统集成与通信模块**
     - 将仿真数据传递给可视化模块
     - 保证数据更新与渲染同步

### 2.2 数据流程

- 数据流程图

  ```
  [ MuJoCo ]
         |
         v
  [SimpleCar模块] ---计算速度/转速/油量---> [仪表盘数据]
         |
         v
  [OpenGL渲染模块] <---使用仪表盘数据绘制界面
  ```

- #### 数据结构设计

  1. **车辆状态数据**

     - `qpos[3]`：车辆位置 `(x, y, z)`
     - `qvel[3]`：车辆速度 `(vx, vy, vz)`
     - `qacc[3]`：车辆加速度 `(ax, ay, az)`
     - `ctrl[2]`：控制输入 `(throttle, steering)`

  2. **仪表盘数据结构**

     ```cpp
     struct DashboardData {
         float speed_ratio;   // 当前速度/最大速度
         float rpm_ratio;     // 当前转速/最大转速
         float fuel_percent;  // 剩余油量百分比
         float pos[3];        // 仪表盘位置
     };
     ```

  3. **渲染几何体**

     - 类型：`mjGEOM_CYLINDER`, `mjGEOM_BOX`, `mjGEOM_LABEL`
     - 属性：
       - `pos[3]`：几何体位置
       - `size[3]`：几何体尺寸
       - `mat[9]`：旋转矩阵
       - `rgba[4]`：颜色与透明度
       - `label`：数字或单位显示

### 2.3 渲染方案

- #### 渲染流程

  1. 清屏：`glClear(GL_COLOR_BUFFER_BIT)`
  2. 仪表盘坐标系设置：`glTranslatef` + 仪表盘旋转矩阵
  3. 绘制速度表和转速表：
     - 圆环（外圈）
     - 刻度线和数字
     - 指针旋转
     - 数字速度显示与单位显示
     - 红区警示
  4. 调用`glutSwapBuffers()`完成双缓冲渲染

- #### OpenGL使用

  - **基本绘制函数**
    - `glBegin(GL_LINE_LOOP)` / `glBegin(GL_LINES)`：绘制圆和指针
    - `glVertex2f`：指定顶点
    - `glColor3f`：设置颜色
  - **文字显示**
    - `glRasterPos2f` + `glutBitmapCharacter`
  - **矩阵操作**
    - 手动构造旋转矩阵，实现仪表盘和指针旋转
  - **场景集成（mjvScene）**
    - 利用`mjvGeom`将仪表盘各部件添加到MuJoCo场景中，实现物理仿真与可视化结合

## 三、实现细节

### 3.1 场景创建

- MJCF文件设计

  场景文件为 `task.xml`，提供了基本场景
  汽车模型文件为 `car_model.xml`，包含了车辆底盘 和 目标点

- 场景截图

  ![scene_load](screenshots\scene_load.png)

### 3.2 数据获取

- 关键代码

  ```cpp
  double pos_x = data->qpos[0];
  double pos_y = data->qpos[1];
  double acc_x = data->qacc[0];
  double acc_y = data->qacc[1];
  
  // 速度计算
    double* car_velocity = SensorByName(model, data, "car_velocity");
    if (!car_velocity) return;
  
    double speed_ms = mju_norm3(car_velocity);
    double speed_kmh = speed_ms * 3.6;
  
    double* car_pos = data->xpos + 3 * car_body_id;
  // 转速计算
    float rpm = speed_ms * 600.0f;
    const float max_rpm = 8000.0f;
    if (rpm > max_rpm) rpm = max_rpm;
    float rpm_ratio = rpm / max_rpm;
  ```

- 数据验证

  观察终端数据是否正确

  ![terminall_data](screenshots\terminall_data.png)

### 3.3 仪表盘渲染

#### 3.3.1 速度表

- 实现思路
  1. **仪表盘底盘**：使用 OpenGL 圆环绘制，作为速度表的外框。
  2. **刻度与数字**：通过绘制线段 (`GL_LINES`) 生成刻度线，并使用 `mjGEOM_LABEL` 绘制数字刻度，保证刻度均匀分布。
  3. **速度指针**：根据车辆速度计算百分比 `speed_ratio = speed_kmh / max_speed_kmh`，再转换成指针角度，旋转指针显示实时速度。
  4. **动态更新**：每个渲染帧更新速度指针和数字，实现动态速度显示。
  
- 代码片段

  ```cpp
    //=============================================================================
   //转速表
   //=============================================================================
  
    //底盘
   
    if (scene->ngeom < scene->maxgeom) {
      mjvGeom* geom = scene->geoms + scene->ngeom;
    
      geom->type = mjGEOM_CYLINDER;
      geom->size[0] = geom->size[1] = 0.25f; 
      geom->size[2] = 0.002f;                 
      
      geom->pos[0] = dashboard_pos[0];
      geom->pos[1] = dashboard_pos[1];
      geom->pos[2] = dashboard_pos[2];
      
      for (int j = 0; j < 9; j++) {
        geom->mat[j] = static_cast<float>(dashboard_rot_mat[j]);
      }
      geom->rgba[0] = 0.04f;
      geom->rgba[1] = 0.07f;
      geom->rgba[2] = 0.14f;
      geom->rgba[3] = 0.50f;
      scene->ngeom++;
    }
    
    for (int i = 0; i <= 10; i=i+2) {
      if (scene->ngeom >= scene->maxgeom) break;
  
      float tick_angles = -45.0f +54.0f*i/2;
      float rad_tick_angle = tick_angles * 3.14159f / 180.0f;
      
    // 刻度数字
      if (scene->ngeom < scene->maxgeom) {
        mjvGeom* label_geom = scene->geoms + scene->ngeom;
        label_geom->type = mjGEOM_LABEL;
        label_geom->size[0] = label_geom->size[1] = label_geom->size[2] = 0.05f;
        
        float label_radius = 0.28f;
        float label_y = dashboard_pos[1] - label_radius * cos(rad_tick_angle);
        float label_z = dashboard_pos[2] + label_radius * sin(rad_tick_angle);
        
        label_geom->pos[0] = dashboard_pos[0];
        label_geom->pos[1] = label_y;
        label_geom->pos[2] = label_z;
        
        label_geom->rgba[0] = 0.8f;  
        label_geom->rgba[1] = 0.8f;
        label_geom->rgba[2] = 0.8f;
        label_geom->rgba[3] = 1.0f;
        
        char tick_label[10];
        std::snprintf(tick_label, sizeof(tick_label), "%d", i);
        std::strncpy(label_geom->label, tick_label, sizeof(label_geom->label) - 1);
        label_geom->label[sizeof(label_geom->label) - 1] = '\0';
        scene->ngeom++;
      }
    }
    
    //速度指针
    if (scene->ngeom < scene->maxgeom) {
      mjvGeom* geom = scene->geoms + scene->ngeom;
      geom->type = mjGEOM_BOX;
      geom->size[0] = 0.004f;   
      geom->size[1] = 0.11f;   
      geom->size[2] = 0.003f;   
      
  
      float angle = -45.0f + 270.0f * speed_ratio;
      float rad_angle = angle * 3.1415926f / 180.0f;
  
      // 指针位置
      float pointer_y = dashboard_pos[1] - 0.055f * cos(rad_angle);
      float pointer_z = dashboard_pos[2] + 0.055f * sin(rad_angle);
      
      geom->pos[0] = dashboard_pos[0];
      geom->pos[1] = pointer_y;
      geom->pos[2] = pointer_z;
      
      // 指针旋转
      double pointer_angle = angle - 90.0;  
      double rad_pointer_angle = pointer_angle * 3.14159 / 180.0;
      double cos_p = cos(rad_pointer_angle);
      double sin_p = sin(rad_pointer_angle);
      double pointer_rot_mat[9] = {
        cos_p, -sin_p, 0,
        sin_p,  cos_p, 0,
        0,      0,     1
      };
      
      // 组合旋转
      double temp_mat[9];
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          temp_mat[i*3 + j] = 0;
          for (int k = 0; k < 3; k++) {
            temp_mat[i*3 + j] += dashboard_rot_mat[i*3 + k] * pointer_rot_mat[k*3 + j];
          }
        }
      }
      
      for (int i = 0; i < 9; i++) {
        geom->mat[i] = static_cast<float>(temp_mat[i]);
      }
      
  
      geom->rgba[0] = 1.0f; 
      geom->rgba[1] = 0.0f;
      geom->rgba[2] = 0.0f;
      geom->rgba[3] = 1.0f;
      scene->ngeom++;
    }
    
    // 数字速度显示
    if (scene->ngeom < scene->maxgeom) {
      mjvGeom* geom = scene->geoms + scene->ngeom;
      geom->type = mjGEOM_LABEL;
      geom->size[0] = geom->size[1] = geom->size[2] = 0.08f;
      geom->pos[0] = dashboard_pos[0];
      geom->pos[1] = dashboard_pos[1];
      geom->pos[2] = dashboard_pos[2] + 0.02f; 
      
      geom->rgba[0] = 0.9f; 
      geom->rgba[1] = 0.9f;
      geom->rgba[2] = 0.9f;
      geom->rgba[3] = 1.0f;
      
      char speed_label[50];
      std::snprintf(speed_label, sizeof(speed_label), "%.1f", speed_kmh);
      std::strncpy(geom->label, speed_label, sizeof(geom->label) - 1);
      geom->label[sizeof(geom->label) - 1] = '\0';
      scene->ngeom++;
    }
    
    // km/h
    if (scene->ngeom < scene->maxgeom) {
      mjvGeom* geom = scene->geoms + scene->ngeom;
      geom->type = mjGEOM_LABEL;
      geom->size[0] = geom->size[1] = geom->size[2] = 0.05f;
      geom->pos[0] = dashboard_pos[0];
      geom->pos[1] = dashboard_pos[1];
      geom->pos[2] = dashboard_pos[2] - 0.06f;
      
      geom->rgba[0] = 0.8f;  
      geom->rgba[1] = 0.8f;
      geom->rgba[2] = 0.8f;
      geom->rgba[3] = 1.0f;
      
      std::strncpy(geom->label, "km/h", sizeof(geom->label) - 1);
      geom->label[sizeof(geom->label) - 1] = '\0';
      scene->ngeom++;
    }
  
  
  ```

  

- 效果展示

  ![speed_](screenshots\speed_.png)

#### 3.3.2 转速表

- 实现思路
    1. **RPM计算**：根据车辆线速度计算转速 `rpm = speed_ms * 600`，并限定最大值 `max_rpm`。
    2. **刻度与红区**：绘制转速刻度线和数字。
    3. **转速指针**：根据转速百分比 `rpm_ratio = rpm / max_rpm`，计算指针角度，旋转显示当前转速。
    4. **实时更新**：每个渲染帧更新指针位置和数字，实现动态转速显示。
    
- 代码片段

- 效果展示

    ![rpm_](screenshots\rpm_.png)

## 四、遇到的问题和解决方案

### 问题1: 速度或加速度单位不正确

- **现象**: 打印或仪表盘上显示速度和加速度单位错误。
- **原因**: 使用 MuJoCo 内部原始数据未进行单位换算。
- **解决**: 运用速度转换公式：`speed_kmh = speed_ms * 3.6` 转换为 km/h

### 问题2: 仪表盘数字显示模糊或重叠

- **现象**:  速度/转速数字显示位置不对，出现重叠或偏移。
- **原因**: 数字位置计算只考虑了角度，但未考虑仪表盘旋转矩阵或坐标偏移。
- **解决**: 将数字位置乘以仪表盘旋转矩阵，保证与指针和刻度对齐。调整半径和偏移量，避免数字重叠。

## 五、测试与结果

### 5.1 功能测试

- 测试用例

  - 仪表盘数据是否正确

  - 刻度是否表在正确位置

  - 速度和转速仪表盘角度是否正确

- 测试结果

  - 仪表盘数据正确且能实时显示
  - 刻度标在预期角度
  - 仪表盘可以按照个子数值旋转至对应位置

### 5.2 性能测试

- 帧率测试正常，无明显卡顿。
- 资源占用较小，可以与其他应用共同运行。

### 5.3 效果展示

- 截图

  ![full_](screenshots\full_.png)

- 视频链接

  [点我打开](https://www.bilibili.com/video/BV1NeBDB2Eyc/)

## 六、总结与展望

### 6.1 学习收获

通过本次 MuJoCo MPC 汽车仪表盘实验，深入理解了 MuJoCo 仿真系统的整体架构及其数据流转方式，掌握了从 MJCF 场景建模、物理状态获取到可视化渲染的完整流程。在实验过程中，学习了如何利用 `mjData` 中的状态变量（如位置、速度、加速度等）进行实时数据采集，并将其映射为具有物理意义的可视化仪表盘信息。

此外，本实验还加深了对 OpenGL 基本绘制流程和 MuJoCo 渲染接口（`mjvScene`、`mjvGeom`）的理解，能够通过几何体组合与变换实现速度表和转速表等复杂可视化组件。同时，通过对速度、转速与控制输入之间关系的分析，进一步理解了车辆运动学特性及 MPC 控制结果的直观表现形式。

### 6.2 不足之处

尽管完成了基本功能的实现，但本系统仍存在一定不足。首先，仪表盘数据与真实车辆物理模型之间的映射关系较为简化，例如转速是通过速度线性换算得到，未建立完整的发动机与传动系统模型，因此在物理真实性方面仍有提升空间。其次，当前仪表盘的布局和交互功能较为基础，缺乏用户可配置性和多视角切换支持。

在性能方面，虽然系统在当前场景规模下能够稳定运行，但未对复杂场景等极端条件下的性能进行系统性评估，仍缺少更全面的性能优化分析。

### 6.3 未来改进方向

在后续工作中，可以从以下几个方面对系统进行改进与扩展：

1. **物理模型增强**：引入更真实的车辆动力学模型，将发动机转速、挡位、扭矩等因素纳入计算，使仪表盘数据更加贴近真实车辆行为。
2. **仪表盘功能扩展**：增加加速度表、油量表、报警提示（如超速、红区提示）等信息显示，提高仪表盘的完整性与实用性。
3. **交互与可视化优化**：支持仪表盘样式参数化配置，增强颜色对比度与动画效果，提升整体可读性与用户体验。
4. **性能与测试优化**：引入自动化测试与性能监控机制，对帧率、CPU 资源占用等指标进行长期统计分析，提升系统的工程可靠性。
5. **应用场景拓展**：将当前仪表盘系统扩展到更复杂的 MuJoCo 场景或多智能体任务中，探索其在教学仿真和智能驾驶研究中的应用潜力。