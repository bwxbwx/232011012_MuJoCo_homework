// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/tasks/simple_car/simple_car.h"

#include <cmath>
#include <cstdio>
#include <string>

#include <absl/random/random.h>
#include <mujoco/mujoco.h>

#include "mjpc/task.h"
#include "mjpc/utilities.h"

#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>

namespace mjpc {

//==============================================================================
// Info
//==============================================================================

std::string SimpleCar::XmlPath() const {
  return GetModelPath("simple_car/task.xml");
}

std::string SimpleCar::Name() const {
  return "SimpleCar";
}
//==============================================================================
// Residual
//==============================================================================

void SimpleCar::ResidualFn::Residual(const mjModel* model,
                                     const mjData* data,
                                     double* residual) const {
  residual[0] = data->qpos[0] - data->mocap_pos[0];
  residual[1] = data->qpos[1] - data->mocap_pos[1];
  residual[2] = data->ctrl[0];
  residual[3] = data->ctrl[1];
}

//==============================================================================
// Transition
//==============================================================================

void SimpleCar::TransitionLocked(mjModel* model, mjData* data) {
  double car_pos[2] = {data->qpos[0], data->qpos[1]};
  double goal_pos[2] = {data->mocap_pos[0], data->mocap_pos[1]};

  double delta[2];
  mju_sub(delta, goal_pos, car_pos, 2);

  if (mju_norm(delta, 2) < 0.2) {
    absl::BitGen gen;
    data->mocap_pos[0] = absl::Uniform<double>(gen, -2.0, 2.0);
    data->mocap_pos[1] = absl::Uniform<double>(gen, -2.0, 2.0);
    data->mocap_pos[2] = 0.01;
  }
}

//==============================================================================
//绘制辅助函数
//==============================================================================

// 画圆
void SimpleCar::drawCircle(float radius, int segments) {
  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < segments; ++i) {
    float angle = 2.0f * M_PI * i / segments;
    glVertex2f(radius * cos(angle), radius * sin(angle));
  }
  glEnd();
}

// 画线
void SimpleCar::drawTicks(float radius, int tickCount) {
  float angleStep = 180.0f / tickCount;
  for (int i = 0; i < tickCount; ++i) {
    float angle = i * angleStep * M_PI / 180.0f;
    float tickLength = 0.02f;

    glBegin(GL_LINES);
    glVertex2f(radius * cos(angle), radius * sin(angle));
    glVertex2f((radius - tickLength) * cos(angle),
               (radius - tickLength) * sin(angle));
    glEnd();
  }
}

// 画指针
void SimpleCar::drawPointer(float angle) {
  glBegin(GL_LINES);
  glVertex2f(0.0f, 0.0f);
  glVertex2f(0.8f * cos(angle), 0.8f * sin(angle));
  glEnd();
}

// 画数字
void SimpleCar::drawNumber(float radius, int number, float angle) {
  char buffer[10];
  std::snprintf(buffer, sizeof(buffer), "%d", number);
  glRasterPos2f(radius * cos(angle), radius * sin(angle));
  for (int i = 0; buffer[i]; ++i) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, buffer[i]);
  }
}

//==============================================================================
// 仪表盘绘制
//==============================================================================

void SimpleCar::drawDashboard(float* dashboard_pos, float speed_ratio) {
  glClear(GL_COLOR_BUFFER_BIT);

  glPushMatrix();
  glTranslatef(dashboard_pos[0], dashboard_pos[1], dashboard_pos[2]);

  // 外圈
  glColor3f(0.1f, 0.1f, 0.1f);
  drawCircle(0.6f, 100);

  // 刻度
  glColor3f(1.0f, 1.0f, 1.0f);
  drawTicks(0.5f, 10);

  // 指针
  float pointerAngle =
      (90.0f - 180.0f * speed_ratio) * M_PI / 180.0f;
  glColor3f(1.0f, 0.0f, 0.0f);
  drawPointer(pointerAngle);

  // 数字刻度
  for (int i = 0; i <= 10; ++i) {
    float angle = (90.0f - 18.0f * i) * M_PI / 180.0f;
    drawNumber(0.45f, i, angle);
  }

  // 单位
  glColor3f(0.9f, 0.9f, 0.9f);
  glRasterPos2f(0.0f, -0.7f);
  const char* unit = "km/h";
  for (int i = 0; unit[i]; ++i) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, unit[i]);
  }
  float speed = 0.0f;
  // 当前速度
  char speedStr[32];
  std::snprintf(speedStr, sizeof(speedStr), "%.1f", speed);
  glRasterPos2f(-0.1f, 0.0f);
  for (int i = 0; speedStr[i]; ++i) {
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, speedStr[i]);
  }

  glPopMatrix();
  glutSwapBuffers();
}


void SimpleCar::ModifyScene(const mjModel* model, const mjData* data,mjvScene* scene) const {
  //==============================================================================
  // 状态与油耗
  //==============================================================================
  static double fuel_capacity = 100.0;   
  static double fuel_used = 0.0;   
  double dt = model->opt.timestep;
  double throttle = data->ctrl[0];
  fuel_used += 0.2 * std::abs(throttle) * dt;
  fuel_used = std::min(fuel_used, fuel_capacity);
  double fuel_percent =(fuel_capacity - fuel_used) / fuel_capacity * 100.0;
  //==============================================================================
  //数据获取 & 显示
  //==============================================================================
  double pos_x = data->qpos[0];
  double pos_y = data->qpos[1];
  //double vel_x = data->qvel[0];
  //double vel_y = data->qvel[1];
  double acc_x = data->qacc[0];
  double acc_y = data->qacc[1];

  // 获取汽车ID
  int car_body_id = mj_name2id(model, mjOBJ_BODY, "car");
  if (car_body_id < 0) return; 
  //==============================================================================
  // 速度计算
  //==============================================================================
  
  double* car_velocity = SensorByName(model, data, "car_velocity");
  if (!car_velocity) return;

  double speed_ms = mju_norm3(car_velocity);
  double speed_kmh = speed_ms * 3.6;

  double* car_pos = data->xpos + 3 * car_body_id;
  //==============================================================================
  // 转速计算
  //==============================================================================
  float rpm = speed_ms * 600.0f;
  const float max_rpm = 8000.0f;
  if (rpm > max_rpm) rpm = max_rpm;
  float rpm_ratio = rpm / max_rpm;

  // 打印数据
  printf( "\r坐标(%.2f, %.2f) | 速度(%.2f km/h)|转速(%.0f)| 加速度(%.2f m/s², %.2f m/s²) 剩余油百分比：%3.0f%% ,",pos_x, pos_y,speed_kmh,rpm,acc_x, acc_y,fuel_percent);

  //==============================================================================
  // 仪表盘位置
  //==============================================================================
  float dashboard_pos[3] = {
    static_cast<float>(car_pos[0]),         // 左右
    static_cast<float>(car_pos[1] - 0.30f), // 前后
    static_cast<float>(car_pos[2] + 0.5f)   // 上下
  };
  
  // 最大速度
  const float max_speed_kmh = 10.0f;
  
  // 速度百分比（0-1）
  float speed_ratio = static_cast<float>(speed_kmh) / max_speed_kmh;
  if (speed_ratio > 1.0f) speed_ratio = 1.0f;
  //==============================================================================
  // 仪表盘旋转
  //==============================================================================
  double angle_x = 90.0 * 3.14159 / 180.0;
  double cos_x = cos(angle_x);
  double sin_x = sin(angle_x);
  double mat_x[9] = {
    1, 0,      0,
    0, cos_x, -sin_x,
    0, sin_x,  cos_x
  };
  
  double angle_z = -90.0 * 3.14159 / 180.0; 
  double cos_z = cos(angle_z);
  double sin_z = sin(angle_z);
  double mat_z[9] = {
    cos_z, -sin_z, 0,
    sin_z,  cos_z, 0,
    0,      0,     1
  };
  

  double dashboard_rot_mat[9];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      dashboard_rot_mat[i*3 + j] = 0;
      for (int k = 0; k < 3; k++) {
        dashboard_rot_mat[i*3 + j] += mat_z[i*3 + k] * mat_x[k*3 + j];
      }
    }
  }
  //==============================================================================
  //转速表
  //==============================================================================

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
  
  // km/h单位
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



  //==============================================================================
  //转速表
  //==============================================================================
  //计算转速

  //转速表位置
  float rpm_pos[3] = {
    dashboard_pos[0],
    dashboard_pos[1] + 0.60f,
    dashboard_pos[2] 
  };

  //底盘
  if (scene->ngeom < scene->maxgeom) {
    mjvGeom* geom = scene->geoms + scene->ngeom;
    geom->type = mjGEOM_CYLINDER;
    geom->size[0] = geom->size[1] = 0.25f;
    geom->size[2] = 0.002f;

    geom->pos[0] = rpm_pos[0];
    geom->pos[1] = rpm_pos[1];
    geom->pos[2] = rpm_pos[2];

    for (int j = 0; j < 9; j++)
      geom->mat[j] = static_cast<float>(dashboard_rot_mat[j]);

    geom->rgba[0] = 0.64f;
    geom->rgba[1] = 0.57f;
    geom->rgba[2] = 0.34f;
    geom->rgba[3] = 0.45f;
    scene->ngeom++;
  }

  //转速刻度
  for (int i = 0; i <= 8; i++) {
    if (scene->ngeom >= scene->maxgeom) break;

    float angle = -45.0f + 270.0f * (i / 8.0f);
    float rad = angle * 3.14159f / 180.0f;

    mjvGeom* geom = scene->geoms + scene->ngeom;
    geom->type = mjGEOM_LABEL;
    geom->size[0] = geom->size[1] = geom->size[2] = 0.05f;

    geom->pos[0] = rpm_pos[0];
    geom->pos[1] = rpm_pos[1] - 0.28f * cos(rad);
    geom->pos[2] = rpm_pos[2] + 0.28f * sin(rad);

    geom->rgba[0] = (i >= 6) ? 1.0f : 0.8f;  
    geom->rgba[1] = (i >= 6) ? 0.2f : 0.8f;
    geom->rgba[2] = (i >= 6) ? 0.2f : 0.8f;
    geom->rgba[3] = 1.0f;

    char label[8];
    std::snprintf(label, sizeof(label), "%d", i);
    std::strncpy(geom->label, label, sizeof(geom->label)-1);

    scene->ngeom++;
  }

  //指针
  if (scene->ngeom < scene->maxgeom) {
    mjvGeom* geom = scene->geoms + scene->ngeom;
    geom->type = mjGEOM_BOX;
    geom->size[0] = 0.004f;
    geom->size[1] = 0.11f;
    geom->size[2] = 0.003f;

    float angle = -45.0f + 270.0f * rpm_ratio;
    float rad = angle * 3.14159f / 180.0f;

    geom->pos[0] = rpm_pos[0];
    geom->pos[1] = rpm_pos[1] - 0.055f * cos(rad);
    geom->pos[2] = rpm_pos[2] + 0.055f * sin(rad);
    geom->rgba[0] = 0.2f;
    geom->rgba[1] = 1.0f;
    geom->rgba[2] = 0.2f;
    geom->rgba[3] = 1.0f;

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
    geom->rgba[1] = 0.2f;
    geom->rgba[2] = 0.2f;
    geom->rgba[3] = 1.0f;

    scene->ngeom++;
  }

  //数字显示
  if (scene->ngeom < scene->maxgeom) {
    mjvGeom* geom = scene->geoms + scene->ngeom;
    geom->type = mjGEOM_LABEL;
    geom->size[0] = geom->size[1] = geom->size[2] = 0.06f;

    geom->pos[0] = rpm_pos[0];
    geom->pos[1] = rpm_pos[1];
    geom->pos[2] = rpm_pos[2] - 0.05f;

    geom->rgba[0] = 0.9f;
    geom->rgba[1] = 0.9f;
    geom->rgba[2] = 0.9f;
    geom->rgba[3] = 1.0f;

    char rpm_text[16];
    std::snprintf(rpm_text, sizeof(rpm_text), "%.0f", rpm);
    std::strncpy(geom->label, rpm_text, sizeof(geom->label)-1);

    scene->ngeom++;
  }

  //RPM 单位
  if (scene->ngeom < scene->maxgeom) {
    mjvGeom* geom = scene->geoms + scene->ngeom;
    geom->type = mjGEOM_LABEL;
    geom->size[0] = geom->size[1] = geom->size[2] = 0.04f;

    geom->pos[0] = rpm_pos[0];
    geom->pos[1] = rpm_pos[1];
    geom->pos[2] = rpm_pos[2] - 0.10f;

    geom->rgba[0] = 0.8f;
    geom->rgba[1] = 0.8f;
    geom->rgba[2] = 0.8f;
    geom->rgba[3] = 1.0f;

    std::strncpy(geom->label, "x1000 rpm", sizeof(geom->label)-1);
    scene->ngeom++;
  }
}

}  // namespace mjpc