# LQR Attitude Control of a Rigid Spacecraft (Quaternion-Based)
# 基于四元数的刚体航天器 LQR 姿态控制

---

## Description | 项目简介
This project implements an LQR-based attitude control scheme for a rigid spacecraft using quaternion representation. The controller is designed from a linearized attitude error model and applied to the full nonlinear rotational dynamics.

本项目基于四元数姿态表示，实现了刚体航天器的 LQR 姿态控制。控制器由线性化误差模型设计，并作用于完整的非线性姿态动力学系统。

---

## Model | 模型说明
- Rigid-body rotational dynamics  
- Quaternion kinematics (singularity-free)  

- 刚体姿态动力学模型  
- 四元数姿态运动学（无奇异性）

---

## Controller | 控制器
- Linear Quadratic Regulator (LQR)
- State: quaternion vector error and angular velocity
- Control input: body-frame torque

- 线性二次型调节器（LQR）  
- 状态量：四元数向量误差与角速度  
- 控制输入：体坐标系力矩  

---

## Simulation Setup | 仿真设置
- Inertia: diag(84.7089, 84.7089, 169.4178) kg·m²  
- Initial attitude: nonzero quaternion error  
- Target attitude: identity quaternion  
- Time step: 0.01 s  
- Duration: 120 s  

- 转动惯量：diag(84.7089, 84.7089, 169.4178) kg·m²  
- 初始姿态：非零四元数误差  
- 目标姿态：单位四元数  
- 步长：0.01 s  
- 仿真时长：120 s  

---

## Outputs | 仿真结果
- Quaternion response  
- Angular velocity response  
- Control torque  
- Quaternion error components  
- Rotation angle error  

- 姿态四元数响应  
- 角速度响应  
- 控制力矩  
- 四元数误差分量  
- 姿态旋转角误差  

---

## Notes | 说明
- LQR is designed from a linearized model (local stability)
- Actuator saturation and disturbances are not considered

- LQR 基于线性化模型设计（局部稳定性）
- 未考虑执行器饱和与外部扰动

---

## Requirements | 运行环境
- MATLAB (Control System Toolbox)

- MATLAB（需 Control System Toolbox）
