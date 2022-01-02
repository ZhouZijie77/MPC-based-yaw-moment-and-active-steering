# 直接横摆力矩和主动转向模型预测控制（MPC）

文件列表

- braking_cal.m：制动力分配逻辑模块
- c5new.mdl：simulink模型文件
- chapter5_2_2.m：控制器S-function主要代码
- lisan.m：根据车辆动力学模型计算雅格比矩阵并使用向前欧拉法离散化
- processdata.m：处理得到的数据，画图并计算跟踪误差
- syx.m：计算输入的参考双移线轨迹
- test2.m：验证双移线轨迹中横摆角加速度是否正确

主要参考文献：

[1] Falcone P, Eric Tseng H, Borrelli F, et al. MPC-based yaw and lateral stabilisation via active front steering and braking[J]. Vehicle System Dynamics, 2008, 46(S1): 611-628.

[2] 龚建伟. 无人驾驶车辆模型预测控制[M]. 北京：北京理工大学出版社, 2020. 