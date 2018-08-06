# VINS-Stereo
   
## image_processor:
- 使用了msckf_vio前端(https://github.com/KumarRobotics/msckf_vio)
- 加入了outlier剔除。
- publish的特征点用unified camera model做了归一化。

## vins_estimator
- 使用了VINS-Mono的后端(https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- 为了适应image_processor，这里调整了一下feature解析格式。


## vins_estimator_stereo
- 在vins_estimator基础上加入了双目约束。

## vins_estimator_multi
- 参考vins_so改了一个支持双目甚至多目的vins,(https://github.com/gaowenliang/vins_so)

## others
- 特征点分布均匀时效果更好。
- 使用bmi160(osr4)的效果比mpu9250(dlfp2.9ms)效果要好，bmi160的数据更稳定、噪声更小。
- 双目比单目效果略有提升。
- 加入imu插值后效果反而变差了，可能是vins-mono采用的是mid-point方法所致，但是港科大master分支也加了插值，这个问题待分析。
