# LIO-SAM_noted
LIO SAM学习，代码注释及原理解析  
LIO SAM紧耦合imu数据且松耦合gnss数据，为当前主流的激光的slam方案，imu预积分等理论调用gtsam库后使代码大大简化，易于阅读
## 1.框架

## 2.论文解读

## 3.代码解析
lio-sam分为4个节点(pipeline为序):

**1.imageProjection节点**  
输入--imu原始数据 +激光里程计
作用--imu预积分  
输出--imu里程计(高频)

**2.imageProjection节点**  
输入--点云原始数据+imu原始数据+imu里程计  
作用--重构点云数据结构并去畸变
输出--去畸变的点云数据

**3.featureExtraction 节点**  
输入--去畸变后的点云数据   
作用--提取特征点(角点和平面点)  
输出--发布所提取到的点云数据  


**4.mapOptmization节点**  
输入--去畸变的点云数据+gps数据  
作用-- 激光匹配、激光雷达里程计、图优化  
输出-- 激光里程计(低频)




## 3.数据集及实车测试


## 4. 个人思考
自定义的数据类型cloud_info数据量大,在ros1框架下使用节点间通讯效率过低 ,可采用ros1 nodelet、ros2、多线程;  
对比aloam,未充分利用所提取的平面点;  
工程化不足,例如ros通讯和逻辑业务未做隔离、代码面向对象的特点少;




## 5.参考
lio sam paper  
github.com/smilefacehh/LIO-SAM-DetailedNote
