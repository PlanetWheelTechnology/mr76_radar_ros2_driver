> 撰写人：胡明栋        版本号及修订时间:    v1.0  20250618
>

# 1.系统设计
+ MR76 毫米波雷达 ROS2 驱动功能包的系统设计思路如下图示。
+ ROS2 节点通过 Linux USB 串口驱动应用接口层拿到 USB 转 CAN 模块转发的 MR76 毫米波的 CAN 报文，通过数据处理后转换为相应的 ROS2 话题类型的消息数据，以供 ROS2 应用层其他节点订阅和请求

![画板](https://cdn.nlark.com/yuque/0/2025/jpeg/2902589/1750239824703-34213431-dcd1-4baf-a469-c99224ea3df3.jpeg)

# 2.外设简介
## 2.1. MR76 毫米波雷达介绍
<img src="https://cdn.nlark.com/yuque/0/2025/png/2902589/1749528129687-ac2ae4c3-5441-4614-8701-e43b256fc835.png" style="zoom: 50%;" />

<img src="https://cdn.nlark.com/yuque/0/2025/png/2902589/1749527516114-b6121042-5326-4a8e-a395-c2b65c8e66ba.png" style="zoom: 33%;" />

<img src="https://cdn.nlark.com/yuque/0/2025/png/2902589/1749527538926-babed1f3-f7db-490c-9694-6325e3be4c2b.png" style="zoom:50%;" />

<img src="https://cdn.nlark.com/yuque/0/2025/png/2902589/1749527649645-000153b4-481c-4edb-a19f-6501bcf904f2.png" style="zoom:50%;" />

## 2.2. USB 转 CAN 模块介绍
1. 模块型号： 泥人电子的 USB-CAN-V3

<img src="https://cdn.nlark.com/yuque/0/2025/png/2902589/1749553718250-3a3a6dd9-ec63-4aaf-941e-e3018497aaa6.png" style="zoom:50%;" />

https://item.taobao.com/item.htm?id=748966111172&pisk=gdi__BgIi1f1A4EAhPvFVcsaM8Ebcp-yDtwxExINHlETliHZN-S4uF2fcWMJI58DSjgIMX4a6Sz4c-GztAS2IAJXcxH86oua_-hEGXbZ_mPqTnMoNPSZHmWi-bkRbc8g0iZgmodya3-rQAq0DphUdufGp8ey0GUOWlqda56wy3-rIv6aBLkp4myxnnwNkihYWyCLUWeOkrUvdkeaCoITkGQd9WVLDRFAWkFLn-IABShA9JegCZQYWPELv8e4DShYDpMLt-ZYMATlF-T_sA95qZyUYTpKQWsADDwpi7HsXJ2EA2UTwPVCDvoQCPN-BcxJf0wKYDaz8tRQvY0E6JZWf_2sJYZThmvRlSajjkwj-C_ZNf2xpyGw3Z2Ilxnuju6vXvZ_1ze_7n9jwvMxzyiee3q_X534jxWW_vis_AmI3tQLfluQPcZJq1Fq8YibhmADAbMxEqUIc1syQgP7UIbfdzjbd7JBdZbD1Hi-vxql9fzTKRaedp1toPe3d7JBdZb0WJ2_Tp9C6qf..&spm=tbpc.boughtlist.suborder_itemtitle.1.e8d42e8dRVWdFQ&skuId=5161000476969

2. 模块使用手册：[CAN转换器系列使用手册V2.5.pdf](https://www.yuque.com/attachments/yuque/0/2025/pdf/2902589/1750242691613-b47b8e75-002e-4e98-9f2b-8421275f346e.pdf)
3. 模块配置软件：[CAN配置软件V2.3.9.zip](https://www.yuque.com/attachments/yuque/0/2025/zip/2902589/1750242691729-7a691516-f47d-4e2b-923f-73a3f788ab9c.zip)
4. 模块配置，如下所示：

<img src="https://cdn.nlark.com/yuque/0/2025/png/2902589/1749553934595-cba89d84-6917-4357-9cd1-a7babd2de589.png" style="zoom: 67%;" />

5. 串口端数据应用协议：

<img src="https://cdn.nlark.com/yuque/0/2025/png/2902589/1749554069439-3ef0c2ce-0276-4006-84b2-b35a1656e5b9.png" style="zoom:67%;" />

# 3. MR76 技术资料汇总
[MR76 77GHz毫米波雷达白皮书V1.1.pdf](https://www.yuque.com/attachments/yuque/0/2025/pdf/2902589/1750242691903-9d21d442-27f7-42ad-b917-cd19d84241c1.pdf)

[MR76毫米波雷达通信协议V1.2_20200307.pdf](https://www.yuque.com/attachments/yuque/0/2025/pdf/2902589/1750242691975-781cf873-dd56-4a33-a479-1d2287ab241f.pdf)

# 4. ROS2 功能包简介
+ 安装依赖包

```bash
sudo apt-get install ros-<ros2版本>-radar-msgs
sudo apt-get install ros-<ros2版本>-can-msgs

# 以 ros2版本为 Foxy为例
sudo apt-get install ros-foxy-radar-msgs
sudo apt-get install ros-foxy-can-msgs

# galactic
sudo apt-get install ros-galactic-radar-msgs
sudo apt-get install ros-galactic-can-msgs
```

+ 组建与运行

```bash
colcon build --packages-select mr76_radar_ros2_driver

source install/local_setup.bash 

ros2 launch mr76_radar_ros2_driver mr76_radar_ros2_driver.launch.py 
```

# 5. 参考资料
+ [https://github.com/ros-industrial/ros_canopen](https://github.com/ros-industrial/ros_canopen)
+ [https://github.com/harrylal/radar_mr76](https://github.com/harrylal/radar_mr76)
+ [https://github.com/ros-perception/radar_msgs](https://github.com/ros-perception/radar_msgs)
+ [https://docs.ros.org/en/ros2_packages/rolling/api/visualization_msgs/msg/Marker.html](https://docs.ros.org/en/ros2_packages/rolling/api/visualization_msgs/msg/Marker.html)



