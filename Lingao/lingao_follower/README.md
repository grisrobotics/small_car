# 灵遨 跟随节点

## 节点 - HSV视觉寻线

该项目使用摄像头寻线

### 视觉寻线检测启动

1. 启动寻线节点 + 摄像头节点
```
roslaunch  lingao_follower hsv_detection.launch
```

### 视觉寻线跟随启动

1. 启动底盘基础驱动
```
roslaunch lingao_bringup lingao_base_driver.launch
```

2. 启动寻线节点 + 摄像头节点
```
roslaunch  lingao_follower lingao_line_follow.launch
```

### 查看
1. 使用rqt_image_view查看寻线
```
rosrun rqt_image_view rqt_image_view
```

2. 使用rqt_reconfigure动态调节参数
```
rosrun rqt_reconfigure rqt_reconfigure
```

## 节点 - 深度视觉跟踪

1. 启动底盘基础驱动
```
roslaunch lingao_bringup lingao_base_driver.launch
```

3. 启动深度视觉跟踪节点
```
rosrun  lingao_follower lingao_depth_follower.launch
```