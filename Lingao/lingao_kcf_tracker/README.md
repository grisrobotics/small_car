# 灵遨科技 深度视觉KCF跟踪目标

该项目使用深度摄像头跟踪选中物体基于KCF追踪算法

## 节点启动

1. 启动底盘基础驱动
``` linux
roslaunch lingao_bringup lingao_base_driver.launch
```

2. 启动KCF跟踪节点（必须在本地机器人界面端启动）
``` linux
roslaunch  lingao_kcf_tracker lingao_kcf_tracking.launch
```
节点启动后弹出`RGB Image window`窗口，通过鼠标框选跟踪物品。
按下g键启动跟踪，按下s键停止
