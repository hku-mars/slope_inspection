# mars_path_search

# 1 Astar

提供一种General的 A*搜索算法，

![image-20221211125010640](/home/yunfan/.config/Typora/typora-user-images/image-20221211125010640.png)

当起点或终点位于局部地图外时，算法会自动选择连接地图中心最近的切入点。

如果起点在地图中，算法会自动选择和起点连线的点为切入点（Local End Point）

![image-20221211205709713](/home/yunfan/.config/Typora/typora-user-images/image-20221211205709713.png)