代码c语言实现，可以运行在stm32嵌入式平台，小车属于阿克曼转向。
使用纯跟踪和Stanley算法结合，车体最小转弯半径为R的情况下，当车体中心位置到目标点轨迹切线距离大于2倍R距离使用Stanley算法，使得车体靠近轨迹，反之使用改进的纯跟踪方法，不是跟踪前视点的位置，改为跟踪前视点轨迹的切线方向，并且切点距离前视点在一个R范围内。
具体效果如下：
<div align="center">
  <img src="https://github.com/dockermyself/ackerman_car_nav_control/blob/master/result/5.png">
</div>