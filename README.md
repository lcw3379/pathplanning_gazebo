Path planning 프로젝트에서 충돌회피를 위한 local planning 알고리즘 중 하나인 dwa 알고리즘을 구현했다.

이번 레포지토리는 파이썬에서 구현하고 pygame으로 시각화 한 dwa 알고리즘을 ROS2 humble의 GAZEBO로 옮겨서, 시뮬레이터 내에서 현실 상황에 맞게 실시간 장애물 회피를 하는 것이 목적이다. 그러기 위해선 추가적인 공부가 꽤나 필요했다.

먼저 2D lidar 센서의 Laserscan 데이터를 통한 장애물 인식이 필요했다.

# 1. Obstacle Detection


