Path planning 프로젝트에서 충돌회피를 위한 local planning 알고리즘 중 하나인 dwa 알고리즘을 구현했다.

이번 레포지토리는 파이썬에서 구현하고 pygame으로 시각화 한 dwa 알고리즘을 ROS2 humble의 GAZEBO로 옮겨서, 시뮬레이터 내에서 현실 상황에 맞게 실시간 장애물 회피를 하는 것이 목적이다. 그러기 위해선 추가적인 공부가 꽤나 필요했다.

먼저 2D lidar 센서의 Laserscan 데이터를 통한 장애물 인식이 필요했다.

# 1. Obstacle Detection



시뮬레이션 환경에선 2D Lidar의 laserscan 데이터로 장애물을 인식하는 방법을 먼저 알아야 했다. 

Ros의 laserscan 데이터는 센서의 스캔 시작점 angle_min 에서부터 스캔과 다음 스캔 사이의 각도 angle_increment을 이용해서 각 스캔의 거리 데이터 ranges를 글로벌 좌표로 나타낼 수 있다. 


# 2. Gazebo

처음부터 직접 World를 새로 만들어서 cmd_vel로 제어 명령을 넣으려니 로봇의 각속도 제어가 너무 느렸다. 그래서 turtlebot3의 기본 월드에서 먼저 사용법을 익히기로 했다.

dwa는 속도와 각속도를 제어 입력으로 사용하니 기존의 cmd_vel에 명령을 주기가 편했다.


[gazebo1.webm](https://github.com/user-attachments/assets/96f276f8-b743-44b3-80b2-3b66d5cb7d8d)




하지만 pygame에서와는 달리 gazebo에서는 장애물이 근처에 있을 때 최적의 제어입력 best_vw가 형성되지 않는 경우가 매우 많았다.

그래서 일단은 장애물의 반지름을 보수적으로 넓게 잡은 후, best_vw가 존재하지 않을 시 강제로 속도 입력을 내서 충돌을 회피하게 만들었다. 개선이 필요해 보인다.



