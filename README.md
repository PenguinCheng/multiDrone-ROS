# multiDrone-ROS
本工程基于开源飞控程序Ardupilot，以及Mavros，实现多个飞行器的编队控制。
Package:
px4ros: 负责数据通信，从串口/网口读写数据，解码接收到的数据
spiri_go: 主程序，程序完整流程和循环控制

# leader参考代码
px4ros/scripts/seral_ros.py   </br>
px4rod/scripts/leader_control_decode.py   </br>
spiri_go/src/spiri_go.cpp    </br>
spiri_go/src/leader.cpp  

# follower参考代码类似
