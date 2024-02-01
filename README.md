# CrashLab
Crash Lab navigation and slam code

## simulation
* 모두 다른 터미널에서 실행
```
ros2 launch crash_simulation launch_sim.launch.py use_sim_time:=true
```
```
ros2 launch crash_nav2 online_async_launch.py use_sim_time:=true
```
```
ros2 launch crash_nav2 navigation2.launch.py use_sim_time:=true
```
```
ros2 launch crash_nav2 navigation2_rviz.launch.py use_sim_time:=true
```
```
ros2 run crash_nav2 navigate_controller
```
  
* 모드 pub
```
ros2 topic pub --once /HNCheck std_msgs/msg/Int8 "data: 2"
```
  
* 위치 보내는 pub “01~04”
```
ros2 topic pub --once /behaviorFromGPT std_msgs/msg/String "data: '02'"
```

## 영상자료
https://github.com/kwongeunwoo/CrashLab/assets/110722569/53645128-e71e-49b6-a2d6-ff5d8fe25cdd

