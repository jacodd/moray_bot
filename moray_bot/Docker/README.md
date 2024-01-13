```
docker build -t moray .
sh docker_run.sh 
```

```
ros2 launch moray_bot gazebo_gps_world.launch.py &
```

wait until gazebo loads

```
ros2 launch moray_bot gnss_waypoint_follower.launch.py &
```

wait until nav2 loads

```
ros2 run moray_bot follower
```
