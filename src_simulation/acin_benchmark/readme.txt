1) roslaunch acin_moveit demo.launch db:=true
2) Save scene, start state and query
3) Stop the launch file
4) Edit config in acin_benchmark
5) roslaunch acin_benchmark benchmark.launch
6) Change to desired directory
7) rosrun moveit_ros_benchmarks moveit_benchmark_statistics.py <path_to_log_file>
8) Upload benchmark.db on http://plannerarena.org/
