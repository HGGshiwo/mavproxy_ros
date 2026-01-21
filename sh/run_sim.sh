roslaunch gazebo_sim iris_with_roscam.launch
sim_vehicle.py --no-rebuild --no-mavproxy -v ArduCopter -f gazebo-iris --custom-location=30.1119319,120.140883,0,0
roslaunch gazebo_sim apm.launch fcu_url:=tcp://:5760@
# rosrun mavproxy_ros pland.py /pland_camera/image_raw:=/roscam/cam/image_raw
roslaunch mavproxy_ros onclick_run.launch
rosrun rsos_msgs fake_record.py

rosrun mavros mavsys rate --all 10
rostopic pub -r 10 /ugv_0/cmd_vel geometry_msgs/Twist -- '[0.1, 0, 0]' '[0, 0, 0]'
rosrun mavproxy_ros key_contrl.py
roslaunch ego_planner single_run_in_exp.launch odom_topic:=/mavros/local_position/odom
rosservice call /mavros/param/get "param_id: 'PLND_ENABLED'"

rostopic pub /mavros/ws std_msgs/String 'data: "{\"type\": \"event\", \"event\": \"detect\"}"' --rate 10

rosservice call /mavros/cmd/command "{command: 246, param1: 1, confirmation: true}"


