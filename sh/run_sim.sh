roslaunch gazebo_sim iris_with_roscam.launch
sim_vehicle.py --no-rebuild --no-mavproxy -v ArduCopter -f gazebo-iris
roslaunch gazebo_sim apm.launch fcu_url:=tcp://:5760@
# rosrun mavproxy_ros connection.py
# rosrun mavproxy_ros control.py
# rosrun mavproxy_ros pland.py /pland_camera/image_raw:=/roscam/cam/image_raw
roslaunch mavproxy_ros onclick_run.launch
rosrun mavros mavsys rate --all 10
rostopic pub -r 10 /ugv_0/cmd_vel geometry_msgs/Twist -- '[0.1, 0, 0]' '[0, 0, 0]'
rosrun mavproxy_ros key_contrl.py

rosservice call /mavros/param/get "param_id: 'PLND_ENABLED'"