/home/hggshiwo/ardupilot/Tools/autotest/sim_vehicle.py --no-rebuild --no-mavproxy -v ArduCopter
roslaunch gazebo_sim apm.launch fcu_url:=tcp://:5760@
rosrun mavproxy_ros connection.py
rosrun mavproxy_ros control.py

rosrun mavros mavsys rate --all 10