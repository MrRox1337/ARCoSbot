Script to run simulation mode with map (change room.yaml):-

ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=False map:=/home/aman/workspaces/erc_ws/src/tortoisebot/tortoisebot_bringup/maps/room.yaml


Script to run the simulation in exploration:-

ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=True


Script to run the robot in exploration:-

ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=True