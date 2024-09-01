#!/bin/bash

# Change to the PolluxPenguin directory
cd "$HOME/PolluxPenguin" || exit

gnome-terminal --tab -- bash -c "roslaunch gazebo_ros multi.launch; exec $SHELL" &
sleep 1
gnome-terminal --tab -- bash -c "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -v ArduCopter -f gazebo-iris -I0; exec $SHELL" &
sleep 1
gnome-terminal --tab -- bash -c "python3 control2.py; exec $SHELL" &
sleep 2
gnome-terminal --tab -- bash -c "python3.10 gpt7.py; exec $SHELL" &
sleep 1
gnome-terminal --tab -- bash -c "python3 olo3.py; exec $SHELL" &
sleep 1
gnome-terminal --tab -- bash -c "python3 chabofifo.py; exec $SHELL" &
sleep 1
gnome-terminal --tab -- bash -c "python3.10 gemvis.py; exec $SHELL" &
