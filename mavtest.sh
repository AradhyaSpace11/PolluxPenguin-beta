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
gnome-terminal --tab -- bash -c "python3 detector.py; exec $SHELL" &
sleep 1
gnome-terminal --tab -- bash -c "python3 chfifo.py; exec $SHELL" &
sleep 1
gnome-terminal --tab -- bash -c "python3.10 vision.py; exec $SHELL" &
