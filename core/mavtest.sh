#!/bin/bash

# Source the API key from the .env file
source "$HOME/PolluxPenguin-beta/.env"
export GOOGLE_API_KEY 

# Change to the PolluxPenguin directory
cd "$HOME/PolluxPenguin-beta/core" || exit

gnome-terminal --tab -- bash -c "cd ~/ardupilot/Tools/autotest && ./sim_vehicle.py -v ArduCopter -f gazebo-iris -I0; exec $SHELL" &
sleep 1
gnome-terminal --tab -- bash -c "roslaunch gazebo_ros multi.launch; exec $SHELL" &
sleep 8
gnome-terminal --tab -- bash -c "python3 controller.py; exec $SHELL" &
sleep 2

# Now run Python programs with access to the API key
gnome-terminal --tab -- bash -c "python3.10 llm_communicator.py; exec $SHELL" &
sleep 1
gnome-terminal --tab -- bash -c "python3 balldetector.py; exec $SHELL" &
sleep 1
gnome-terminal --tab -- bash -c "python3 chfifo.py; exec $SHELL" &
sleep 1

source visiongpt-env/bin/activate
gnome-terminal --tab -- bash -c "python3 visiongpt.py; exec $SHELL" &
sleep 1

# Start the web server
cd "$HOME/PolluxPenguin-beta/webApp" || exit
gnome-terminal --tab -- bash -c "node server.js; exec $SHELL" &
