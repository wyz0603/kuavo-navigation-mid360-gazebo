#!/bin/bash

# Source and launch nimservos_controller package
gnome-terminal -- bash -c "
    cd ../..;
    source devel/setup.bash;
    roslaunch nimservos_controller nimservos_controller.launch;
    exec bash"

# Source and launch user_pkg main.py for keyboard control
gnome-terminal -- bash -c "
    cd ../..;
    source devel/setup.bash;
    rosrun user_pkg main.py;
    exec bash"

echo "SkyRail automated test setup has been started. UI control window for keyboard listening will appear shortly."
