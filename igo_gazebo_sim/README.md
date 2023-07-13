
# Short description on how to use this package
## Run only simulation from docker container
Move to the workspace top-level folder and in there in the subfolder `docker`. Within this folder run the simualtion by 
- `make nav2-run-simulation` using a humble-base-image
- `make nav2-run-simulation-foxy` using a foxy-base-image

If you got an error that the display can not be opened, get your current display using `echo $DISPLAY` and modify the file `docker/setDisplay.sh`accordingly. The restart the container as shown above.

## Dealing with multiple terminals within one container
To run the simualtion in combination with the nav2-stack, start a terminal-docker-container using
- `make nav2-term` using a humble-base-image
- `make nav2-term-foxy` using a foxy-base image
Within this terminal, several terminals have to be launched. For this, start a terminal muxer by `tmux`. Use the following command for managing multiple terminals:
- `Ctrl-b "` for new terminal
- `Ctrl-b o` for change to next terminal
- `Ctrl-b x` for close terminal (mind to answer the question at the bottom status bar on closing the terminal with "y")

Initialize every terminal by

`source install/setup.bash && source docker/setDisplay.py`

Launch inside the terminals:
- 1st terminal: `source docker/setGazeboPath.py && ros2 launch igo_gazebo_sim sim_igo_workshop_warehouse.launch.py`
- 2nd terminal: `ros2 launch igo_nav2_sim igo_navigation2.launch.py use_sim_time:=True`
- 3rd terminal: `ros2 run igo_nav2_sim igo_waypoint_pilot`

If you got an error reporting that no display can be opened, have a look to the comment above on how to fix this error.

## Run with native ROS Humble installation (real warehouse environemnt)
In a second simulation, the operation environment of the IMOCO scenario is modeled. In a later project step the essenetial parts of the scenario will be added to the simulation. 
For starting the scenario incl. a waypoint-based navigation launch inside terminals:
- 1st terminal: `ros2 launch igo_gazebo_sim sim_igo_real_warehouse.launch.py`
- 2nd terminal: `ros2 launch igo_nav2_sim igo_navigation2.launch.py use_sim_time:=True localization_method:='0_map_frame'`
- 3rd terminal: `ros2 run igo_nav2_sim igo_waypoint_pilot`
