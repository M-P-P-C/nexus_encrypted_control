# nexhom_encrypted_control
A ROS package that uses homomorphic encryption in the control a formation of robots for the DTPA lab at the University of Groningen

# Installing

Download these files and put them in a package called "nexus_encrypted_control" within your catkin workspace, and run "catkin_make"

# Launch Scripts

This package contains multiple launch files:

- sim_spawn_and_control: Launch the formation without encryption

- sim_spawn_and_control_encrypted_formation_with_estimator: Launch the encrypted formation with estimator using "xterm" terminals

- sim_spawn_and_control_encrypted_formation_with_estimator_gnome: Launch the encrypted formation with estimator using "gnome" terminals

- sim_spawn_and_control_encrypted_one_robot_FILTER: Launch the formation with encryption, using the research done by Suzanne on rlms adaptive filter

- sim_spawn_and_control_est: Launch the formation without encryption, with estimation

#Dataflow

![alt text](Encryption_Dataflow.svg)
