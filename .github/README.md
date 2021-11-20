# Nexus Encrypted Control
nexus_encrypted_control is a ROS package that uses homomorphic encryption in the control a formation of robots for the DTPA lab at the University of Groningen.

# Background
You can find the original thesis [here](https://fse.studenttheses.ub.rug.nl/21680/), or the resulting conference paper [here](https://arxiv.org/abs/2104.07684#).

For more information on the homomorphic encryption see the work by Junsoo Kim in [[1]](#1)

# Installing

Download these files and put them in a package called "nexus_encrypted_control" within your catkin workspace, and run "catkin_make"

# Launch Scripts

This package contains multiple launch files:

- sim_spawn_and_control: Launch the formation without encryption

- sim_spawn_and_control_encrypted_formation_with_estimator: Launch the encrypted formation with estimator using "xterm" terminals

- sim_spawn_and_control_encrypted_formation_with_estimator_gnome: Launch the encrypted formation with estimator using "gnome" terminals

- sim_spawn_and_control_encrypted_one_robot_filter: Launch the formation with encryption, using the research done by Suzan on rlms adaptive filter

- sim_spawn_and_control_encrypted_one_robot_filter_alt: Launch the formation with encryption, using the research done by Suzan on rlms adaptive filter. Using the same type of controller used in her thesis. (This one works better than the other)

- sim_spawn_and_control_est: Launch the formation without encryption, with estimation

#Dataflow

![alt text](Encryption_Dataflow.svg)

# References

<a id="1">[1]</a> 
Kim J., Shim H., Han K. (2020). 
"*Comprehensive Introduction to Fully Homomorphic Encryption for Dynamic Feedback Controller via LWE-based Cryptosystem*". 
In: Farokhi F. (eds) Privacy in Dynamical Systems Springer, Singapore. https://doi.org/10.1007/978-981-15-0493-8_10