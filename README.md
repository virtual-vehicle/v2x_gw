# V2X Gateway
The *v2x_gw* is part of the [vehicleCAPTAIN](https://github.com/virtual-vehicle/vehicle_captain) toolbox.

The *v2x_gw* is a central ROS2 node that (i) connects to the vehicleCAPTAIN routing core (ii) converts V2X messages to ROS messages (iii) converts ROS messages to V2X messages.

The vehicleCAPTAIN routing core may be replaced by your implementation for rx/tx of V2X messages.

Version Support:
* v1.x
  * [v2x_msgs](https://github.com/virtual-vehicle/v2x_msgs) v1.1.0
  * [vcits](https://github.com/virtual-vehicle/vehicle_captain_its_lib_c_cxx) 1.0

## Copyright
Please cite the [vehicleCAPTAIN paper](https://TODO_link_to_paper_when_it_is_published) if you used any part of this library for your work.

## Contribution Guidelines
Feel free to add fixes and new features!

## Authors
Main Author: [Christoph Pilz](https://github.com/MrMushroom)

## Acknowledgement
The majority of this work is part of my ([Christoph Pilz](https://www.researchgate.net/profile/Christoph-Pilz)) PhD studies at [Graz University of Technology](https://www.tugraz.at/home) in cooperation with the [Virtual Vehicle Research GmbH](https://www.v2c2.at/).

Features are also integrated across various funded projects (see [vehicleCAPTAIN](https://github.com/virtual-vehicle/vehicle_captain)).
