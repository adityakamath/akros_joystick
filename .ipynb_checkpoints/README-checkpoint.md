# akros_joystick

### Description
ROS node for a 3rd party SixAxis controller to be used as the main remote controller for the AKROS platforms. This node performs the following functionality:
* Replicates the functionality of teleop_twist_keyboard
** Left joystick for linear/throttle
** Right joystick for angular/steering
** Left button pad for scaling linear/angular (up/down) and only angular (left/right) velocities
** Right button pad for constant linear velocity (Y), scaling constant linear velocity (X/B)
* Additional features like start/stop, mode select
** Start button for starting/stopping the controller
** Select button to loop between 3 modes - teleop, fully autonomous, semi-autonomous (linear velocity provided by controller)
This node requires the ROS Joy (generic Linux joystick driver in ROS) package to be installed

### Configuration
The config_joystick.yaml file in the 'config' directory stores the key mapping between the joy node defaults and the 3rd party controller. This mapping is done once by running the joy node and comparing the published topics with the buttons pressed. The configuration file is loaded into the parameter server when the node is initialized 

### Subscribers
The node subscribes to the `/joy` topic,  which includes the values of each button/joystick of the controller at 50Hz. 

### Publishers
There are two publishers:
* `/joy_node/cmd_vel`: Twist output like the teleop_twist_keyboard package
* `/joy_node/mode`: Int8 output - 0, 1, 2, 3 to indicate 'undefined' and the 3 driving modes

### Launch
A launch file is provided:
* joystick_sixaxis.launch: `roslaunch akros_joystick joystick_sixaxis.launch`
