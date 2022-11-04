# Darwin-Ach and Darwin Lofaro Legacy System
The Darwin-Ach and the Darwin Lofaro Legacy System is made to run on an origonal ROBOTIS Darwin OP platform utilizing Ach and Ros2 as a middle ware to allow for communication between the origonal computer and a second computer such as a raspi.
The primary purpose of this project is to enable the old (or legacy) Darwin OP to utilize modern software.
The primary reason why the current plaform can not utilize modern software is because the onboard computer is outdated and is a 32 bit system with minimal memory and compute power.

# Assumptions:
* Access to a 3D printer or an alternative way of making a mounting system for a Raspberry Pi (Raspi) 3 or better (or Zero W2)
* Have a Raspi 3 or better (or Zero W2)
* Have a short USB to micro USB cable
* Have a short ethernet cable
* Have the ability to operate in a linux enviroment
* Have a ROBOTIS Darwin-OP (origonal version).

# Changes
* This system is tested with the motors, FTs sensors, and the CM730 to have a return status level of 1 instead of the default is 2
* Ensure the LEFT FT sensor to have an ID of 111.
* Ensure the RIGHT FT sensor to have an ID of 112.

# Todo:
* Impliment alternative serial library to cut down on the latency and overhead caused by the dyn lib
* Design a backpack for Raspi Zero W2
* Procure a Raspi Zero W2
* Impliment USB to Ethernet on Zero W2
* Test the auto install for the Darwin side
* Test the auto install for the Backpack side
* Design 3D printed backpack case for the Raspi 3b+ and 4
* Design 3D printed backpack csse for the Raspi Zero W2
* Verify auto install for main computer
* Verify auto install for backpack comptuer

# Joint Index for the Darwin-Op
The following are the joint indexes for the Darwin OP:

![Darwin OP Joint Index](img/op_id_map.jpg)

<table>
<tr> 
<th>Software Definition / Abveration</th><th>Numberical ID</th><th>Human Readable Name</th>
</tr>
<tr> 
<th>RSP</th><th> 1 </th><th> Right Shoulder Pitch </th>
</tr>
<tr> 
<th>LSP</th><th> 2 </th><th> Left Shoulder Pitch </th>
</tr>
<tr> 
<th>RSR</th><th> 3 </th><th> Right Shoulder Roll </th>
</tr>
<tr> 
<th>LSR</th><th> 4 </th><th> Left Shoulder Roll </th>
</tr>
<tr> 
<th>REP</th><th> 5 </th><th> Right Elbow Pitch </th>
</tr>
<tr> 
<th>LEP</th><th> 6 </th><th> Left Elbow Pitch </th>
</tr>
<tr> 
<th>RHY</th><th> 7 </th><th> Right Hip Yaw </th>
</tr>
<tr> 
<th>LHY</th><th> 8 </th><th> Left Hip Yaw </th>
</tr>
<tr> 
<th>RHR</th><th> 9 </th><th> Right Hip Roll </th>
</tr>
<tr> 
<th>LHR</th><th> 10 </th><th> Left Hip Roll </th>
</tr>
<tr> 
<th>RHP</th><th> 11 </th><th> Right Hip Pitch </th>
</tr>
<tr> 
<th>LHP</th><th> 12 </th><th> Left Hip Pitch </th>
</tr>
<tr> 
<th>RKP</th><th> 13 </th><th> Right Knee Pitch </th>
</tr>
<tr> 
<th>LKP</th><th> 14 </th><th> Left Knee Pitch </th>
</tr>
<tr> 
<th>RAP</th><th> 15 </th><th> Right Ankle Pitch </th>
</tr>
<tr> 
<th>LAP</th><th> 16 </th><th> Left Ankle Pitch </th>
</tr>
<tr> 
<th>RAR</th><th> 17 </th><th> Right Ankle Roll </th>
</tr>
<tr> 
<th>LAR</th><th> 18 </th><th> Left Andle Roll </th>
</tr>
<tr> 
<th>NKY</th><th> 19 </th><th> Neck Yaw </th>
</tr>
<tr> 
<th>NKP</th><th> 20 </th><th> Neck Pitch </th>
</tr>
</table>

# ROS 2 Topics
The follow are the feedforward and feedback ROS 2 Topics

## Feedback Topics
The feedback topics are updated at the rate in which the Darwin-Ach and Darwin Lofaro Legacy system is running.

### Inertial Measurement Unit (IMU)
* Message Type: Twist
* The accelerometer information for x, y, and z is in the "linear" portion of the message in units of m/s^2
* The gyro (rotational velocity) information for x, y, and z is in the "angular" portion of the message in units of rad/s

Topic:
```
/darwin/state/imu
```

### Force Torque Left
Topic:
```
/darwin/state/ft/left
```
* Message Type: Twist
* Each of the values are a float. Units for linear.x and linear.y values are in N. Units for lift state (linear.z, angular.x, and angular.y) are booleans (0 or 1).

### Force Torque Right
Topic:
```
/darwin/state/ft/right
```
* Message Type: Twist
* Each of the values are a float. Units for linear.x and linear.y values are in N. Units for lift state (linear.z, angular.x, and angular.y) are booleans (0 or 1).

### Motor Load
Topic:
```
/darwin/state/motor/load
```
* Message Type: Float64MultiArray
* Each of the values are floats where the index of the given motor is given in the index section of this document.
* Unit: N/m

### Motor Position
Topic:
```
/darwin/state/motor/position
```
* Message Type: Float64MultiArray
* Each of the values are floats where the index of the given motor is given in the index section of this document.
* Unit: rad

### Motor Speed
Topic:
```
/darwin/state/motor/speed
```
* Message Type: Float64MultiArray
* Each of the values are floats where the index of the given motor is given in the index section of this document.
* Unit: m/s

### Motor Temperature
Topic:
```
/darwin/state/motor/temperature
```
* Message Type: Float64MultiArray
* Each of the values are floats where the index of the given motor is given in the index section of this document.
* Unit: Celsius

### Motor Voltage
Topic:
```
/darwin/state/motor/voltage
```
* Message Type: Float64MultiArray
* Each of the values are floats where the index of the given motor is given in the index section of this document.
* Unit: Volts

### Time
Topic:
```
/darwin/time
```
* Message Type: Float64
* The value is in float and is the time on the main computer on the Darwin OP
* Unit: sec

## Feedforward Topics
The feedforward topics can be posted at any rate.  The actoators will updated at the rate that the Darwin-Ach and Darwin Lofaro Legacy system is running. If multiple values are posted between motor updates only the most recent value will be applied to the motor. 

### Desired Motor Position
Topic:
```
/darwin/ref/position
```
* Message Type: String
* Message Example: 'r 1 1.234 17 -0.123 6 0.23'
* * This message sets the desired position of motor 1, 17, and 6 to 1.234, -0.123, and 0.23 rad respectively
* * The "r" denotes rad 
* Message Example: 'd 1 10.2 17 -20.3456 6 7.23'
* * This message sets the desired position of motor 1, 17, and 6 to 10.2, -20.3456, and 7.23 deg respectively
* * The "d" denotes deg
* The length of the message can be any length as long as it keeps the order as stated above.
* The "String" message is used instead of a custom message to allow for greater compatiability with future systems.
* This message "stages" the given values but does not send them to the motors.  You need to "post" (or apply) the messages using the "post" command on the "cmd" topic.

### Desired Motor Speed
Topic:
```
/darwin/ref/speed
```
* Message Type: String
* Message Example: 'r 1 1.234 17 -0.123 6 0.23'
* * This message sets the desired speed of motor 1, 17, and 6 to 1.234, -0.123, and 0.23 rad/s respectively
* * The "r" denotes rad/s
* Message Example: 'd 1 10.2 17 -20.3456 6 7.23'
* * This message sets the desired speed of motor 1, 17, and 6 to 10.2, -20.3456, and 7.23 deg/s respectively
* * The "d" denotes deg/s
* The length of the message can be any length as long as it keeps the order as stated above.
* The "String" message is used instead of a custom message to allow for greater compatiability with future systems.
* This message "stages" the given values but does not send them to the motors.  You need to "post" (or apply) the messages using the "post" command on the "cmd" topic.

### Desired Motor Maximum Torque
Topic:
```
/darwin/ref/torque
```
* Message Type: String
* Message Example: 'p 1 0.5 17 0.111 6 1.0'
* * This message sets the maximum torque of motor 1, 17, and 6 to 0.5 (50%), 0.111 (11.1%), and 1.0 (100%) of the maximum torque respectively
* * The "p" denotes percent from 0.0 to 1.0
* The length of the message can be any length as long as it keeps the order as stated above.
* The "String" message is used instead of a custom message to allow for greater compatiability with future systems.
* This message "stages" the given values but does not send them to the motors.  You need to "post" (or apply) the messages using the "post" command on the "cmd" topic.

### Command Message
Topic:
```
/darwin/cmd
```
* Message Type: String
* This topic performs various commands and functions.  Each of the commands/functions occure once per cycle of the Darwin-Ach and the Darwin Lofaro Legacy system.
* The "String" message is used instead of a custom message to allow for greater compatiability with future systems.

Avaliable Commands:

#### Post
Message:
```
post
```
Post / Apply the Position, Velocity, and Torque desired reference values

#### Open
Message:
```
open
```
Open the Serial port and start the control loop on the main comptuer

#### Close 
Message:
```
close
```
Close the serial port and stop the control loop on the main comptuer 

#### Turn on power
This command turns on the power.  This also automatically apply the "open" command.

##### All Motors
Message:
```
on all
```
Turn on all motors and the main controller

##### Specific Motor
Message:
```
on [Motor Number]
```
Turn on a specific motor

Example (Turns on motor 7):
```
on 7
```

Note 1: The motor numerical IDs are defined in the motor ID section 
Note 2: The ID of the main controller board with the IMU on it is 200.


#### Turn off power
This command turns off the power.  This also automatically apply the "close" command.

##### All Motors
Message:
```
off all
```
Turn off all motors and the main controller

##### Specific Motor
Message:
```
off [Motor Number]
```
Turn off a specific motor

Example (Turns off motor 7):
```
off 7
```

Note 1: The motor numerical IDs are defined in the motor ID section 
Note 2: The ID of the main controller board with the IMU on it is 200.

#### Debug mode
Debug mode print values in the screens that the controllers are running in.  By default "debug" mode is disable. 

##### Enable 
Message:
```
debug true
```
Enable Debug Mode

##### Disable
Message:
```
debug false
```
Disable Debug Mode

#### Change Control Loop Mode
This allows for changing control loop modes which includes changing refreshrates and enable/disable state feedback.  It also chages how the reference is controlled and applied.

##### State Feedback and Control Loop Modes
Changes to the state feedback and control rate

1. 50hz, IMU, Motor, and FT
Message:
```
loop state 50hz_imu_motor_ft
```
* Loop Rate: 50hz
* IMU Feedback: Yes - 50hz
* FT Feedback: Yes - 50hz
* Motor Feedback: Yes - 50hz

2. 50hz and IMU
Message:
```
loop state 50hz_imu
```
* Loop Rate: 50hz
* IMU Feedback: Yes - 50hz
* FT Feedback: No
* Motor Feedback: No

3. 125hz and IMU
Message:
```
loop state 125hz_imu
```
* Loop Rate: 125hz
* IMU Feedback: Yes - 125hz
* FT Feedback: No
* Motor Feedback: No

4. 100hz, IMU, and FT (slow)
Message:
```
loop state 125hz_imu_ft_slow
```
* Loop Rate: 100hz
* IMU Feedback: Yes - 100hz
* FT Feedback: Yes - 50hz per FT
* Motor Feedback: No

5. 100hz, IMU, and Motors (slow)
Message:
```
loop state 125hz_imu_motors_slow
```
* Loop Rate: 100hz
* IMU Feedback: Yes - 100hz
* FT Feedback: No
* Motor Feedback: Yes - 5hz per motor

6. Default
Message:
```
loop state default
```
* Loop Rate: Default - 125hz
* IMU Feedback: Default - Yes - 125hz
* FT Feedback: Default - No
* Motor Feedback: Default - No

##### Reference Mode
This sets how the reference values are applied.

1. Normal
Message:
```
loop ref normal
```
* Control Application Rate (Upper Body): Same as control loop
* Control Application Rate (Lower Body): Same as control loop

2. Slow Top
Message:
```
loop ref slow_top
```
* Control Application Rate (Upper Body): (Control Loop Rate) / 8
* Control Application Rate (Lower Body): Same as control loop

3. Default
Message:
```
loop ref default
```
* Control Application Rate (Upper Body): Same as control loop
* Control Application Rate (Lower Body): Same as control loop

# Install
This section shows how to install the Darwin-Ach / Darwin Lofaro Legacy system on the Darwin-OP and its backpack computer.

## Darwin OP's Computer
These tutorials assume that you have Debian 11 32bit (headless/server) installed on the Darwin's computer (FitPC2)

### Debian 11 32bit
* Image Used: [Debian 11 32 bit (i386) Mirror: OSU](http://debian.osuosl.org/debian-cdimage/11.5.0/i386/iso-dvd/debian-11.5.0-i386-DVD-1.iso)
* SD Card Used: [to be posted]
* Mini SD to Micro SD Card Adapter used: [Mini to Micro SD Card adapter: Click Here](https://www.amazon.com/gp/product/B000TLXCDW/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)
* Pre-Installed Image: [to be posted]

### Network Configuration 
On the Darwin's comptuer (the fitpc) we will use the 'interfaces' method to apply the wifi settings and the static ip for connecting to the backpack.

```
$ vi install/interfaces
```
- Change "YOUR_WIFI_INTERFACE" to the name of your wireless interface.
- Change "YOUR_SSID" to the SSID of your network.
- Change "YOUR_PASSWORD" to the password to your wifi network.

Note: The ethernet is set to static with the ethernet having a higher number metric (lower priority) than the wifi.  This allows the comptuer to get internet from the wifi.

Now run the following:

```
$ cd install
$ ./install.sh darwin-network
```

This will conect the Darwin's comptuer to your wifi network and set the ethernet on the back to have the static ip of:

```
10.111.111.11
```

### Darwin Computer Software Instillation
This tutorial assumes that you have Debian 11 32bit (headless/server) installed on the comptuer.  This tutorial will install the base system on the comptuer with the optional step of enabling auto-start of the software on boot. This tutorial also assumes that you have SSHed into the Darwin's main computer (FitPC2).

#### Install Base Software (Darwin Lofaro Legacy and Darwin-Ach)
A script has been made for easy install of the Darwin Lofaro Legacy and Darwin-Ach system.

##### 1. Run in a screen

To ensure the software is installed correctly even if network connection is lost it is highly recomended to run the instillation inside of a "screen".  The following steps will install the "screen" software, if it is not already installed, then it will run a "screen" session.
```
$ sudo apt update
$ sudo apt install screen
$ screen
```

##### 2. Install the base software

Inside the terminal that has the "screen" running run the following script.
```
$ cd darwin-ach/install
$ ./install-darwin.sh
```
This script will install everything needed and enable the Darwin Lofaro Legacy / Darwin-Ach system to start on boot.

##### 3. (Optional) Disable Darwin Lofaro Legacy / Darwin-Ach auto start on boot

If you would like to stop Darwin Lofaro Legacy / Darwin-Ach from starting on boot run the following:
```
$ ./install.sh darwin-auto-start-server-stop 
```

#### Install Examples
The following examples are applicaple to being used on the Darwin's main computer (FitPC2).  As with the instillation, this assumes that you are SSHed into the Darwin's main computer.
```
$ git clone https://github.com/darwin-op/darwin-ach-simple-demo-cpp
```
Note: The README in the example above explains the usage.  If you system is setup to be "auto start" you can skip the "Start Darwin-OP" steps.

## Backpack Computer
### Ubuntu 22.04 32bit for Raspi
* Image Used (32 Bit): [to be posted]
* Alternative Image (64 Bit): [Click Here](https://cdimage.ubuntu.com/releases/22.04.1/release/ubuntu-22.04.1-preinstalled-server-arm64+raspi.img.xz?_ga=2.173845286.468747468.1665704955-657318545.1654912167)
* SD Card Used: [to be posted]
* Mini SD to Micro SD Card Adapter used: [Click Here](https://www.amazon.com/gp/product/B000TLXCDW/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)
* Pre-Installed Image: [to be posted]

### Pre-Setup
Be sure to update and upgrade the system first.  This step assums that you are SSHed into the backpack comptuer and it has an active network connection.

1. Update and Upgrade your system
```
$ sudo apt update
$ sudo apt upgrade
```

2. Reboot
```
$ sudo reboot
```

3. Install git
```
$ sudo apt install git
```

4. Install screen
```
sudo apt install screen
```

5. Run screen
```
$ screen
```

6. Clone the Darwin-Ach / Darwin Lofaro Legacy system
```
$ mkdir projects
$ cd projects
$ git clone https://github.com/thedancomplex/darwin-ach
```

7. Go into the Darwin-Ach / Darwin Lofaro Legacy system's folder
```
$ cd darwin-ach
```

### Network Configuration 
The backpack and the main darwin computer are hooked up via a single cable.  In "olden days" this would require a "crossover cable" but a regualr eithernet cable will be sufficent as modern ethernet chipsets automatically detect when a system is in a crossover mode.

#### Backpack Wifi
To set the wifi for the backpack computer run the following:

```
$ vi install/51-wifi-init.yaml
```

- Change "YOUR_SSID" to the SSID of your network.
- Change "YOUR_PASSWORD" to the password to your wifi network.
- Change "wlan0" to your wireless lan device name.  Note: If you are running Ubuntu 22.04 on a raspi this should be wlan0 already and will not need to be changed.  However this should be verified.
- Note: Some Raspi's only have a 2.4ghz radio.  Be sure to pick a wireless network compatiable with your wireless hardware/radio.

Now install and apply the settings.

```
$ cd install
$ ./install.sh backpack-network-wifi
```

Now the backpack computer should be connected to your wifi network. 

#### Backpack Ethernet

The following will automatically configure the ethernet to connect with the Darwin OP's computer:

```
$ cd install
$ ./install.sh backpack-network
```

This will connect the backpack to the Darwin's computer via a network with a static IP.  The static IP of the backpack computer is:

```
10.111.111.10
```

Note 1: If you were connected via SSH via the ethernet cable then the connection will be lost.  Be sure to reconnect via the wifi address.

Note 2: Connect the ethernet cord from the Raspi to the Darwin-OP.

Note 3: The 10.111.111.xxx network is only avaliable between the backpack and the Darwin's computer.  You will need to use the wifi connection to connect to the robot from an external computer and access the internet.

### Darwin's Backpack Computer Software Instillation
This tutorial assumes that you have Ubuntu 22.04 32bit (headless/server) installed on the comptuer.  This tutorial will install the base system on the comptuer with the optional step of enabling auto-start of the software on boot. This tutorial also assumes that you have SSHed into the Darwin's backpack computer (Raspi).  This was tested on a Raspi 3b+ and will be tested on other SBCs in the future.

#### Install Base Software (Darwin Lofaro Legacy, Darwin-Ach, Ros2, and Ros2AchBridge)
A script has been made for easy install of the Darwin Lofaro Legacy, Darwin-Ach system, Ros2, and the Ros2AchBridge.

##### 1. Run in a screen

To ensure the software is installed correctly even if network connection is lost it is highly recomended to run the instillation inside of a "screen".  The following steps will install the "screen" software, if it is not already installed, then it will run a "screen" session.
```
$ sudo apt update
$ sudo apt install screen
$ screen
```

##### 2. Install the base software

Inside the terminal that has the "screen" running run the following script.  Due to the fact that we have to install Ros2 from source this will take around 20 to 24 hours to complete.
```
$ cd darwin-ach/install
$ ./install-backpack.sh
```
This script will install everything needed and enable the Darwin Lofaro Legacy / Darwin-Ach / Ros2AchBridge system to start on boot.

##### 3. (Optional) Disable Darwin Lofaro Legacy / Darwin-Ach / Ros2AchBridge auto start on boot

If you would like to stop Darwin Lofaro Legacy / Darwin-Ach / Ros2AchBridge from starting on boot run the following:
```
$ ./install.sh darwin-auto-start-ros-bridge-stop
```
#### Install Examples
The following examples are applicaple to being used on the Darwin's backpack computer (Raspi).  As with the instillation, this assumes that you are SSHed into the Darwin's backpack computer.

##### C++ Example (no Ros2)
```
$ git clone https://github.com/darwin-op/darwin-ach-simple-demo-cpp
```
Note: The README in the example above explains the usage.  If you system is setup to be "auto start" you can skip the "Start Darwin-OP" step.  If "auto start" is not enabled then you also need to enable the "client" which setups the remote comunication via Achd.

##### Python Example (using Ros2)
```
$ git clone https://github.com/darwin-op/darwin-ach-simple-demo-ros2-python
```
Note: This example assumes that you have both the Darwin's main comptuer and backpack computer setup as "auto start".  The README in the example has a detailed explination of each of the examples.  These examples can also be run on an external comptuer that is also running Ros2 on the same Ros2 network.

