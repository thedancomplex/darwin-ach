# Darwin Lofaro Legacy
The Darwin Lofaro Legacy system is made to run on an origonal ROBOTIS Darwin OP platform utilizing ros2 as a middle ware to allow for communication between the origonal computer and a second computer such as a raspi.

# CM730 Driver

Designed to talk directly with the CM730.  Updates the state and reference

# Changes
* Call motors, FTs, and the CM730 to have a return status level of 1 (default is 2)
* Change the LEFT FT sensor to have an ID of 111.
* Change the RIGHT FT sensor to have an ID of 112.
* Added a timer to watch the achd and restart the ros2achd bridge if needed

# Todo:
* Impliment walking via ROS utilizing the ROBOTIS walking gate but comunicating via ROS
* Impliment alternative serial library to cut down on the latency and overhead caused by the dyn lib
* Make a version that just publishes the IMU data and takes upper body commands and a twist message for walking
* Put the ros2ach bridge monitor in the darwin-ach script
* Design a backpack for Raspi Zero W2
* Procure a Zero W2
* Impliment USB to Ethernet on Zero W2

# Joint Index for the Darwin-Op
![Darwin OP Joint Index](img/op_id_map.jpg)


# Install
## Darwin OP's Computer
These tutorials assume that you have Debian 11 32bit (headless/server) installed on the Darwin's computer (FitPC2)

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
This tutorial assumes that you have Debial 11 32bit (headless/server) installed on the comptuer.  This tutorial will install the base system on the comptuer with the optional step of enabling auto-start of the software on boot. This tutorial also assumes that you have SSHed into the Darwin's main computer (FitPC2).

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
$ cd darwin-op-debian-11/install
$ ./install-darwin.sh
```
This script will install everything needed and enable the Darwin Lofaro Legacy / Darwin-Ach system to start on boot.

##### 3. (Optional) Disable Darwin Lofaro Legacy / Darwin-Ach auto start on boot

If you would like to stop Darwin Lofaro Legacy / Darwin-Ach from starting on boot run the following:
```
$ ./install.sh darwin-auto-start-server-stop 
```


## Backpack Computer
### Network Configuration 
The backpack and the main darwin computer are hooked up via a single cable.  In "olden days" this would require a "crossover cable" but a regualr eithernet cable will be sufficent as modern ethernet chipsets automatically detect when a system is in a crossover mode.

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

Note: The 10.111.111.xxx network is only avaliable between the backpack and the Darwin's computer.  You will need to use the wifi connection to connect to the robot from an external computer.

#### Backpack Wifi
To set the wifi for the backpack computer run the following:

```
$ vi install/51-wifi-init.yaml
```

- Change "YOUR_SSID" to the SSID of your network.
- Change "YOUR_PASSWORD" to the password to your wifi network.
- Change "wlan0" to your wireless lan device name.  Note: If you are running Ubuntu 22.04 on a raspi this should be wlan0 already and will not need to be changed.  However this should be verified.

Now install and apply the settings.

```
$ cd install
$ ./install.sh backpack-network-wifi
```

Now the backpack computer should be connected to your wifi network. 

 
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
$ cd darwin-op-debian-11/install
$ ./install-backpack.sh
```
This script will install everything needed and enable the Darwin Lofaro Legacy / Darwin-Ach / Ros2AchBridge system to start on boot.

##### 3. (Optional) Disable Darwin Lofaro Legacy / Darwin-Ach / Ros2AchBridge auto start on boot

If you would like to stop Darwin Lofaro Legacy / Darwin-Ach / Ros2AchBridge from starting on boot run the following:
```
$ ./install.sh darwin-auto-start-ros-bridge-stop
```

