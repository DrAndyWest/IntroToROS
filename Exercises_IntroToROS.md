# Introduction to ROS: Exercises
***

### Preflight Checks
***

#### Environment Variables {#Preflight_EnviroVariables}
Once Ubuntu 20.04 LTS and ROS Noetic are installed on a PC or as a Virtual Machine (VM), there are some checks that can be performed to ensure the ROS installation was performed correctly.

Open a new terminal (*hint*: you can use `ctrl+alt+t`)  
Enter the command, `rosversion –-distro`, it should return "noetic":  
```console
user@machine:~$ rosversion --distro
noetic
```
*:exclamation: Linux Tip - use `ctrl+alt+t` to quickly open a new terminal, in a terminal `ctrl+shift+t` opens a new "tab"*  
*:exclamation: Linux Tip - copy/paste in the terminal requires an additional `shift`, i.e. copy = `ctrl+shift+c`, paste = `ctrl+shift+v`*

Enter the command, `echo $ROS_PACKAGE_PATH`, it should return:  
```console
user@machine:~$ echo $ROS_PACKAGE_PATH
/opt/ros/noetic/share
```

If you see this, close the terminal using the command `exit`, and skip to the next section: [Running ROS](#Preflight_RunningROS)



If it returns with nothing, use the command `source /opt/ros/noetic/setup.bash` and try again.  If this then returns the correct response, then simply use the command `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc` followed by `source ~/.bashrc`, this should remedy the issue.

Once complete, close the terminal using the command `exit`.

*:exclamation: Linux Tip - ensure you use `>>` (append) and not `>` (overwrite)*  
*:exclamation: Linux Tip - tilde (`~`) is shorthand for the "home" directory `/home/username/` (eqivalent to a C:\Users\username\ folder in Windows)*  
*:exclamation: Linux Tip - your bashrc file contains preferences, variables and code to be run everytime you open a terminal session, including environment variables*  
*:exclamation: Linux Tip - files or directorys with a leading fullstop (`.`) are hidden files, e.g. `.bashrc`*  
*:exclamation: Linux Tip - the word "`exit`" will close a terminal if nothing is running*  

#### Running ROS {#Preflight_RunningROS}

Double check ROS will run properly by starting a new terminal and enter the command `roscore`.  Some action should happen in the terminal, if an error such as the one below occurs, then [repeat the steps above](#Preflight_EnviroVariables).
```console
user@machine:~$ roscore
roscore: command not found
```

If this was successful, stop the process using `ctrl+c`, then close the terminal using `exit`.

*:exclamation: Linux Tip - the command `ctrl+c` stops any processes running in that terminal, be careful if copy/pasting*  

What is `roscore`?  It starts the ROS master node.  What does that even mean?  We will cover that later!

#### :boom:PREFLIGHT CHECKS | COMPRESSED INSTRUCTIONS:boom:
Open terminal
```console
rosversion --distro
echo $ROS_PACKAGE_PATH
roscore
```
Exit terminal

***
## Installing ROS Packages
***
### Install from ROS Repositories
Many common (and likely most stable) packages are freely available and documented on the [ROS Wiki](http://wiki.ros.org/).  Double check that the package is available for your ROS version (Noetic).

This exercise will demonstrate how to install an existing supported ROS package via the command line.  We will install the very useful '[rosserial](http://wiki.ros.org/rosserial)' package, allowing microcontrollers (such as Arduino compatible units) and other hardware/software to talk with ROS.

Open a new terminal session with `ctrl + alt + t`, enter the command:
`sudo apt install ros-noetic-rosserial`


The 'apt' portion is the command to find/install/remove software via terminal, followed by the 'install' command (`apt install`).  Finally, the package is defined by ros-\<version\>-\<package name\> (`ros-noetic-rosserial`).  If you are unsure of the exact name of a package, using Tab completion can help.

*:exclamation: Linux Tip - The `sudo` part means that administrator priviledges are required (in this case install software), the terminal may ask for your password*  
*:exclamation: Linux Tip - The 'Tab' key can be used to autocomplete when using the terminal, this can save time, reduce typos, and help when you can't quite remember what something is called*

### Install from Source

Installing from source typically means downloading the source code, and compiling it locally.  On most occasions any ROS code is likely freely available from somewhere like [Github](github.com), hopefully with some instructions for installation.  How to use git will not be covered here, but regardless of where the code comes from, it will need to be copied into your 'Catkin Workspace' src directory.  The next section will cover setting up the workspace and compiling code.


***
## :robot: Writing Your First Node
***
### Exercise 0.1 | Setting Up Your Catkin Workspace
***

#### Making a Workspace Directory

Open a new terminal, this should start you in your "home" directory (identified with a `~`).  Double check you are in the home directory with `cd ~`.

*:exclamation: Linux Tip - the `cd` command means "change directory"*  

Create a new directory called "catkin_ws" using `mkdir catkin_ws`.  Move into that directory with `cd catkin_ws`.  Move to the new directory and make another directory called "src" using `cd catkin_ws` followed by `mkdir src`.

*:exclamation: Linux Tip - the `mkdir` makes new directories, using the `-p` option can make sub directories and the parents in one go, e.g. `mkdir -p ~/catkin_ws/src`*  

#### Building the Catkin Workspace

Once the directories have been created we need to "make" the catkin workspace, simply using:  
`catkin_make`  
Once you have packages and nodes, this process will compile any C++ code and make sure other packages and ROS in general are aware of the packages and nodes.

*:sparkles:ROS Tip - there is also the catkin build tool, which is [regarded as a better tool](https://catkin-tools.readthedocs.io/en/latest/migration.html) than the default catkin_make.  It can be installed with `sudo apt install python3-catkin-tools` in a terminal session*  

#### Sourcing the Workspace

The ROS system needs to know where to look for these packages, and identical to the preflight checks of adding information to the .bashrc file, the same is done here.

`echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`  

Open a new terminal or use `source ~/.bashrc` to reload the current terminal.  Use the command `echo $ROS_PACKAGE_PATH` and now there should be a second path listed compared to [earlier](#Preflight_EnviroVariables).

```console
user@machine:~$ echo $ROS_PACKAGE_PATH
/home/user/catkin_ws/src:/opt/ros/noetic/share
```


#### :boom:EXERCISE 0.1 | COMPRESSED INSTRUCTIONS:boom:
Open terminal
```console
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo $ROS_PACKAGE_PATH
```
Exit terminal

***
### Exercise 0.2 | Setting Up Your First Package
***

Packages often have dependancies from different packages, e.g. message definitions.  These dependancies can be added later, however, it is usually much easier to add them when making the package to begin with.  Check out the [ROS wiki tutorial](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) for all the details.

This tutorial will use some basic dependancies to demonstrate how to create a package.  Open a terminal and navigate to the catkin workspace source folder using `cd ~/catkin_ws/src`.  The `catkin_create_pkg` command takes a few arguments, in the form `catkin_create_pkg <name_of_package> <dep_1> <dep_2> ...`.  We will create a package called "example_package" with dependancies on "rospy", "roscpp" and "std_msgs".

```console
catkin_create_pkg example_package rospy roscpp std_msgs
```

*:sparkles:ROS Tip - `rospy` and `roscpp` dependancies are for python and C++ functionality respectively, these are usually always required*

Navigate to the `catkin_ws` directory using `cd ..` (or absolute path `cd ~/catkin_ws`), and use `catkin_make`.  Some action should happen in the terminal, and your package should now be established.

*:exclamation: Linux Tip - `..` means up one directory level*  

Check the package is availble to the system.  Open a new terminal and use the command `rospack list | grep example_package`, where example is the name of the package in this case.  If this works, you should get a result similar to below.

```console
user@machine:~$ rospack list | grep example_package
example_package /home/user/catkin_ws/src/example_package
```

*:sparkles:ROS Tip - All available ROS packages can be seen using `rospack`*  
*:exclamation: Linux Tip - the pipe `|` literally "pipes" the output of one function into another, very useful*  
*:exclamation: Linux Tip - `grep` performs a partial string match (search)* 

#### :boom:EXERCISE 0.2 | COMPRESSED INSTRUCTIONS:boom:
Open terminal
```console
cd ~/catkin_ws/src
catkin_create_pkg example_package rospy roscpp std_msgs
cd ..
catkin_make
```
Exit terminal

***
### Exercise 1.0 | Package Preparation
***

Navigate to your new package using `roscd example_package`.  That's right, ROS has a version of `cd`!  Instead of using `cd ~/catkin_ws/src/example_package`, you can jump straight there with `roscd`.

Make a directory called "scripts" using `mkdir scripts` in the example_package directory.

*:sparkles:ROS Tip - Python code lives in a "scripts" directory in a package, whereas C++ code lives in a "src" directory in a package*  

#### :boom:EXERCISE 1.0 | COMPRESSED INSTRUCTIONS:boom:
Open terminal
```console
roscd example_package
mkdir scripts
```
Exit terminal

***
### Exercise 1.1 | Write a Subscriber Node
***
This exercise will create a node (filename subscriber_node.py) which will subscribe to incoming "std_msgs/Int32" messages on the "counter" topic name.  To save time, a bare bones example is available on [Github](https://github.com/DrAndyWest/IntroToROS).

Create a new file in ~/catkin_ws/src/example_package/scripts/ called 'subscriber_node.py', then open this file in your preferred text editor.  Copy the skeleton code from the [Github example](https://github.com/DrAndyWest/IntroToROS/tree/main/scripts/subscriber_node.py), and complete the tasks below.

Tasks to complete:
- [ ] Add a subscriber callback function
- [ ] Output a [rospy info log](http://wiki.ros.org/rospy/Overview/Logging) message when a new message has been received

It is a good habit to `catkin_make` when changes have been made to a node.  This is not necessary for python, but it is necessary to recompile C++ code before running.

Ensure the python code is executable.  Through the terminal use `roscd example_package/scripts/` followed by `chmod +x subscriber_node.py`.  This only needs to be done once.

Open a terminal and start the ROS master node with `roscore`.  Open a new terminal or new terminal tab and run the subscriber node with `rosrun example_package subscriber_node.py`.

Open a third terminal or tab, and manually publish data on the correct topic name using `rostopic pub -r 1 /counter std_msgs/Int32 "data: 42"`.  Go back to the terminal running the subscriber node and witness the logging information mirror the incoming message.

*:exclamation: Linux Tip - the `Tab` key can be used to autocomplete in the terminal*  
*:sparkles:ROS Tip - Using the additional command `-r 1` means publish the data at a rate of 1 Hz, without this the data is published just once*  


***
### Exercise 1.2 | Write a Publisher Node
***
Now we have something listening out for messages on the "counter" topic, this exercise will create a node to publish that data.

Create a new file in ~/catkin_ws/src/example_package/scripts/ called 'publisher_node.py', then open this file in your preferred text editor.  Copy the skeleton code from the [Github example](https://github.com/DrAndyWest/IntroToROS/tree/main/scripts/publisher_node.py), and complete the tasks below.

Tasks to complete:
- [ ] Add a variable which increases by 1 each time a message is published
- [ ] Add the neccessary code to publish a std_msg/Int32 in the main while loop of the code

Ensure the python code is executable.  Through the terminal use `roscd example_package/scripts/` followed by `chmod +x publisher_node.py`.  This only needs to be done once.

Open a terminal and start the ROS master node with `roscore`.  Open a new terminal or new terminal tab and run the publisher node with `rosrun example_package publisher_node.py`.

Open a third terminal or tab, use `rostopic echo /counter` to view the messages being published.  Stop this process with `ctrl+c` and start the publisher node with `rosrun example_package publisher_node.py`.  Witness the logging information mirror the incoming message.  Now the value should be incrementing with each message.

***
### Exercise 2.0 | Writing a Launch File
***

It can be burdensome to open many terminal sessions to run many nodes at once.  It is much more convenient to group complimentary nodes together, called from a single command.  Launch files are .xml files which can start many nodes, as well as launch other launch files.

Create directory 'launch' in 'example_package'.  Create a file called 'example.launch' in the newly created directory and open it in a text editor.

```xml
<launch>

  <!-- Format for including nodes, can be in any order -->
  <!-- pkg = package name, type = file name of node, name = rosnode name -->
  <node pkg="example_package" type="publisher_node.py" name="publisher" />
  <node pkg="example_package" type="subscriber_node.py" name="subscriber" />

</launch>
```

Launch files will also start a ROS Master node if one is not already running.  With a new terminal and no existing roscore running, start the launch file using `roslaunch example_package example.launch`.

Launch files can be quite sophisicated, with arguments and variables.  Check out [tutorials](http://wiki.ros.org/roslaunch/XML) for [more details](http://wiki.ros.org/roslaunch).

***
### Exercise 3.0 | Writing a Service Server
***

Services provide intermittent functionality for sending messages or performing tasks, without needing to publish messages at a constant rate.  This can be useful for turning on/off LEDs, playing a sound etc.

Services can be defined in a package, however, in this exercise we will use preexisting service structures.  Check out the [ROS Wiki Tutorial](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29) for details.

Using the std_srvs/Trigger service, create a simple service.  There is an example on [Github](https://github.com/DrAndyWest/IntroToROS).

Tasks to complete:

- [ ] Run the service node and make calls to it via terminal and GUI interfaces
- [ ] After reading the tutorials, write your own service to accept an incoming primative type (e.g. std_msgs/Int32) and return a string with some text and the received data (remember - changes to package.xml and CMake.txt will be necessary)

To make a service call via the terminal, use `rosservice list` to find the service of interest, then `rosservice call` using Tab completion to fill in the rest.

