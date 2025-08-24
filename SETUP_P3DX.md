
## SETUP_P3DX.md  (your “how I set up, installed and ran” guide)
# Pioneer 3-DX Setup & Install Notes

> This is the reproducible log of how I set up the robots for use.

## 1) Base OS Setup & networking for Raspberry Pi 4
### Flash Ubuntu 20.04 64-bit (Noetic target) to microSD via Raspberry Pi imager. 
#### Linked guides may help to visualize the following steps 
 1. [The Ultimate Guide to Get Started with Ubuntu on Raspberry Pi](https://raspberrytips.com/install-ubuntu-desktop-raspberry-pi/)
 2. [How to install Ubuntu Server on your Raspberry Pi]()
- Install [Raspberry Pi Imager](https://www.raspberrypi.com/software/) if not installed.
- Insert microSD  
- Open Raspberry Pi Imager
  + Choose Device : Raspberry Pi 4
  + Choose OS: Ubuntu 20.04 from other general-purpose OS
  + Select Storage
  + Click Next. There will be a prompt asking to edit settings. Give the username (preferably `Pi` ), password, and internet connection details. After this, install the OS onto the Micro SD.
- Once installed, remove the SD card and insert it into the Raspberry Pi.
- Connect external monitor, keyboard, mouse. (connect ethernet if wifi is not being used)
- Power Raspberry Pi on using power supply. There should be a red light which is the power LED, and a green light flashing indicating booting up and SD card activity respectively. 
  + From here, your experience will be different whether you are using Ubuntu Server or Ubuntu Desktop
    + If ubuntu desktop installed automatically by imager then you can easily login to Pi just like regular desktop and [configure wifi network](https://linuxconfig.org/setup-wireless-interface-with-wpa-and-wpa2-on-ubuntu) following the [guide.](https://raspberrytips.com/install-ubuntu-desktop-raspberry-pi/) Otherwise you need to install the desktop environment separately.  
    + If Ubuntu server boots up follow this instruction to setup wifi. Sometimes the structure of the network-config file that you modified before boot gets messed up when you boot. This would cause your Pi not to be able to connect to your wi-fi. Follow section `4` and `5` of [this guide.](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#4-boot-ubuntu-server). Install `GNOME` as desktop environment. 
    + Once installed you will be directed to entering your password and then the desktop screen.
  + Fix date and time if needed from settings
- [Optional Step] Set Static IP Address for Pi following any of these guide – 
  +	[Ubuntu Desktop - How to Configure Static IP Address on Ubuntu 20.04 | Linuxize](https://linuxize.com/post/how-to-configure-static-ip-address-on-ubuntu-20-04/) 
  +	[Ubuntu server How to Set Static IP Address on Ubuntu Server (Step-by-step) – RaspberryTips](https://raspberrytips.com/set-static-ip-address-ubuntu-server/)

- Open terminal window. Press `ctrl+alt+T` 

## 2) ROS Noetic + tools
Two ways to install noetic
- Recommended: [Ubuntu install of ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
  + Complete step 1.2, 1.3, 1.4 (desktop-full install), 1.5 (for bash), 1.6.1
  + One common error you may face using `sudo` command for the first time `user is not in the sudoers file`. This means that your current, logged on user profile for your installation is not able to use commands with root/sudo access. To solve this issue, type `su -` to log in as the root user. The password is the same password you used to setup the Ubuntu. You should now be signed in as the root user. Now, type `sudo adduser [your username] sudo` and then restart.
- [One Line Setup Tutorial](https://www.youtube.com/watch?v=IqrpSi2Xueg) (You may face issues)
- Verify your installation
  + Type and Enter `rosversion -d`. You will see `noetic`. Type `roscore`. You will see ros is running. Open another terminal and type `rosrun roscpp_tutorials talker` 

## 3) ROSARIA (driver) installation
  ARIA Library installation
  ``` bash 
  sudo apt install libaria-dev
  ```
  Make new directories
  ``` bash 
  mkdir -p ~/catkin_es/src
  ```
  Go into new workspace's source
  ``` bash 
  cd ~/catkin_ws/src
  ``` 
  Clone ROSARIA 
   ``` bash 
  git clone https://github.com/amor-ros-pkg/rosaria.git 
  ``` 
  Build the workspace
  ``` bash 
  cd ~/catkin_ws/
  catkin_make
  ``` 
  Source your new workspace
  ``` bash 
  echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```
  Verify the build
  ``` bash 
   rospack list | grep rosaria
  ```
  You should see something like this
  ```` bash
  rosaria /home/<username>/catkin_ws/src/rosaria
  ````
If you do not see something like this, try again after restarting Pi.
Congratulations! You have setup Raspberry Pi to run the robot. To run the robot from remote computer you need to setup `Ubuntu 20.04/Linux` your own PC. I used virtual box to run `Ubuntu 20.04/Linux` in my PC. You can use `Linux` based PC.
## 4) SSH frome Virtual Machine
- Install virtual box and Ubuntu 20.04 LTS on VirtualBox by following [this tutorial.](https://www.youtube.com/watch?v=x5MhydijWmc)
- Ensure Network Connectivity
  + Bridged Adapter in VirtualBox
    + In VirtualBox, go to Settings → Network → Adapter 1. Choose Bridged Adapter so the VM gets an IP address on the same subnet as the Pi.
- Find Pi's IP Address 
  + If you have a monitor and keyboard connected on the Pi, run `ip -a`.  Or check your router’s DHCP client list to see which IP was assigned to the Pi and other connected devices. So it's a good idea to set Static IP Address for Pi as mentioned in first section.
- Boot `ubuntu` in virtual machine, open a terminal, and verify you can ping the Pi’s IP address. Type and Enter 
```` bash
ping <Pis_IP_Address>
````
- If it responds, networking is set up correctly. SSH from Your VM into the Pi. 
```  bash
ssh <Pi_username>@<Pi_IP_Address>
```
Enter the Pi’s password when prompted. Now you’re inside the Pi’s shell, but you’re controlling it from remote PC wirelessly. You may or may not see following error 

![Cannot SSH](C:\Users\sbishal\Documents\GitHub\p3dx-lab-toolkit\Picture1.png?crop=1.00xw:0.753xh;0,0.153xh&resize=1200:*)

Type following to remove all keys belonging to hostname
```` bash
ssh-keygen -R <Pi_ip>
````
- Setup Passwordless SSH: Typing passwords for each SSH call can be tedious. You can use SSH keys so the VM can log in to each Pi without a password prompt 
```` bash
ssh-keygen
ssh-copy-id <PI_username>@<Pi_IP_Address>
````



## 5) Run Robot
Disconnect Pi's keyboard/mouse and wired power source. Connect powerbank or UPS power supply and RS-232 on the Pi. The pioneer typically has an RS-232 D89 port. Use a USB-to-Serial adapter on the Pi. Thus you connect Pi to P3DX.
Many adapters show up as `/dev/ttyUSB0` or `/dev/ttyACM0`
Ensure you have the correct null modem cable. Check if you have SSH'd already. SSH from Your VM into the Pi. 
```  bash
ssh <Pi_username>@<Pi_IP_Address>
```
And type
```` bash
ls /dev/ttyUSB*
dmesg | grep ttyUSB
````
You should see something like `/dev/ttyUSB0`. 
>Switch on the Robot. 
Now launch the `ROSARIA` driver on pi by typing the following. Before that you must run roscore in a separate terminal if not launched before. 
> But for every new terminal you need to open another SSH session
```  bash
ssh <Pi_username>@<Pi_IP_Address>
```
In one terminal, type and enter
```bash
roscore
````
In second terminal, SSH and type
```` bash
rosrun rosaria RosAria _port:=/dev/ttyUSB0
````
You will hear a noise if driver is connected to the robot. Run your script or do teleoperation first if you haven't written your script. For teleoperation, install teleoperation package first. Open a new terminal and SSH. Type
```bash 
sudo apt-get install ros-noetic-teleop-twist-keyboard
````
Run teleop node
```` bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/RosAria/cmd_vel
````
Run your own script
``` bash 
python3 myscript.py
```
You can download and run my scripts.
``` bash
cd ~
git clone https://github.com/<salim-bishal>/p3dx-lab-toolkit.git
cd p3dx-lab-toolkit
chmod +x scripts/*.py
```
Type 
``` bash
python3 pkg_del.py
```
Try other scripts also. 




     
