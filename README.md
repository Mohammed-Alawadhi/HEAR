# HEAR
Flight controller code

Installing PX4 Dependinces from HEAR
1. Install Ubuntu 18.04.5: http://old-releases.ubuntu.com/releases/bionic/ubuntu-18.04.5-desktop-amd64.iso
2. Run Command: $sudo apt-get update
3. Install git & curl using commands: $sudo apt-get install git, $sudo apt-get install curl
4. Install ROS Melodic Moreia: http://wiki.ros.org/melodic/Installation/Ubuntu
5. Uncheck http://packages.microsoft.com/repo/code stable main from Software & Updates
6. Fork HEAR to your github from https://github.com/AhmedHumais/HEAR
7. git clone https://github.com/"YourUserName"/HEAR
8. cd HEAR
9. git checkout hear_px4
10. git submodule update --init --recursive
11. cd HEAR_px4/PX4.....
12. git checkout 342e0da7961f9a3301706ed3835cfc163b14b2ed
13. $cd Tools/setup
14. $sudo bash ubuntu.sh

Note: This could help building sapog Firmware.
