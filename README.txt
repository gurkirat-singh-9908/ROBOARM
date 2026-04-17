Setting up the headless pi setup
flash the required sever image 

connect with pi  
ssh <hostname>@<username>.local
password - 

tailscale setup 
install tailscale on windows and Linux
 
for ubuntu 
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

visit the site 
and connect the device 

connect to pi
ssh <usrname>@<newtailscaleprovidedip>

 
setting up workspace 
install github cli 
sudo apt install gh
login git 
gh auth login

copy git repo
git clone https://github.com/gurkirat-singh-9908/ROBOARM.git

install ros2 jazzy 
follow - https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
chose base ros instead of desktop 

install moveit 
sudo apt install ros-jazzy-moveit

delete the log, install and build folders in ros dir 
then build the workspace 
colcon build



get all the ros topics and nodes on the host 
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
