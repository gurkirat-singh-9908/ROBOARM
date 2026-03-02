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
