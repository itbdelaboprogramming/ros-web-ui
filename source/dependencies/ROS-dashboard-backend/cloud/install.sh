#!/bin/bash
# Install tmux
sudo apt install -y tmux

# Install zerotier
curl -s 'https://raw.githubusercontent.com/zerotier/ZeroTierOne/master/doc/contact%40zerotier.com.gpg' | gpg --import && \
if z=$(curl -s 'https://install.zerotier.com/' | gpg); then echo "$z" | sudo bash; fi

# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update -y
sudo apt install -y ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update

# Install Node.js backend server on ROS
tmux new-session -d -s 'install'
tmux send-keys "mkdir -p catkin_ws/src" C-m
tmux send-keys "cd ~/catkin_ws/src" C-m
tmux send-keys "git clone https://github.com/itbdelaboprogramming/ROS-dashboard-backend" C-m
tmux send-keys "cd ~/catkin_ws" C-m
tmux send-keys "catkin_make" C-m
tmux send-keys 'echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc' C-m
tmux send-keys "source ~/.bashrc" C-m
tmux send-keys "sudo apt install -y ros-noetic-rosbridge-server" C-m
tmux send-keys "exit" C-m

# Install Folder Monitor to MySQL (Folder-MySQL Synchronizer)
cd ~
git clone https://github.com/itbdelaboprogramming/mysql-folder-monitor.git
cd mysql-folder-monitor
sudo apt install -y python3-pip
pip install -r requirements.txt
# next set up config manually

# Install Mosquitto Broker
cd ~
sudo apt -y install mosquitto mosquitto-clients
# next setup config manually

# Install nodejs using nvm
tmux new-session -d -s 'install'
tmux send-keys "cd ~" C-m
tmux send-keys "curl https://raw.githubusercontent.com/creationix/nvm/master/install.sh | bash" C-m
tmux send-keys "source ~/.bashrc" C-m
tmux send-keys "nvm install 18.18.2" C-m

# Install frontend
tmux send-keys "cd ~" C-m
tmux send-keys "git clone https://github.com/itbdelaboprogramming/ROS-dashboard-next-ts" C-m
tmux send-keys "cd ~/ROS-dashboard-next-ts" C-m
tmux send-keys "npm install" C-m
tmux send-keys "npm install pm2 -g" C-m
tmux send-keys "exit" C-m
# next setup config manually

# Install nginx server to port forward ROS dashboard backend
cd ~
sudo apt install -y nginx
# next setup nginx port forwarding manually

# Install MySQL Client to access DB
sudo apt install -y mysql-client

# Print manually config
cd ~
echo "Please set up ZeroTier Network ID manually"
echo "Please set up MySQL Folder Monitor config manually"
echo "Please set up Mosquitto Broker config manually"
echo "Please set up frontend next ts config manually"
echo "Please set up nginx port forwarding config manually"

# Background installation might be still on going, attach tmux session
tmux attach-session -t 'install'
