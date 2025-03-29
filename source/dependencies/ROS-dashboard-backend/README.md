# ROS Dashboard Backend

Backend service for ROS Dashboard. This service is responsible for handling HTTP REST API requests from the frontend and also handling ROS communication with the robot.

## Installation
1. The backend application requires some application from SLAM_ITBdeLabo ROS Package. Please install SLAM_ITBdeLabo ros pacakge first. Refer to this [link](https://github.com/itbdelaboprogramming/SLAM_ITBdeLabo) for installation guide.

2. Clone the repository. Please make sure that catkin_ws directory is already prepared. Also make sure Node.js is installed. Version that is used for development is v18.18.2. If there are issues, refer to this version (v18.18.2).
```bash
sudo apt install -y ros-noetic-rosbridge-server
cd ~/catkin_ws/src
git clone https://github.com/itbdelaboprogramming/ROS-dashboard-backend.git
cd ~/catkin_ws
catkin_make
```

3. Install MySQL server and set root password
```bash
sudo apt install mysql-server
sudo mysql
ALTER USER 'root'@'localhost' IDENTIFIED WITH mysql_native_password BY 'root';
FLUSH PRIVILEGES;
exit
```

4. Set user and password for remote access from Jetson through zerotier
```bash
# expose mysql to all network interfaces
sudo nano /etc/mysql/mysql.conf.d/mysqld.cnf
# find bind-address and add another bind-address = "0.0.0.0"
# save and exit
ctrl + s
ctrl + x

# add new accounts for accesss from Jetson/remote pc
mysql -u root -p
# enter root password, in this case from previous step is 'root'
# enter these commands, ONE BY ONE separated by ";" (without quotes)
CREATE USER 'jetson_username'@'jetson_zerotier_ip_address' IDENTIFIED BY 'jetson_password';
GRANT ALL PRIVILEGES ON *.* TO 'jetson_username'@'jetson_zerotier_ip_address' WITH GRANT OPTION;
FLUSH PRIVILEGES;
exit

# now that user can access the database from remote/other pc assigned with that IP address.
```

5. Prepare database
```bash
mysql -u root -p
# enter root password, in this case from previous step is 'root'
# copy all the commands from sql/init.sql ONE BY ONE separated by ";" (without quotes)
```
6. Fill all configs in .env file
```bash
cd ~/catkin_ws/src/ROS-dashboard-backend/scripts
cp .env.template .env
nano .env

# Fill all configs
MYSQL_HOST=""
MYSQL_USER=""
MYSQL_PASSWORD=""
MYSQL_DATABASE=""
MAP_PATH="/home/<pc_user_here>/catkin_ws/src/ROS-dashboard-backend/map"
MQTT_BROKER_IP=""
TIMEZONE=7          # UTC+(timezone), change according to your timezone
JWT_SECRET_KEY=""       # fill with random string

# save and exit
ctrl + s
ctrl + x
```

7. Install dependencies (Node.js v18.18.2)
```bash
cd ~/catkin_ws/src/ROS-dashboard-backend
npm install
```

8. Install mysql folder monitor to synchronize map folder with database. Follow this step: https://github.com/itbdelaboprogramming/mysql-folder-monitor

9. Run
```bash
cd ~/catkin_ws
roslaunch ros_dashboard_backend ros_dashboard_backend.launch
```

10. The app will run at all network interfaces (0.0.0.0) on port 5000.

## Postman
Postman collection and environment filees are provided to test the HTTP REST API using Postman. Please import the files to Postman. Please also update the postman files whenever there are changes in the API.

## Cloud Server Installation (Optional)
There is a shell script to install required software on cloud VM in `/cloud/install.sh` file. The script is for Ubuntu 20.04 VM cloud server.