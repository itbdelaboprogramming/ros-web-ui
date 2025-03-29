# MySQL Folder Monitor
Synchronize local folder to MySQL database

## How to Run
1. Make sure python 3.8.10 is installed
2. Clone this repository in ~/ directory
```bash
git clone https://github.com/itbdelaboprogramming/mysql-folder-monitor.git
```
3. cd to the repository folder
```bash
cd mysql-folder-monitor
```

4. Install requirements
```bash
pip install -r requirements.txt
```

5. Create a config.py file
```bash
# create a config.py file inside mysql-folder-monitor directory
nano config.py

# add this line, change and fill accordingly
DIRECTORY_TO_WATCH = "/path/to/directory/to/watch"
MYSQL_HOST = ""
MYSQL_DATABASE = ""
MYSQL_USER = ""
MYSQL_PASS = ""
LOCAL_TIMEZONE = ""    # UTC or Asia/Jakarta or Asia/Tokyo            
CONVERT_TIMEZONE = ""  # time to be converted (UTC or Asia/Jakarta or Asia/Tokyo)
# save and exit
ctrl + s
ctrl + x
```



6. Run the program in Systemd background service (auto on when computer starts)
```bash
# open mysql_folder_monitor.service and change <username>
nano mysql_folder_monitor.service

# save and exit
ctrl + s
ctrl + x

# then follow these steps:
sudo cp mysql-folder-monitor.service /etc/systemd/system/   # copy service file
sudo systemctl daemon-reload                                # reload daemon
sudo systemctl enable mysql-folder-monitor.service          # auto on
sudo systemctl start mysql-folder-monitor.service           # start service
sudo systemctl status mysql-folder-monitor.service          # check status, make sure it's active (running)
```