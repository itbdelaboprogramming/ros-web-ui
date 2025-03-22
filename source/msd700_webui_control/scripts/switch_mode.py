#!/usr/bin/env python3
import rospy
import subprocess
from msd700_webui_msg.srv import SwitchMode, SwitchModeResponse
from msd700_webui_msg.msg import SwitchModeMsg

current_launch = None
current_simulation = False

def is_provided(s):
    """
    Returns True if the value of s is valid:
      - For strings: valid if not empty (after stripping).
      - For booleans: valid if not None.
    """
    if s is None:
        return False
    if isinstance(s, str):
        return s.strip() != ""
    return True

def build_roslaunch_command(mode_msg):
    """
    Construct roslaunch command based on mode and parameters.
    
    For 'navigation' mode:
      - If map_file and point_mode are provided, then:
          roslaunch msd700_navigations msd700_navigation.launch map_file:={MAP_FILE}.yaml point_mode:={POINT_MODE} use_simulator:=<value>
      - If not, use the parameters:
          roslaunch msd700_navigations msd700_navigation.launch open_rviz:=<value> use_simulator:=<value>
    For 'slam' and 'explore' modes, the command is:
          roslaunch msd700_navigations msd700_slam.launch (or explore) open_rviz:=<value> use_simulator:=<value>
    """
    package_name = "msd700_navigations"
    arg_list = []
    mode_lower = mode_msg.mode.strip().lower()
    
    if mode_lower == "navigation":
        launch_file_name = "msd700_navigation.launch"
        if is_provided(mode_msg.map_file) and is_provided(mode_msg.point_mode):
            arg_list.append("map_file:={}.yaml".format(mode_msg.map_file.strip()))
            arg_list.append("point_mode:={}".format(mode_msg.point_mode.strip()))
        else:
            if is_provided(mode_msg.open_rviz):
                arg_list.append("open_rviz:={}".format(str(mode_msg.open_rviz).lower()))
        if is_provided(mode_msg.use_simulator):
            arg_list.append("use_simulator:={}".format(str(mode_msg.use_simulator).lower()))
    elif mode_lower == "slam":
        launch_file_name = "msd700_slam.launch"
        if is_provided(mode_msg.open_rviz):
            arg_list.append("open_rviz:={}".format(str(mode_msg.open_rviz).lower()))
        if is_provided(mode_msg.use_simulator):
            arg_list.append("use_simulator:={}".format(str(mode_msg.use_simulator).lower()))
    elif mode_lower == "explore":
        launch_file_name = "msd700_explore.launch"
        if is_provided(mode_msg.open_rviz):
            arg_list.append("open_rviz:={}".format(str(mode_msg.open_rviz).lower()))
        if is_provided(mode_msg.use_simulator):
            arg_list.append("use_simulator:={}".format(str(mode_msg.use_simulator).lower()))
    else:
        rospy.logerr("SWITCH MODE || Invalid mode to construct the command: %s", mode_msg.mode)
        return None, None

    # Susun command list: ["roslaunch", package_name, launch_file_name, arg1, arg2, ...]
    cmd_list = ["roslaunch", package_name, launch_file_name] + arg_list
    cmd_display = "roslaunch {} {} {}".format(package_name, launch_file_name, " ".join(arg_list))
    return cmd_list, cmd_display

def start_launch(mode_msg):
    """
    Runs the roslaunch process based on the parameters in mode_msg.
    Determines the simulation status (current_simulation) based on use_simulator.
    """
    global current_simulation
    # Tentukan apakah simulasi aktif
    current_simulation = False
    if is_provided(mode_msg.use_simulator):
        if isinstance(mode_msg.use_simulator, bool):
            current_simulation = mode_msg.use_simulator
        else:
            if mode_msg.use_simulator.strip().lower() == "true":
                current_simulation = True

    cmd_list, cmd_display = build_roslaunch_command(mode_msg)
    if cmd_list is None:
        return None

    rospy.loginfo("SWITCH MODE || Executing command: %s", cmd_display)
    
    try:
        process = subprocess.Popen(cmd_list)
        rospy.loginfo("SWITCH MODE || Launch file for mode '%s' has been started.", mode_msg.mode)
        return process
    except Exception as e:
        rospy.logerr("SWITCH MODE || Failed to execute roslaunch: %s", str(e))
        return None

def kill_simulation_processes():
    """
    Executes the 'killall gzserver gzclient' command to clean up simulator processes.
    """
    try:
        subprocess.call(["killall", "gzserver", "gzclient"])
        rospy.loginfo("SWITCH MODE || Command 'killall gzserver gzclient' executed.")
    except Exception as e:
        rospy.logerr("SWITCH MODE || Failed to execute killall: %s", str(e))

def shutdown_current_launch():
    """
    Stops the currently active roslaunch process.
    If a simulation is active, after stopping the process, it also calls killall for the simulator.
    """
    global current_launch, current_simulation
    if current_launch is not None:
        rospy.loginfo("SWITCH MODE || Shutting down the currently running launch file.")
        current_launch.terminate()
        try:
            current_launch.wait(timeout=5)
        except Exception:
            current_launch.kill()
        current_launch = None
    if current_simulation:
        kill_simulation_processes()
        current_simulation = False

def process_switch(mode_msg):
    """
    Processes the switching command:
      - If the mode is set to 'idle', it only stops the active process (without running a new roslaunch command).
      - For other modes, it stops the existing process and then starts a new roslaunch command.
    """
    global current_launch
    # Mode idle: matikan semua proses dan kembali idle
    if mode_msg.mode.strip().lower() == "idle":
        shutdown_current_launch()
        rospy.loginfo("SWITCH MODE || Mode 'idle' executed: All processes stopped, system is now idle.")
        return True
    else:
        shutdown_current_launch()
        current_launch = start_launch(mode_msg)
        return current_launch is not None

def service_callback(req):
    if process_switch(req.mode_msg):
        message = "Successfully switched to mode: {}".format(req.mode_msg.mode)
        return SwitchModeResponse(success=True, message=message)
    else:
        return SwitchModeResponse(success=False, message="Gagal switch mode.")

def topic_callback(msg):
    if process_switch(msg):
        rospy.loginfo("SWITCH MODE || Successfully switched to mode '%s' via topic.", msg.mode)
    else:
        rospy.logerr("SWITCH MODE || Failed to switch to mode '%s' via topic.", msg.mode)

def main():
    rospy.init_node('switch_mode_node')
    
    # Parameter setting
    service_name = rospy.get_param('~service_name', 'switch_mode')
    topic_name = rospy.get_param('~topic_name', 'switch_mode_topic')

    # Initialize service with service_callback
    service = rospy.Service(service_name, SwitchMode, service_callback)
    # Initialize subscriber to the topic with type SwitchModeMsg
    rospy.Subscriber(topic_name, SwitchModeMsg, topic_callback)
    
    # Logger Detailed Info
    rospy.loginfo("Switch Mode Node initialized with:")
    rospy.loginfo("     Service name: %s", service_name)
    rospy.loginfo("     Topic name: %s", topic_name)
    
    rospy.spin()
    shutdown_current_launch()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
