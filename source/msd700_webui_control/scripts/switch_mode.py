#!/usr/bin/env python3
import rospy
import subprocess
from msd700_webui_msg.srv import SwitchMode, SwitchModeResponse
from msd700_webui_msg.msg import SwitchModeMsg

current_launch = None
current_simulation = False

def is_provided(s):
    """
    Mengembalikan True jika nilai s valid:
      - Untuk string: valid jika tidak kosong (setelah di-strip).
      - Untuk boolean: jika tidak None.
    """
    if s is None:
        return False
    if isinstance(s, str):
        return s.strip() != ""
    return True

def build_roslaunch_command(mode_msg):
    """
    Menyusun roslaunch berdasarkan mode dan parameter.
    
    Untuk mode 'navigation':
      - Jika map_file dan point_mode disediakan, maka :
          roslaunch msd700_navigations msd700_navigation.launch map_file:={MAP_FILE}.yaml point_mode:={POINT_MODE} use_simulator:=<value>
      - Jika tidak, maka gunakan parameter :
          roslaunch msd700_navigations msd700_navigation.launch open_rviz:=<value> use_simulator:=<value>
    Untuk mode 'slam' dan 'explore', perintahnya:
          roslaunch msd700_navigations msd700_slam.launch (atau explore) open_rviz:=<value> use_simulator:=<value>
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
        rospy.logerr("Mode tidak valid untuk membangun perintah: %s", mode_msg.mode)
        return None, None

    # Susun command list: ["roslaunch", package_name, launch_file_name, arg1, arg2, ...]
    cmd_list = ["roslaunch", package_name, launch_file_name] + arg_list
    cmd_display = "roslaunch {} {} {}".format(package_name, launch_file_name, " ".join(arg_list))
    return cmd_list, cmd_display

def start_launch(mode_msg):
    """
    Menjalankan proses roslaunch berdasarkan parameter pada mode_msg.
    Menentukan status simulasi (current_simulation) berdasarkan use_simulator.
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

    rospy.loginfo("Akan menjalankan instruksi: %s", cmd_display)
    
    try:
        process = subprocess.Popen(cmd_list)
        rospy.loginfo("Launch file untuk mode '%s' dijalankan.", mode_msg.mode)
        return process
    except Exception as e:
        rospy.logerr("Gagal menjalankan roslaunch: %s", str(e))
        return None

def kill_simulation_processes():
    """
    Menjalankan perintah 'killall gzserver gzclient' untuk membersihkan proses simulator.
    """
    try:
        subprocess.call(["killall", "gzserver", "gzclient"])
        rospy.loginfo("Perintah killall gzserver gzclient dijalankan.")
    except Exception as e:
        rospy.logerr("Gagal menjalankan killall: %s", str(e))

def shutdown_current_launch():
    """
    Menghentikan proses roslaunch yang sedang aktif.
    Jika simulasi aktif, setelah proses dihentikan juga memanggil killall untuk simulator.
    """
    global current_launch, current_simulation
    if current_launch is not None:
        rospy.loginfo("Mematikan launch file yang sedang berjalan.")
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
    Memproses perintah switching:
      - Jika mode di-set ke 'idle', maka hanya mematikan proses yang aktif (tanpa menjalankan perintah roslaunch).
      - Untuk mode lain, mematikan proses yang ada lalu menjalankan roslaunch baru.
    """
    global current_launch
    # Mode idle: matikan semua proses dan kembali idle
    if mode_msg.mode.strip().lower() == "idle":
        shutdown_current_launch()
        rospy.loginfo("Mode 'idle' dijalankan: Semua proses dimatikan, sistem dalam keadaan idle.")
        return True
    else:
        shutdown_current_launch()
        current_launch = start_launch(mode_msg)
        return current_launch is not None

def service_callback(req):
    if process_switch(req.mode_msg):
        message = "Berhasil switch ke mode: {}".format(req.mode_msg.mode)
        return SwitchModeResponse(success=True, message=message)
    else:
        return SwitchModeResponse(success=False, message="Gagal switch mode.")

def topic_callback(msg):
    if process_switch(msg):
        rospy.loginfo("Berhasil switch ke mode '%s' via topik.", msg.mode)
    else:
        rospy.logerr("Gagal switch ke mode '%s' via topik.", msg.mode)

def main():
    rospy.init_node('switch_mode_node')
    
    # Inisiasi service dengan custom service SwitchMode
    service = rospy.Service('switch_mode', SwitchMode, service_callback)
    # Inisiasi subscriber ke topik dengan tipe SwitchModeMsg
    rospy.Subscriber('switch_mode_topic', SwitchModeMsg, topic_callback)
    
    rospy.loginfo("Node switch_mode_node stanby")
    
    rospy.spin()
    shutdown_current_launch()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
