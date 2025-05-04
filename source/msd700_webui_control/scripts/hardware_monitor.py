#!/usr/bin/env python3
import rospy
import yaml
import subprocess
import threading
import time
import os
import signal
from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Trigger, TriggerResponse
from rostopic import get_topic_class
from rospy.exceptions import ROSException

class HardwareMonitor:
    def __init__(self):
        rospy.init_node("hardware_monitor")
        
        # Log Node is running
        rospy.loginfo("Hardware Monitor Node is running...")

        # State for shutdown & timers
        self.shutting_down = False
        self.timers = []           # List of threading.Timer

        # Parameters and configuration path
        self.config_path = rospy.get_param(
            "~config_file",
            "/home/farhan-sw/Documents/GitHub/ros-web-ui-ws/src/ros-web-ui/source/msd700_webui_control/config/hardware_list.yaml"
        )
        self.is_start_all = rospy.get_param("~is_start_all_on_startup", True)
        
        # Log all monitoring hardware
        rospy.loginfo(f"Start all hardware on startup: {self.is_start_all}")
        rospy.loginfo("Hardware list:")
        with open(self.config_path, 'r') as f:
            cfg = yaml.safe_load(f)
        for name, hardware in cfg.get("hardware", {}).items():
            rospy.loginfo(f"  - {name}: {hardware['launch_file']} (topic: {hardware['topic']})")

        # Load hardware configuration
        self.hardware_configs = self._load_config(self.config_path)

        # States for processes, timestamps, and monitor threads
        self.processes = {}
        self.last_msg_times = {}
        self.threads = {}

        # Register services
        rospy.Service("/hardware_node/shutdown_all_hardware", SetBool, self.shutdown_all)
        rospy.Service("/hardware_node/init_all",      SetBool, self.init_all)
        rospy.Service("/hardware_node/check_hardware", Trigger, self.check_hardware)

        # CTRL+C
        signal.signal(signal.SIGINT, self._on_sigint)

        # Start all hardware if needed
        if self.is_start_all:
            self.start_all()

        rospy.on_shutdown(self.shutdown_all)

    def _on_sigint(self, signum, frame):
        rospy.loginfo("SIGINT received, initiating shutdown.")
        rospy.signal_shutdown("SIGINT")

    def _load_config(self, path):
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)
        return cfg.get("hardware", {})

    def start_all(self):
        rospy.loginfo("Starting all hardware...")
        for name in self.hardware_configs:
            self.start_hardware(name)

    def shutdown_all(self, req=None):
        """Stop all processes & cancel timers."""
        if self.shutting_down:
            # Already in the process of shutting down
            if req:
                return SetBoolResponse(success=True, message="Shutdown in progress.")
            return

        rospy.loginfo("Shutting down all hardware and cancelling timers...")
        self.shutting_down = True

        # Cancel all retry timers
        for t in self.timers:
            t.cancel()
        self.timers.clear()

        # Stop all hardware
        for name in list(self.processes.keys()):
            self._stop_process(name)

        if req:
            return SetBoolResponse(success=True, message="All hardware stopped.")

    def init_all(self, req):
        """Restart all hardware: shutdown then start."""
        self.shutdown_all()
        # Reset flag to allow restart
        self.shutting_down = False
        time.sleep(1.0)
        self.start_all()
        return SetBoolResponse(success=True, message="All hardware restarted.")

    def check_hardware(self, req):
        """Ensure each hardware process is alive and data is fresh."""
        failed = []
        now = rospy.get_rostime()
        for name, cfg in self.hardware_configs.items():
            proc = self.processes.get(name)
            last = self.last_msg_times.get(name)
            timeout = float(cfg.get("timeout", 5.0))

            alive = (proc and proc.poll() is None)
            fresh = (last and (now - last).to_sec() <= timeout)

            if not alive:
                failed.append(f"{name} (process dead)")
            elif not fresh:
                failed.append(f"{name} (no data >{timeout}s)")

        if failed:
            msg = "Hardware check failed: " + "; ".join(failed)
            rospy.logwarn(msg)
            return TriggerResponse(success=False, message=msg)
        return TriggerResponse(success=True, message="All hardware running and streaming data.")

    def start_hardware(self, name):
        """Launch the hardware, verify its startup, and begin monitoring."""
        # Do not schedule if already shutting down
        if self.shutting_down:
            return

        # If already exists, stop it first
        if name in self.processes:
            self._stop_process(name)

        cfg     = self.hardware_configs[name]
        topic   = cfg["topic"]
        timeout = float(cfg.get("timeout", 5.0))
        cmd     = ["roslaunch"] + cfg['launch_file'].split()
        log_fn  = f"/tmp/{name}.log"

        # Ensure directory & log file exist (mode a+)
        os.makedirs(os.path.dirname(log_fn), exist_ok=True)
        log_f = open(log_fn, 'a+')

        rospy.loginfo(f"[{name}] Launching: {' '.join(cmd)}  (logs→{log_fn})")
        proc = subprocess.Popen(
            cmd,
            stdout=log_f,
            stderr=log_f,
            preexec_fn=os.setsid
        )
        self.processes[name] = proc
        self.last_msg_times[name] = None

        # Check for early crash after 2 seconds
        time.sleep(2.0)
        if proc.poll() is not None:
            rospy.logerr(f"[{name}] Process exited immediately. See log for details.")
            log_f.flush(); log_f.seek(0)
            lines = log_f.readlines()
            for line in lines[-10:]:
                rospy.logerr(f"[{name} LOG] {line.strip()}")
            log_f.close()
            # Schedule restart if still running
            if not self.shutting_down:
                t = threading.Timer(timeout, lambda: self.start_hardware(name))
                self.timers.append(t); t.start()
            return

        # Wait until topic appears
        start_t = time.time()
        msg_cls = None
        while time.time() - start_t < timeout and not rospy.is_shutdown():
            msg_cls, _, _ = get_topic_class(topic)
            if msg_cls:
                break
            time.sleep(0.5)

        if not msg_cls:
            rospy.logwarn(f"[{name}] Topic '{topic}' not found within {timeout}s.")
            self._terminate_process_group(proc)
            log_f.close()
            if not self.shutting_down:
                t = threading.Timer(timeout, lambda: self.start_hardware(name))
                self.timers.append(t); t.start()
            return

        # Wait for the first message
        try:
            rospy.loginfo(f"[{name}] Waiting for the first message on '{topic}'...")
            rospy.wait_for_message(topic, msg_cls, timeout=timeout)
            self.last_msg_times[name] = rospy.get_rostime()
            rospy.loginfo(f"[{name}] Topic active, entering monitor loop.")
            self._start_monitor(name, topic, timeout)
        except ROSException:
            rospy.logwarn(f"[{name}] No message on '{topic}' within {timeout}s.")
            self._terminate_process_group(proc)
            log_f.close()
            if not self.shutting_down:
                t = threading.Timer(timeout, lambda: self.start_hardware(name))
                self.timers.append(t); t.start()
            return

        log_f.close()

    def _terminate_process_group(self, proc):
        """SIGTERM → SIGKILL to process group."""
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGTERM)
            time.sleep(1.0)
            os.killpg(pgid, signal.SIGKILL)
        except Exception as e:
            rospy.logwarn(f"Error terminating process group: {e}")

    def _stop_process(self, name):
        """Terminate process group and cleanup state."""
        proc = self.processes.get(name)
        if proc and proc.poll() is None:
            rospy.loginfo(f"[{name}] Terminating process group.")
            self._terminate_process_group(proc)
        self.processes.pop(name,    None)
        self.last_msg_times.pop(name, None)

    def _start_monitor(self, name, topic, timeout):
        """Subscriber + thread to restart if timeout."""
        def cb(msg):
            self.last_msg_times[name] = rospy.get_rostime()

        msg_cls, _, _ = get_topic_class(topic)
        rospy.Subscriber(topic, msg_cls, cb)

        def loop():
            while not rospy.is_shutdown() and not self.shutting_down:
                last = self.last_msg_times.get(name)
                if last and (rospy.get_rostime() - last).to_sec() > timeout:
                    rospy.logwarn(f"[{name}] Data timeout (> {timeout}s). Restarting...")
                    self.start_hardware(name)
                    return
                time.sleep(1.0)

        th = threading.Thread(target=loop)
        th.daemon = True
        th.start()
        self.threads[name] = th

if __name__ == "__main__":
    try:
        HardwareMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
