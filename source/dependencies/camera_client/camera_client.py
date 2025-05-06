# Code information
"""
Name            : Robo Client Camera Streamer
Version         : v0.1.0 (Beta 1)
Publiser        : Anas Fathurrahman
Description     : This code is an agent for robo's camera streaming. This code do connection 
                  between robo and client controller by contacting signalling server for do
                  handshake.

Dependencies    : 
    
    Python v3.10.12

        Standart library : 
                asyncio         (python standard library)
                json            v2.0.9
                logging         v0.5.1.2
                os              (python standard library)
                re              2.2.1
                typing          (python standard library)
        
        third party :
                websockets      v15.0.1
                cv2             v4.11.0
                numpy           v1.24.4
                aiortc          v1.10.1
                av              v13.1.0

ITB de Labo 2025 - Nakayama Inc.
"""


# library declaration
import asyncio
import json
import logging
import os
import cv2
import numpy as np
import websockets
import re
from logging.handlers import RotatingFileHandler
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, RTCConfiguration, RTCIceServer
from aiortc.mediastreams import VideoStreamTrack
from av import VideoFrame
from typing import Optional, Dict, Any, List

# Set up logging configuration with both file and console handlers
def setup_logging():
    log_level_name = os.environ.get('LOG_LEVEL', 'INFO').upper()
    log_level = getattr(logging, log_level_name, logging.INFO)
    
    log_dir = 'logs'
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    logger = logging.getLogger()
    logger.setLevel(log_level)
    
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
    
    log_format = logging.Formatter(
        '%(asctime)s - %(process)d - %(name)s - %(levelname)s - %(message)s'
    )
    
    file_handler = RotatingFileHandler(
        os.path.join(log_dir, 'camera_client.log'),
        maxBytes=10*1024*1024,
        backupCount=5
    )
    file_handler.setFormatter(log_format)
    logger.addHandler(file_handler)
    
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(log_format)
    logger.addHandler(console_handler)
    
    return logger

# Initialize logging
logger = setup_logging()
logger.info("Logging system initialized")

# --- Configuration ---
# DEFAULT_SERVER_URL = "ws://localhost:3001"
DEFAULT_SERVER_URL = "wss://delabo.asthaweb.com/signalling"
DEFAULT_STREAMER_ID = "AX591"
# Define multiple reliable STUN servers directly

SERVER_URL = os.environ.get("SERVER_URL", DEFAULT_SERVER_URL)
STREAMER_ID = os.environ.get("STREAMER_ID", DEFAULT_STREAMER_ID)

"""WebRTC video streaming client using aiortc and OpenCV."""
class VideoCamera(VideoStreamTrack):
    def __init__(self, error_callback=None):
        lowres = [640, 480]
        highres = [1280, 720]
        
        self.width = highres[0]
        self.height = highres[1]

        super().__init__()
        self.logger = logging.getLogger("VideoCamera")
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        
        self.fallback_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self._camera_released = False
        
        # Camera recovery parameters
        self.consecutive_failures = 0
        self.max_consecutive_failures = 5
        self.reboot_in_progress = False
        self.reboot_attempts = 0
        self.max_reboot_attempts = 3
        self.last_reboot_time = 0
        self.reboot_cooldown = 10  # seconds
        
        # Error notification callback
        self.error_callback = error_callback
        self.error_notification_sent = False

        # Fatal camera failure flag
        self.camera_fatally_failed = False
        # Add a message to the fallback frame
        cv2.putText(self.fallback_frame, "CAMERA ERROR", (int(self.width/2)-100, int(self.height/2)), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(self.fallback_frame, "Please check camera connection", (int(self.width/2)-170, int(self.height/2)+40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        # If camera is in fatal failure state, just return the fallback frame
        if self.camera_fatally_failed:
            video_frame = VideoFrame.from_ndarray(self.fallback_frame, format="rgb24")
            video_frame.pts = pts
            video_frame.time_base = time_base
            return video_frame

        ret, frame = self.cap.read()
        if not ret:
            self.consecutive_failures += 1
            
            # Send error notification if we hit the threshold and haven't notified yet
            if (self.consecutive_failures >= self.max_consecutive_failures and 
                self.error_callback and not self.error_notification_sent):
                error_info = {
                    "type": "camera_failure",
                    "severity": "error",
                    "message": f"Camera read failure after {self.consecutive_failures} attempts",
                    "attempts": self.consecutive_failures,
                    "max_attempts": self.max_consecutive_failures,
                    "recovery": "automatic reboot in progress"
                }
                asyncio.create_task(self.error_callback(error_info))
                self.error_notification_sent = True
            
            # If we've had multiple consecutive failures and we're not already rebooting
            if self.consecutive_failures >= self.max_consecutive_failures and not self.reboot_in_progress and not self.camera_fatally_failed:
                asyncio.create_task(self.reboot_camera())
            
            frame = self.fallback_frame
        else:
            # Reset consecutive failures on success
            if self.consecutive_failures > 0:
                self.logger.info(f"Camera reading recovered after {self.consecutive_failures} failures")
                # If we previously sent an error notification, send a recovery notification
                if self.error_notification_sent and self.error_callback:
                    recovery_info = {
                        "type": "camera_recovery",
                        "severity": "info",
                        "message": f"Camera recovered after {self.consecutive_failures} failures",
                        "previous_failures": self.consecutive_failures
                    }
                    asyncio.create_task(self.error_callback(recovery_info))
                self.consecutive_failures = 0
                self.error_notification_sent = False

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    async def reboot_camera(self):
        """Attempt to reinitialize the camera."""
        if self.reboot_in_progress or self.camera_fatally_failed:
            return
            
        current_time = asyncio.get_event_loop().time()
        if current_time - self.last_reboot_time < self.reboot_cooldown:
            return
            
        self.reboot_in_progress = True
        self.reboot_attempts += 1
        self.last_reboot_time = current_time
        
        self.logger.warning(f"Attempting camera reboot (attempt {self.reboot_attempts}/{self.max_reboot_attempts})")
        
        # Send reboot notification
        if self.error_callback:
            reboot_info = {
                "type": "camera_reboot",
                "severity": "warning",
                "message": f"Camera reboot attempt {self.reboot_attempts}/{self.max_reboot_attempts}",
                "attempt": self.reboot_attempts,
                "max_attempts": self.max_reboot_attempts
            }
            asyncio.create_task(self.error_callback(reboot_info))
        
        try:
            # Release current camera if it exists
            if not self._camera_released and hasattr(self, 'cap') and self.cap is not None:
                self.logger.info("Releasing current camera resources")
                self.cap.release()
                self._camera_released = True
                
            # Short delay to allow camera to fully release
            await asyncio.sleep(1)
            
            # Reinitialize camera
            self.logger.info("Reinitializing camera device")
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self._camera_released = False
            
            # Check if camera opened successfully
            if not self.cap.isOpened():
                self.logger.error("Failed to reopen camera")
                # If we've reached max attempts, log a critical error and mark camera as fatally failed
                if self.reboot_attempts >= self.max_reboot_attempts:
                    self.logger.critical(f"Maximum camera reboot attempts ({self.max_reboot_attempts}) reached. Camera might be disconnected or faulty.")
                    
                    # Mark camera as fatally failed to prevent further reboot attempts
                    self.camera_fatally_failed = True
                    
                    # Release camera resources
                    if not self._camera_released and hasattr(self, 'cap') and self.cap is not None:
                        self.cap.release()
                        self._camera_released = True
                    
                    # Send critical error notification
                    if self.error_callback:
                        critical_error = {
                            "type": "camera_failure_critical",
                            "severity": "critical",
                            "message": f"Camera failed after {self.max_reboot_attempts} reboot attempts",
                            "attempts": self.reboot_attempts,
                            "requires": "manual intervention",
                            "status": "terminated"
                        }
                        asyncio.create_task(self.error_callback(critical_error))
            else:
                self.logger.info("Camera successfully reinitialized")
                self.consecutive_failures = 0
                self.reboot_attempts = 0
        except Exception as e:
            self.logger.error(f"Error during camera reboot: {e}")
            # Send error notification for reboot failure
            if self.error_callback:
                error_info = {
                    "type": "camera_reboot_error",
                    "severity": "error",
                    "message": f"Error during camera reboot: {str(e)}",
                    "error": str(e)
                }
                asyncio.create_task(self.error_callback(error_info))
                
            # If we've reached max attempts, mark as fatally failed
            if self.reboot_attempts >= self.max_reboot_attempts:
                self.camera_fatally_failed = True
                self.logger.critical("Maximum reboot attempts reached. Camera stream terminated.")
        finally:
            self.reboot_in_progress = False

    def stop(self):
        if not self._camera_released and hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self._camera_released = True
            self.logger.info("Camera resources released by stop()")

    def __del__(self):
        if hasattr(self, '_camera_released') and not self._camera_released and hasattr(self, 'cap'):
            self.cap.release()
            self.logger.info("Camera resources released by destructor")

class CameraClient:
    try:
        token_path = os.environ.get("TOKEN_FILE_PATH", "token.cred")
        with open(token_path, 'r') as f:
            JWT_TOKEN = f.read().strip()
    except FileNotFoundError:
        logger.error(f"Error: Token file '{token_path}' not found")
        JWT_TOKEN = None
    except Exception as e:
        logger.error(f"Error reading token file '{token_path}': {e}")
        JWT_TOKEN = None

    def __init__(self, server_url: str = SERVER_URL, streamer_id: str = STREAMER_ID):
        self.STREAMER_ID = streamer_id
        self.CLIENT_ID: str = ""
        self.server_url = server_url
        self.websocket: Optional[websockets.WebSocketClientProtocol] = None
        self.peer_connection: Optional[RTCPeerConnection] = None
        self.is_streaming: bool = False
        self.answer_received: bool = False
        self.video_track: Optional[VideoCamera] = None
        self.logger = logging.getLogger("CameraClient")
        self.connection_status: str = "Disconnected"
        self.reconnect_attempts: int = 0
        self.max_reconnect_attempts: int = int(os.environ.get("MAX_RECONNECT", 10))
        self.reconnect_delay: int = int(os.environ.get("RECONNECT_DELAY", 2))
        self.timeout: int = int(os.environ.get("WEBRTC_TIMEOUT", 5))
        self.offer_created_at: Optional[float] = None
        self._tasks: set[asyncio.Task] = set()
        self.error_queue: List[Dict[str, Any]] = []
        self.error_status = {
            "camera": "ok", 
            "network": "ok",
            "system": "ok"
        }
        self.error_channel = None  # Data channel for sending error notifications

    def create_tracked_task(self, coro) -> asyncio.Task:
        task = asyncio.create_task(coro)
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)
        return task

    async def connect_websocket(self):
        self.logger.info(f"Initializing WebSocket connection to {self.server_url}...")
        
        if not CameraClient.JWT_TOKEN:
            self.logger.error("Authentication token not set. Please set the JWT_TOKEN variable.")
            self.connection_status = "Auth Error"
            return  
    
        try:
            # Connect to WebSocket without token in URL
            self.websocket = await websockets.connect(self.server_url)
            self.logger.info("WebSocket connection established, authenticating...")
            self.connection_status = "Authenticating"
            
            # Authenticate with token
            auth_success = await self.authenticate()
            if not auth_success:
                self.logger.error("Authentication failed")
                self.connection_status = "Auth Failed"
                if self.websocket:
                    await self.websocket.close()
                return
            
            self.connection_status = "WebSocket Connected"
            
            self.create_tracked_task(self.ping_server())
            await self.message_loop()
            
        except websockets.exceptions.InvalidStatusCode as e:
            if e.status_code == 401:
                self.logger.error(f"WebSocket connection failed: Authentication error (401 Unauthorized). Check your token.")
            else:
                self.logger.error(f"WebSocket connection failed with status code: {e.status_code}")
            self.connection_status = "WebSocket Auth Error"
        except Exception as e:
            self.logger.error(f"WebSocket connection error: {e}")
            self.connection_status = "WebSocket Error"
            await self.try_reconnect()
    
    async def authenticate(self):
        """Send authentication token as a message after connection is established."""
        try:
            auth_message = {
                "type": "authenticate",
                "token": CameraClient.JWT_TOKEN,
                "id": self.STREAMER_ID
            }
            await self.websocket.send(json.dumps(auth_message))
            self.logger.info("Authentication message sent, waiting for response...")
            
            # Wait for authentication response
            response_timeout = 10  # seconds
            try:
                response = await asyncio.wait_for(self.websocket.recv(), timeout=response_timeout)
                data = json.loads(response)
                
                if data.get("type") == "auth_success":
                    self.logger.info("Authentication successful")
                    return True
                elif data.get("type") == "auth_error":
                    self.logger.error(f"Authentication failed: {data.get('message', 'Unknown error')}")
                    return False
                else:
                    self.logger.warning(f"Unexpected authentication response: {data.get('type', 'unknown')}")
                    # Process this message separately since it might be something else
                    self.create_tracked_task(self.process_message(data))
                    # Try to wait for actual auth response
                    return await self.wait_for_auth_response(response_timeout - 1)
            except asyncio.TimeoutError:
                self.logger.error("Authentication response timeout")
                return False
                
        except Exception as e:
            self.logger.error(f"Authentication error: {e}")
            return False
    
    async def wait_for_auth_response(self, timeout):
        """Wait specifically for an auth response message."""
        try:
            start_time = asyncio.get_event_loop().time()
            while asyncio.get_event_loop().time() - start_time < timeout:
                
                response = await asyncio.wait_for(self.websocket.recv(), timeout=1)
                data = json.loads(response)
                
                if data.get("type") == "auth_success":
                    return True
                elif data.get("type") == "auth_error":
                    self.logger.error(f"Authentication failed: {data.get('message', 'Unknown error')}")
                    return False
                else:
                    # Process other messages but keep waiting for auth response
                    self.create_tracked_task(self.process_message(data))
            
            return False  # Timeout reached
        except (asyncio.TimeoutError, Exception) as e:
            self.logger.error(f"Error waiting for auth response: {e}")
            return False
    
    async def send_error_notification(self, error_info: Dict[str, Any]):
        """Send error notification to the client"""
        error_type = error_info.get("type", "unknown")
        severity = error_info.get("severity", "error")
        
        # Update error status based on error type
        if "camera" in error_type:
            if "recovery" in error_type:
                self.error_status["camera"] = "ok"
            else:
                self.error_status["camera"] = severity
        elif "network" in error_type:
            self.error_status["network"] = severity
        else:
            self.error_status["system"] = severity
        
        # Add timestamp
        error_info["timestamp"] = asyncio.get_event_loop().time()
        error_info["id"] = self.STREAMER_ID
        
        # Use WebRTC data channel if available
        if self.error_channel and self.error_channel.readyState == "open":
            try:
                self.error_channel.send(json.dumps(error_info))
                self.logger.info(f"Sent error notification via WebRTC: {error_info['type']} - {error_info['message']}")
                return
            except Exception as e:
                self.logger.error(f"Error sending notification via WebRTC: {e}, falling back to queue")
        
        # Queue error if data channel not available or failed
        if not self.websocket or not self.CLIENT_ID:
            self.logger.warning(f"Cannot send error notification, queuing: {error_info['message']}")
            self.error_queue.append(error_info)
            return
        
        # Add notification to queue for later delivery via data channel
        self.error_queue.append(error_info)
        self.logger.info(f"Queued error notification: {error_info['type']} - {error_info['message']}")

    async def process_message(self, data):
        """Process a non-authentication message."""
        msg_type = data.get('type', 'unknown')
        sender_id = data.get('id', 'unknown')
        
        if msg_type not in ["pong", "candidate"]:
            self.logger.info(f"Received message: {msg_type} from {sender_id}")
        else:
            self.logger.debug(f"Received message: {msg_type} from {sender_id}")

        if msg_type == "error":
            self.logger.error(f"Error from server: {data.get('message')} (related to {data.get('originalType')})")
        elif msg_type == "pong":
            pass
        elif msg_type == "confirm_received":
            await self.send_message("confirmation", {
                "type": "confirm_received",
                "messageType": data.get("messageType"),
                "from": data.get("from"),
                "id": self.STREAMER_ID
            })
        elif msg_type == "client_ready":
            self.logger.info(f"Client with ID {data.get('id')} is ready")
            self.CLIENT_ID = data.get("id", "")
            
            # Send any queued error notifications
            if self.error_queue and self.CLIENT_ID:
                self.logger.info(f"Sending {len(self.error_queue)} queued error notifications")
                for error in self.error_queue:
                    await self.send_error_notification(error)
                self.error_queue = []
                
            await self.start_stream()
        elif msg_type == "answer":
            await self.handle_answer(data)
        elif msg_type == "candidate":
            await self.handle_ice_candidate(data)
    
    async def message_loop(self):
        try:
            async for message in self.websocket:
                try:
                    data: Dict[str, Any] = json.loads(message)
                    await self.process_message(data)
                except json.JSONDecodeError:
                    self.logger.error(f"Error parsing message: {message[:100]}...")
        except websockets.exceptions.ConnectionClosed:
            self.logger.warning("WebSocket connection closed")
            self.connection_status = "Disconnected"
            self.is_streaming = False
            
            if self.peer_connection:
                await self.peer_connection.close()
                self.peer_connection = None
            
            await self.try_reconnect()
    
    async def try_reconnect(self):
        if self.reconnect_attempts < self.max_reconnect_attempts:
            self.reconnect_attempts += 1
            self.logger.info(f"Attempting to reconnect ({self.reconnect_attempts}/{self.max_reconnect_attempts})...")
            await asyncio.sleep(self.reconnect_delay)
            await self.connect_websocket()
        else:
            self.logger.error("Max reconnection attempts reached")
    
    async def ping_server(self): 
        while True:
            if self.websocket:
                try:
                    await self.send_message("ping", {"type": "ping", "id": self.STREAMER_ID, "target": "server"})
                    await asyncio.sleep(15)
                except websockets.exceptions.ConnectionClosed:
                    self.logger.warning("WebSocket closed during ping")
                    break
            else:
                break
    
    async def send_message(self, message_type: str, data: Dict[str, Any]) -> bool:
        if self.websocket:
            try:
                await self.websocket.send(json.dumps(data))
                if message_type not in ["ping", "pong", "candidate"]:
                    self.logger.info(f"Sent {message_type} message successfully")
                else:
                    self.logger.debug(f"Sent {message_type} message successfully")
                return True
            except Exception as e:
                self.logger.error(f"Error sending {message_type} message: {e}")
                return False
        else:
            self.logger.error(f"Cannot send {message_type}, WebSocket not connected")
            return False
    
    async def handle_answer(self, data: Dict[str, Any]):
        sender = data.get('sender', 'unknown')
        self.logger.info(f"Received SDP answer from client {sender}")
        
        try:
            if not self.peer_connection:
                self.logger.error("Error: No RTCPeerConnection when receiving answer")
                return
            
            # Check if we're in a state where we can process an answer
            if self.peer_connection.signalingState == "have-local-offer":
                self.answer_received = True
                answer = RTCSessionDescription(sdp=data["answer"]["sdp"], type=data["answer"]["type"])
                await self.peer_connection.setRemoteDescription(answer)
                self.logger.info("Remote description set successfully")
            else:
                self.logger.warning(f"Ignoring SDP answer: Connection already in {self.peer_connection.signalingState} state")
                
        except Exception as e:
            self.logger.error(f"Error setting remote description: {e}")
            self.logger.info("Attempting to restart the stream connection...")
            await asyncio.sleep(self.reconnect_delay)
            if self.is_streaming:
                await self.start_stream()
    
    async def handle_ice_candidate(self, data: Dict[str, Any]):
        """Process ICE candidate received from client."""
        self.logger.info(f"THE WHOLE DATA {data}")
        sender = data.get('sender', 'unknown')
        self.logger.info(f"Received ICE candidate from client {sender}")
        
        try:
            if not self.peer_connection:
                self.logger.error("Error: No RTCPeerConnection when receiving ICE candidate")
                return
            
            candidate_data = data.get("candidate", {})
            self.logger.info(f"Raw ICE candidate data: {candidate_data}")
            candidate_str = candidate_data.get("candidate", "")
            
            # Log the raw candidate string
            self.logger.info(f"Raw ICE candidate: {candidate_str}")
            
            # Extract and log the candidate type and IP
            candidate_type = "unknown"
            ip_address = "unknown"
            
            # Extract candidate type and the correct IP (before any raddr)
            if " typ " in candidate_str:
                # Regex to capture the host IP (first IPv4 after priority) and the type
                m = re.search(r"\d+ (\d+\.\d+\.\d+\.\d+) \d+ typ (\w+)", candidate_str)
                if m:
                    ip_address = m.group(1)
                    candidate_type = m.group(2)
                else:
                    # Fallback parsing
                    parts = candidate_str.split()
                    if "typ" in parts:
                        idx = parts.index("typ")
                        candidate_type = parts[idx + 1]
                    if len(parts) > 4 and re.match(r'\d+\.\d+\.\d+\.\d+', parts[4]):
                        ip_address = parts[4]
            
            self.logger.info(f"Client ICE candidate: {candidate_type} ({ip_address})")
            
            # Check if this is a srflx candidate (public IP)
            if candidate_type == "srflx":
                self.logger.info(f"🌐 Client's public IP: {ip_address}")
            
            # FIXED: Create RTCIceCandidate with the correct constructor parameters
            # The aiortc library expects these specific parameters without 'candidate'
            candidate = RTCIceCandidate(
                component=1,
                foundation=candidate_str.split(" ")[0].replace("candidate:", ""),
                ip=ip_address,
                port=int(candidate_str.split(" ")[5]) if len(candidate_str.split(" ")) > 5 else 0,
                priority=int(candidate_str.split(" ")[3]) if len(candidate_str.split(" ")) > 3 else 0,
                protocol=candidate_str.split(" ")[2] if len(candidate_str.split(" ")) > 2 else "udp",
                type=candidate_type,
                sdpMLineIndex=candidate_data.get("sdpMLineIndex"),
                sdpMid=candidate_data.get("sdpMid"),
            )
            
            await self.peer_connection.addIceCandidate(candidate)
            self.logger.info(f"Added {candidate_type} ICE candidate from client")
            
        except Exception as e:
            self.logger.error(f"Error adding ICE candidate: {e}")
            # Add detailed debugging for the exception
            self.logger.error(f"Exception details: {type(e).__name__} - {str(e)}")
            import traceback
            self.logger.error(f"Traceback: {traceback.format_exc()}")

    async def start_stream(self):
        self.answer_received = False
        self.offer_created_at = None
        
        try:
            
            if not self.websocket:
                self.logger.error("WebSocket not connected. Reconnecting...")
                await self.connect_websocket()
                return
            
            if self.is_streaming and self.peer_connection:
                self.logger.info("Stopping current stream and starting a new one")
                await self.peer_connection.close()
                self.peer_connection = None
                self.is_streaming = False
            
            self.logger.info("Starting media stream...")
            
            # Pass the error notification callback to VideoCamera
            self.video_track = VideoCamera(error_callback=self.send_error_notification)
            
            # Initialize the _seen_srflx flag at the start of streaming
            self._seen_srflx = False
            
            rtc_config = RTCConfiguration(
                iceServers=[
                    RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
                    RTCIceServer(urls=["stun:stun1.l.google.com:19302"]),
                    RTCIceServer(
                        urls=["turn:delabo.asthaweb.com:3478"],
                        username="AsthaDeLab",
                        credential="DeLaboMSD700"
                    ),
                ]
            )
            self.peer_connection = RTCPeerConnection(rtc_config)
            
            # Create data channel for error notifications
            self.error_channel = self.peer_connection.createDataChannel("errors")
            self.logger.info("Created data channel for error notifications")

            @self.error_channel.on("open")
            def on_error_channel_open():
                self.logger.info("Error notification data channel opened")
                # Send any queued errors
                if self.error_queue:
                    self.logger.info(f"Sending {len(self.error_queue)} queued error notifications via data channel")
                    for error in list(self.error_queue):
                        try:
                            self.error_channel.send(json.dumps(error))
                            self.error_queue.remove(error)
                        except Exception as e:
                            self.logger.error(f"Failed to send queued error: {e}")
            
            @self.error_channel.on("close")
            def on_error_channel_close():
                self.logger.info("Error notification data channel closed")
            
            @self.error_channel.on("error")
            def on_error_channel_error(error):
                self.logger.error(f"Error in error notification data channel: {error}")
            
            self.connection_status = "Setting up WebRTC"
            
            self.peer_connection.addTrack(self.video_track)
            self.logger.info("Added video track to peer connection")
            
            @self.peer_connection.on("icecandidate")
            async def on_ice_candidate(candidate):
            
                if candidate:
                    # Enhanced logging to show candidate details
                    candidate_type = "unknown"
                    ip_address = "unknown"
                    
                    # Extract candidate type and IP address
                    if " typ " in candidate.candidate:
                        parts = candidate.candidate.split(" ")
                        for i, part in enumerate(parts):
                            if part == "typ" and i+1 < len(parts):
                                candidate_type = parts[i+1]
                            elif re.match(r'\d+\.\d+\.\d+\.\d+', part):
                                ip_address = part
                    
                    self.logger.info(f"Generated ICE candidate: {candidate_type} ({ip_address})")
                    
                    # Check if this is a srflx candidate (public IP)
                    if candidate_type == "srflx":
                        self._seen_srflx = True
                        self.logger.info(f"🌐 PUBLIC IP FOUND: {ip_address} (This is good!)")
                    
                    # Send the candidate to the client
                    await self.send_message("candidate", {
                        "type": "candidate",
                        "candidate": {
                            "candidate": candidate.candidate,
                            "sdpMid": candidate.sdpMid,
                            "sdpMLineIndex": candidate.sdpMLineIndex,
                        },
                        "target": self.CLIENT_ID,
                        "id": self.STREAMER_ID
                    })
                    self.logger.info("Sent ICE candidate to client")
                else:
                    self.logger.info("ICE candidate gathering complete")
                    
                    # Check if we got any public IP candidates
                    if not self._seen_srflx:
                        self.logger.warning("⚠️ No public IP (srflx) candidates were gathered!")
                        self.logger.warning("This will cause connection problems across different networks.")
                        self.logger.warning("Running STUN connectivity test to diagnose the issue...")
                        
                        # Run STUN test
                        await self.test_stun_connectivity()

            @self.peer_connection.on("iceconnectionstatechange")
            async def on_ice_connection_state_change():
                state = self.peer_connection.iceConnectionState
                self.logger.info(f"ICE connection state changed: {state}")
                self.connection_status = f"WebRTC: {state}"
                
                if state == "connected" or state == "completed":
                    self.is_streaming = True
                    self.logger.info("Stream is now active and connected to client")
                elif state == "failed":
                    self.logger.error("ICE connection failed - attempting ICE restart...")
                    # Don't use restartIce() as it's not available in this aiortc version
                    await self.restart_ice()
                elif state == "disconnected" or state == "closed":
                    self.is_streaming = False
            
            self.logger.info("Creating SDP offer...")
            offer = await self.peer_connection.createOffer()
            await self.peer_connection.setLocalDescription(offer)
            # Add a longer delay to ensure ICE has time to complete
            self.logger.info("Setting local description and waiting for ICE gathering...")
            # Wait longer to ensure all ICE candidates are gathered
            self.logger.info("Waiting for ICE candidates (5 seconds)...")
            await asyncio.sleep(5)  # Increased from 5 to 8 seconds
            self.logger.info("ICE gathering complete")
            # record timestamp when offer is sent for answer timeout checks
            self.offer_created_at = asyncio.get_event_loop().time()
            offer_dict = {
                "sdp": self.peer_connection.localDescription.sdp,
                "type": self.peer_connection.localDescription.type
            }
            

            success = await self.send_message("offer", {
                "type": "offer",
                "offer": offer_dict,
                "target": self.CLIENT_ID,
                "id": self.STREAMER_ID
            })
            
            if success:
                self.logger.info("SDP offer sent to client")
                
                async def check_answer():
                    await asyncio.sleep(10)
                    current_time = asyncio.get_event_loop().time()
                    if (not self.answer_received and 
                        self.peer_connection and 
                        self.offer_created_at and 
                        (current_time - self.offer_created_at) >= 10):
                        self.logger.warning("No answer received within timeout, connection may have failed")
                
                self.create_tracked_task(check_answer())
            
        except Exception as e:
            self.logger.error(f"Error starting stream: {e}")
            self.connection_status = "Error"
            self.is_streaming = False
            
            # Send error notification about stream startup failure
            error_info = {
                "type": "stream_start_error",
                "severity": "error",
                "message": f"Failed to start stream: {str(e)}",
                "error": str(e)
            }
            await self.send_error_notification(error_info)

    async def restart_ice(self):
        """Restart ICE process by creating a new offer."""
        self.logger.info("Restarting ICE connection...")
        
        if not self.peer_connection or not self.CLIENT_ID:
            self.logger.error("Cannot restart ICE: No peer connection or client ID")
            return
            
        try:
            # In older aiortc versions, createOffer() doesn't support iceRestart option
            # Instead, we close and recreate the connection
            self.logger.info("Recreating connection for ICE restart")
            
            # Save track reference before closing
            saved_track = self.video_track
            
            # Close existing peer connection
            await self.peer_connection.close()
            
            # Create new peer connection with same config
            rtc_config = RTCConfiguration(
                iceServers=[
                    RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
                    RTCIceServer(urls=["stun:stun1.l.google.com:19302"]),
                    RTCIceServer(urls=["stun:stun.ekiga.net:3478"]),
                ]
            )
            self.peer_connection = RTCPeerConnection(rtc_config)
            
            # Re-add the track and data channel
            self.peer_connection.addTrack(saved_track)
            self.error_channel = self.peer_connection.createDataChannel("errors")
            self.logger.info("Recreated data channel for error notifications")
            
            # Reset flags
            self._seen_srflx = False
            self.answer_received = False
            
            # Set up event handlers again
            @self.peer_connection.on("icecandidate")
            async def on_ice_candidate(candidate):
                if candidate:
                    candidate_type = "unknown"
                    ip_address = "unknown"
                    
                    if " typ " in candidate.candidate:
                        parts = candidate.candidate.split(" ")
                        for i, part in enumerate(parts):
                            if part == "typ" and i+1 < len(parts):
                                candidate_type = parts[i+1]
                            elif re.match(r'\d+\.\d+\.\d+\.\d+', part):
                                ip_address = part
                    
                    self.logger.info(f"Generated ICE candidate (restart): {candidate_type} ({ip_address})")
                    
                    if candidate_type == "srflx":
                        self._seen_srflx = True
                        self.logger.info(f"🌐 PUBLIC IP FOUND after restart: {ip_address}")
                    
                    await self.send_message("candidate", {
                        "type": "candidate",
                        "candidate": {
                            "candidate": candidate.candidate,
                            "sdpMid": candidate.sdpMid,
                            "sdpMLineIndex": candidate.sdpMLineIndex,
                        },
                        "target": self.CLIENT_ID,
                        "id": self.STREAMER_ID
                    })
                else:
                    self.logger.info("ICE candidate gathering complete (restart)")
                    
                    if not self._seen_srflx:
                        self.logger.warning("⚠️ No public IP candidates after restart!")
        
            @self.peer_connection.on("iceconnectionstatechange")
            async def on_ice_connection_state_change():
                # Same handler as in start_stream
                state = self.peer_connection.iceConnectionState
                self.logger.info(f"ICE connection state changed (restart): {state}")
                
                if state == "connected" or state == "completed":
                    self.is_streaming = True
                    self.logger.info("Stream is now active after ICE restart")
                elif state == "failed":
                    self.logger.error("ICE connection failed again after restart")
                    # Instead of restarting immediately, notify failure
                    error_info = {
                        "type": "ice_failure_persistent",
                        "severity": "error",
                        "message": "ICE connection failed repeatedly, likely NAT traversal issue",
                    }
                    await self.send_error_notification(error_info)
                    
                    # Run STUN test on persistent failure to help diagnose
                    await self.test_stun_connectivity()
                elif state == "disconnected" or state == "closed":
                    self.is_streaming = False
            
            # Create new offer
            self.logger.info("Creating new offer after connection recreation")
            offer = await self.peer_connection.createOffer()
            await self.peer_connection.setLocalDescription(offer)
            
            # Wait a short time to gather some candidates
            await asyncio.sleep(2)
            
            # Send the new offer
            offer_dict = {
                "sdp": self.peer_connection.localDescription.sdp,
                "type": self.peer_connection.localDescription.type
            }
            
            success = await self.send_message("offer", {
                "type": "offer",
                "offer": offer_dict,
                "target": self.CLIENT_ID,
                "id": self.STREAMER_ID,
                "isRestart": True  # Flag for the client to know this is a restart
            })
            
            if success:
                self.logger.info("Restart offer sent successfully")
                self.offer_created_at = asyncio.get_event_loop().time()
        except Exception as e:
            self.logger.error(f"Error during ICE restart: {e}")
            # If the manual restart fails, fall back to full reconnection
            self.logger.info("ICE restart failed, attempting to recreate the entire stream...")
            await asyncio.sleep(2)
            await self.start_stream()

    async def cleanup(self):
        self.logger.info("Cleaning up resources...")
        for task in self._tasks:
            if not task.done():
                task.cancel()
        
        if self._tasks:
            await asyncio.gather(*self._tasks, return_exceptions=True)
        
        if self.peer_connection:
            await self.peer_connection.close()
        
        if self.websocket and self.websocket.open:
            await self.websocket.close()
        self.logger.info("Cleanup complete.")

    async def test_stun_connectivity(self):
        """Test direct connectivity to STUN servers to diagnose problems."""
        import socket
        import random
        import struct
        
        self.logger.info("Running STUN connectivity test...")
        
        stun_servers = [
            ("stun.l.google.com", 19302),
            ("stun1.l.google.com", 19302),
            ("stun.ekiga.net", 3478)  # Use servers that worked in your test
        ]
        
        # Create a simple STUN binding request
        transaction_id = random.randbytes(12)
        stun_request = struct.pack("!HHI12s", 0x0001, 0x0000, 0x2112A442, transaction_id)
        
        for server, port in stun_servers:
            try:
                # Resolve hostname to IP
                server_ip = socket.gethostbyname(server)
                self.logger.info(f"Testing STUN server: {server} ({server_ip}:{port})")
                
                # Create UDP socket
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(3)  # 3 second timeout
                
                # Send binding request
                sock.sendto(stun_request, (server_ip, port))
                self.logger.info(f"STUN request sent to {server}")
                
                # Try to receive response
                try:
                    data, addr = sock.recvfrom(1024)
                    if data[0:2] == b'\x01\x01':  # Success response
                        self.logger.info(f"✅ STUN test successful: {server} responded!")
                        
                        # Try to parse the mapped address
                        try:
                            # Extract mapped address type and address from STUN response
                            # This is a simplified parser and may not work for all STUN server responses
                            if len(data) > 20:
                                # Skip the header (20 bytes)
                                pos = 20
                                while pos < len(data):
                                    attr_type = struct.unpack("!H", data[pos:pos+2])[0]
                                    attr_len = struct.unpack("!H", data[pos+2:pos+4])[0]
                                    if attr_type == 0x0001:  # MAPPED-ADDRESS
                                        family = data[pos+5]
                                        port = struct.unpack("!H", data[pos+6:pos+8])[0]
                                        if family == 1:  # IPv4
                                            ip = socket.inet_ntoa(data[pos+8:pos+12])
                                            self.logger.info(f"🌐 YOUR PUBLIC IP: {ip}:{port}")
                                        break
                                    # Move to the next attribute
                                    pos += 4 + attr_len
                                    # Align to 4-byte boundary if needed
                                    padding = attr_len % 4
                                    if padding > 0:
                                        pos += 4 - padding
                        except Exception as e:
                            self.logger.error(f"Error parsing STUN response: {e}")
                    else:
                        self.logger.warning(f"Received unexpected response from {server}")
                except socket.timeout:
                    self.logger.error(f"⚠️ Timeout waiting for response from {server}")
                except Exception as e:
                    self.logger.error(f"Error receiving STUN response: {e}")
                
                sock.close()
                
            except Exception as e:
                self.logger.error(f"STUN test failed for {server}: {e}")

async def main():
    main_logger = logging.getLogger("CameraClient.Main")
    main_logger.info("Starting Camera Client...")
    
    if not CameraClient.JWT_TOKEN:
        main_logger.error("JWT_TOKEN is not found or token file is missing/unreadable.")
        return
        
    client = CameraClient(server_url=SERVER_URL, streamer_id=STREAMER_ID)
    main_logger.info(f"Attempting to connect to {client.server_url} as {client.STREAMER_ID}...")
    
    try:
        await client.connect_websocket()
        if client.websocket:
            main_logger.info("WebSocket connection established. Client running...")
            await asyncio.Future()
        else:
            main_logger.error("Failed to establish initial WebSocket connection.")
    except KeyboardInterrupt:
        main_logger.info("Interrupted by user")
    except asyncio.CancelledError:
        main_logger.info("Main task cancelled.")
    finally:
        main_logger.info("Initiating client cleanup...")
        await client.cleanup()

if __name__ == "__main__":
    import sys
    # Allow custom STUN server setting via command line
    if len(sys.argv) > 1 and sys.argv[1].startswith("stun:"):
        custom_stun = sys.argv[1]
        logger.info(f"Using custom STUN server: {custom_stun}")
        os.environ["STUN_SERVERS"] = custom_stun
    
    asyncio.run(main())
