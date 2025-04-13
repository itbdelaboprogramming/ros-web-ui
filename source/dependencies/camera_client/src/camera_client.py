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
from typing import Optional, Dict, Any

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
DEFAULT_SERVER_URL = "ws://localhost:3001"
DEFAULT_STREAMER_ID = "AX591"
DEFAULT_STUN_SERVER = "stun:stun.l.google.com:19302"

SERVER_URL = os.environ.get("SERVER_URL", DEFAULT_SERVER_URL)
STREAMER_ID = os.environ.get("STREAMER_ID", DEFAULT_STREAMER_ID)
STUN_SERVERS = [
    RTCIceServer(urls=[os.environ.get("STUN_SERVER_1", DEFAULT_STUN_SERVER)]),
]

"""WebRTC video streaming client using aiortc and OpenCV."""
class VideoCamera(VideoStreamTrack):
    def __init__(self):
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

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        ret, frame = self.cap.read()
        if not ret:
            self.logger.error("Could not read from camera")
            frame = self.fallback_frame

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

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
    
        ws_url = f"{self.server_url}?token={CameraClient.JWT_TOKEN}"
        
        try:
            self.websocket = await websockets.connect(ws_url)
            self.logger.info("WebSocket connection established")
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
    
    async def message_loop(self):
        try:
            async for message in self.websocket:
                try:
                    data: Dict[str, Any] = json.loads(message)
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
                        await self.start_stream()

                    elif msg_type == "answer":
                        await self.handle_answer(data)

                    elif msg_type == "candidate":
                        await self.handle_ice_candidate(data)

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
    
    async def handle_answer(self, data: Dict[str, Any]):
        self.answer_received = True
        self.logger.info(f"Received SDP answer from client {data.get('sender')}")
        
        try:
            if not self.peer_connection:
                self.logger.error("Error: No RTCPeerConnection when receiving answer")
                return
            
            await self.send_message("answer_ack", {
                "type": "answer_ack",
                "target": data.get("sender"),
                "id": self.STREAMER_ID
            })
            
            answer = RTCSessionDescription(sdp=data["answer"]["sdp"], type=data["answer"]["type"])
            await self.peer_connection.setRemoteDescription(answer)
            self.logger.info("Remote description set successfully")
            
        except Exception as e:
            self.logger.error(f"Error setting remote description: {e}")
            self.logger.info("Attempting to restart the stream connection...")
            await asyncio.sleep(self.reconnect_delay)
            if self.is_streaming:
                await self.start_stream()
    
    async def handle_ice_candidate(self, data: Dict[str, Any]):
        sender = data.get('sender', 'unknown')
        self.logger.debug(f"Received ICE candidate from client {sender}")
        try:
            if not self.peer_connection:
                self.logger.error("Error: No RTCPeerConnection when receiving ICE candidate")
                return

            candidate_data = data.get("candidate")
            if not isinstance(candidate_data, dict):
                self.logger.error(f"Invalid candidate data format received from {sender}: {candidate_data}")
                return

            candidate_str = candidate_data.get("candidate", "")
            if not candidate_str:
                self.logger.error(f"Empty candidate string received from {sender}")
                return

            parts = re.match(r'candidate:(\S+) (\d+) (\S+) (\d+) (\S+) (\d+) typ (\S+)(?: raddr (\S+) rport (\d+))?', candidate_str)

            if not parts:
                self.logger.error(f"Could not parse candidate string from {sender}: {candidate_str}")
                await self.send_message("candidate_error", {
                    "type": "candidate_error",
                    "target": data.get("sender", self.CLIENT_ID),
                    "id": self.STREAMER_ID,
                    "error": "Invalid candidate format"
                })
                return

            try:
                foundation = parts.group(1)
                component = int(parts.group(2))
                protocol = parts.group(3)
                priority = int(parts.group(4))
                ip = parts.group(5)
                port = int(parts.group(6))
                typ = parts.group(7)
                relatedAddress = parts.group(8)
                relatedPort_str = parts.group(9)
                relatedPort = int(relatedPort_str) if relatedPort_str else None
                sdpMLineIndex = candidate_data.get("sdpMLineIndex")
                sdpMid = candidate_data.get("sdpMid")

                if sdpMLineIndex is None or sdpMid is None:
                    raise KeyError("Missing sdpMLineIndex or sdpMid")

            except (ValueError, TypeError) as e:
                self.logger.error(f"Error converting candidate parts from {sender}: {e}. String was: {candidate_str}")
                return
            except KeyError as e:
                self.logger.error(f"Missing required field in ICE candidate data from {sender}: {e}")
                self.logger.debug(f"Received candidate data: {data}")
                return

            candidate = RTCIceCandidate(
                foundation=foundation,
                component=component,
                protocol=protocol,
                priority=priority,
                ip=ip,
                port=port,
                type=typ,
                relatedAddress=relatedAddress,
                relatedPort=relatedPort,
                sdpMLineIndex=sdpMLineIndex,
                sdpMid=sdpMid
            )

            await self.peer_connection.addIceCandidate(candidate)
            self.logger.debug("ICE candidate added successfully")

        except Exception as e:
            self.logger.error(f"Unexpected error adding ICE candidate from {sender}: {e}")
            self.logger.debug(f"Problematic candidate data: {data}")
    
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
            
            self.video_track = VideoCamera()
            
            rtc_config = RTCConfiguration(iceServers=STUN_SERVERS)
            self.logger.info(f"Initializing RTCPeerConnection with STUN: {[s.urls for s in STUN_SERVERS]}")
            self.peer_connection = RTCPeerConnection(rtc_config)
            
            self.connection_status = "Setting up WebRTC"
            
            self.peer_connection.addTrack(self.video_track)
            self.logger.info("Added video track to peer connection")
            
            @self.peer_connection.on("icecandidate")
            async def on_ice_candidate(candidate):
                if candidate:
                    self.logger.info("Generated ICE candidate")
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
            
            @self.peer_connection.on("iceconnectionstatechange")
            async def on_ice_connection_state_change():
                state = self.peer_connection.iceConnectionState
                self.logger.info(f"ICE connection state changed: {state}")
                self.connection_status = f"WebRTC: {state}"
                
                if state == "connected" or state == "completed":
                    self.is_streaming = True
                    self.logger.info("Stream is now active and connected to client")
                elif state == "failed":
                    self.logger.error("ICE connection failed - attempting to restart ICE")
                    await self.peer_connection.restartIce()
                elif state == "disconnected":
                    self.logger.warning("Client connection unstable, will close if not recovered quickly")
                    self.is_streaming = False
                    
                    async def disconnect_timeout():                        
                        await asyncio.sleep(self.timeout)
                        if (self.peer_connection and 
                            self.peer_connection.iceConnectionState == "disconnected"):
                            self.logger.warning("Client disconnection timeout reached, closing connection")
                            if self.video_track:
                                self.video_track.stop()
                                self.video_track = None
                            await self.peer_connection.close()
                            self.peer_connection = None
                    
                    self.create_tracked_task(disconnect_timeout())
                    
                elif state == "closed":
                    self.is_streaming = False
                    self.logger.warning("Client connection closed, stopping stream")
                    if self.video_track:
                        self.video_track.stop()
                        self.video_track = None
                    await self.peer_connection.close()
                    self.peer_connection = None
            
            self.logger.info("Creating SDP offer...")
            offer = await self.peer_connection.createOffer()
            await self.peer_connection.setLocalDescription(offer)
            self.logger.info("Local description set")
            
            self.offer_created_at = asyncio.get_event_loop().time()
            
            self.logger.info("Waiting briefly to gather ICE candidates...")
            await asyncio.sleep(1)
            
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
    asyncio.run(main())
