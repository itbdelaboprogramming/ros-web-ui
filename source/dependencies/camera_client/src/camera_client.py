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

# Set up logging configuration with both file and console handlers
def setup_logging():
    # Get log level from environment variable or default to INFO
    log_level_name = os.environ.get('LOG_LEVEL', 'INFO').upper()
    log_level = getattr(logging, log_level_name, logging.INFO)
    
    # Create logs directory if it doesn't exist
    log_dir = 'logs'
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # Configure root logger
    logger = logging.getLogger()
    logger.setLevel(log_level)
    
    # Clear any existing handlers
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
    
    # Detailed log format with process ID
    log_format = logging.Formatter(
        '%(asctime)s - %(process)d - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Add rotating file handler (10MB max size, keep 5 backup files)
    file_handler = RotatingFileHandler(
        os.path.join(log_dir, 'camera_client.log'),
        maxBytes=10*1024*1024,  # 10MB
        backupCount=5
    )
    file_handler.setFormatter(log_format)
    logger.addHandler(file_handler)
    
    # Add console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(log_format)
    logger.addHandler(console_handler)
    
    return logger

# Initialize logging
logger = setup_logging()
logger.info("Logging system initialized")

"""WebRTC video streaming client using aiortc and OpenCV."""
class VideoCamera(VideoStreamTrack):
    """Camera video track that captures from a camera."""

    def __init__(self):
        # Camera quality settings, set the camera resolution
        lowres = [640, 480]
        highres = [1280, 720]
        
        # Set the resolution we'll use
        self.width = highres[0]
        self.height = highres[1]

        # Initialize camera capture
        super().__init__()
        self.logger = logging.getLogger("VideoCamera")
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        
        # Create fallback frame immediately for better performance
        self.fallback_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Track whether camera is released
        self._camera_released = False

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        ret, frame = self.cap.read()
        if not ret:
            self.logger.error("Could not read from camera")
            frame = self.fallback_frame

        # Color conversion from BGR to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        # Release camera resources
        if not self._camera_released and hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self._camera_released = True
            self.logger.info("Camera resources released by stop()")

    def __del__(self):
        # Destructor to ensure camera is released
        if hasattr(self, '_camera_released') and not self._camera_released and hasattr(self, 'cap'):
            self.cap.release()
            self.logger.info("Camera resources released by destructor")

# Class of the client
class CameraClient:
    """Client that connects to a signaling server and streams video via WebRTC."""
    
    STREAMER_ID = "AX591"  # This should ideally match the userId in the JWT
    CLIENT_ID = "client"  # Target client ID for WebRTC signalling
    SERVER = "ws://localhost:3001"  # Example HTTP endpoint (adjust if needed)
    # SERVER = "wss://delabo.asthaweb.com/signalling"  # Use WS for WebSocket connection
    
    # --- Authentication Token ---
    # !! IMPORTANT: Read token from file !!
    try:
        with open('token.cred', 'r') as f:
            JWT_TOKEN = f.read().strip()
    except FileNotFoundError:
        logger.error("Error: token.cred file not found")
        JWT_TOKEN = None
    except Exception as e:
        logger.error(f"Error reading token file: {e}")
        JWT_TOKEN = None
    # --------------------------

    # Initial WebRTC constructor
    def __init__(self, server_url=SERVER):
        self.server_url = server_url
        self.websocket = None
        self.peer_connection = None
        self.is_streaming = False
        self.answer_received = False
        self.video_track = None
        self.logger = logging.getLogger("CameraClient")
        self.connection_status = "Disconnected"
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 10
        self.reconnect_delay = 2
        self.timeout = 5
        
        # Task tracking
        self._tasks = set()
    
    # Helper for creating tracked tasks
    def create_tracked_task(self, coro):
        task = asyncio.create_task(coro)
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)
        return task
    
    async def connect_websocket(self):
        # Connect to the signaling server via WebSocket
        self.logger.info(f"Initializing WebSocket connection to {self.server_url}...")
        
        if not CameraClient.JWT_TOKEN:
            self.logger.error("Authentication token not set. Please set the JWT_TOKEN variable.")
            self.connection_status = "Auth Error"
            return  
    
        # Build the URL with token as query parameter
        """
        This method isn't the most secure way to handle tokens
        """
        ws_url = f"{self.server_url}?token={CameraClient.JWT_TOKEN}"
        
        try:
            self.websocket = await websockets.connect(ws_url)
            self.logger.info("WebSocket connection established")
            self.connection_status = "WebSocket Connected"
            
            # Start ping task to keep connection alive
            self.create_tracked_task(self.ping_server())
            
            # Start message handling loop
            await self.message_loop()
            
        except websockets.exceptions.InvalidStatusCode as e:
             # Handle specific authentication errors (e.g., 401 Unauthorized)
            if e.status_code == 401:
                self.logger.error(f"WebSocket connection failed: Authentication error (401 Unauthorized). Check your token.")
            else:
                 self.logger.error(f"WebSocket connection failed with status code: {e.status_code}")
            self.connection_status = "WebSocket Auth Error"
        except Exception as e:
            self.logger.error(f"WebSocket connection error: {e}")
            self.connection_status = "WebSocket Error"
            await self.try_reconnect()
    
    # Attempt to reconnect to the WebSocket server
    async def try_reconnect(self):
        if self.reconnect_attempts < self.max_reconnect_attempts:
            self.reconnect_attempts += 1
            self.logger.info(f"Attempting to reconnect ({self.reconnect_attempts}/{self.max_reconnect_attempts})...")
            await asyncio.sleep(self.reconnect_delay)
            await self.connect_websocket()
        else:
            self.logger.error("Max reconnection attempts reached")
    
    # Send periodic pings to keep the connection alive
    async def ping_server(self): 
        while True:
            if self.websocket:
                try:
                    await self.send_message("ping", {"type": "ping", "id": self.STREAMER_ID, "target": "server"})
                    await asyncio.sleep(15)  # Every 15 seconds
                except websockets.exceptions.ConnectionClosed:
                    self.logger.warning("WebSocket closed during ping")
                    break
            else:
                break
    
    # Send a message to the signaling server
    async def send_message(self, message_type, data):
        if self.websocket:
            try:
                await self.websocket.send(json.dumps(data))
                self.logger.info(f"Sent {message_type} message successfully")
                return True
            except Exception as e:
                self.logger.error(f"Error sending {message_type} message: {e}")
                return False
        else:
            self.logger.error(f"Cannot send {message_type}, WebSocket not connected")
            return False
    
    # Handle incoming messages from the server
    async def message_loop(self):
        try:
            async for message in self.websocket:
                try:
                    data = json.loads(message)
                    self.logger.info(f"Received message: {data.get('type')} from {data.get('id', 'unknown')}")
                    
                    # Handle different message types
                    if data.get("type") == "error":
                        self.logger.error(f"Error from server: {data.get('message')} (related to {data.get('originalType')})")
                    
                    elif data.get("type") == "pong":
                        pass
                    
                    elif data.get("type") == "confirm_received":
                        await self.send_message("confirmation", {
                            "type": "confirm_received",
                            "messageType": data.get("messageType"),
                            "from": data.get("from"),
                            "id": self.STREAMER_ID
                        })

                    elif data.get("type") == "client_ready":
                        self.logger.info(f"Client with ID {data.get('id')} is ready")
                        # Update client ID
                        self.CLIENT_ID = data.get("id")
                        await self.start_stream()
                    
                    elif data.get("type") == "answer":
                        await self.handle_answer(data)
                    
                    elif data.get("type") == "candidate":
                        await self.handle_ice_candidate(data)
                        
                except json.JSONDecodeError:
                    self.logger.error(f"Error parsing message: {message[:100]}...")
                    
        # Handle connection closed
        except websockets.exceptions.ConnectionClosed:
            self.logger.warning("WebSocket connection closed")
            self.connection_status = "Disconnected"
            self.is_streaming = False
            
            if self.peer_connection:
                await self.peer_connection.close()
                self.peer_connection = None
            
            await self.try_reconnect()
    
    # Handle SDP answer from the client, sent after receiving the offer
    async def handle_answer(self, data):
        self.answer_received = True
        self.logger.info(f"Received SDP answer from client {data.get('sender')}")
        
        try:
            if not self.peer_connection:
                self.logger.error("Error: No RTCPeerConnection when receiving answer")
                return
            
            # Acknowledge receipt to the server
            await self.send_message("answer_ack", {
                "type": "answer_ack",
                "target": data.get("sender"),
                "id": self.STREAMER_ID
            })
            
            # Set the remote description (client's answer)
            answer = RTCSessionDescription(sdp=data["answer"]["sdp"], type=data["answer"]["type"])
            await self.peer_connection.setRemoteDescription(answer)
            self.logger.info("Remote description set successfully")
            
        except Exception as e:
            self.logger.error(f"Error setting remote description: {e}")
            self.logger.info("Attempting to restart the stream connection...")
            await asyncio.sleep(self.reconnect_delay)
            if self.is_streaming:
                await self.start_stream()
    
    # Handle ICE candidate from the client, sent after receiving the offer
    async def handle_ice_candidate(self, data):
        self.logger.info(f"Received ICE candidate from client {data.get('sender')}")
        try:
            if not self.peer_connection:
                self.logger.error("Error: No RTCPeerConnection when receiving ICE candidate")
                return
            
            # Parse the candidate string - contoh: "candidate:foundation component protocol priority ip port type ..."
            candidate_str = data["candidate"].get("candidate", "")
            if not candidate_str:
                self.logger.error("Empty candidate string received")
                return
                
            # Regex to parse the candidate string
            parts = re.match(r'candidate:(\S+) (\d+) (\S+) (\d+) (\S+) (\d+) typ (\S+)(?: raddr (\S+) rport (\d+))?', candidate_str)
            
            if not parts:
                self.logger.error(f"Could not parse candidate string: {candidate_str}")
                return
                
            # Create RTCIceCandidate object
            candidate = RTCIceCandidate(
                foundation=parts.group(1),
                component=int(parts.group(2)),
                protocol=parts.group(3),
                priority=int(parts.group(4)),
                ip=parts.group(5),
                port=int(parts.group(6)),
                type=parts.group(7),
                relatedAddress=parts.group(8) if parts.group(8) else None,
                relatedPort=int(parts.group(9)) if parts.group(9) else None,
                sdpMLineIndex=data["candidate"].get("sdpMLineIndex"),
                sdpMid=data["candidate"].get("sdpMid")
            )
            
            await self.peer_connection.addIceCandidate(candidate)
            self.logger.info("ICE candidate added successfully")
            
        except Exception as e:
            self.logger.error(f"Error adding ICE candidate: {e}")
    
    # Start the media stream
    async def start_stream(self):
        # Reset answer received state
        self.answer_received = False
        
        try:

            # =======================================================================
            # Initial check for WebSocket connection and peer connection
            if not self.websocket:
                self.logger.error("WebSocket not connected. Reconnecting...")
                await self.connect_websocket()
                return
            
            if self.is_streaming and self.peer_connection:
                # If already streaming, close the current connection and start a new one
                self.logger.info("Stopping current stream and starting a new one")
                await self.peer_connection.close()
                self.peer_connection = None
                self.is_streaming = False
            # =======================================================================
            
            self.logger.info("Starting media stream...")
            
            # Create video track
            self.video_track = VideoCamera()
            
            # Create a new RTCPeerConnection with ice Servers
            rtc_config = RTCConfiguration(
                iceServers=[
                    RTCIceServer(urls=["stun:stun.l.google.com:19302"]),
                    RTCIceServer(urls=["stun:stun1.l.google.com:19302"]),
                ]
            )
            self.logger.info("Initializing RTCPeerConnection...")
            self.peer_connection = RTCPeerConnection(rtc_config)
            
            self.connection_status = "Setting up WebRTC"
            
            # Add track to peer connection
            self.peer_connection.addTrack(self.video_track)
            self.logger.info("Added video track to peer connection")
            
            # Set up ICE candidate handling
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
            
            # Monitor connection state changes
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
                    # Set a short timeout before considering it a full disconnection
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
            
            # Create the offer
            self.logger.info("Creating SDP offer...")
            offer = await self.peer_connection.createOffer()
            await self.peer_connection.setLocalDescription(offer)
            self.logger.info("Local description set")
            
            # Wait a moment to gather some ICE candidates
            self.logger.info("Waiting briefly to gather ICE candidates...")
            await asyncio.sleep(1)
            
            # Send the offer to the client
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
                
                # Set up a timer to check if we got an answer
                async def check_answer():
                    await asyncio.sleep(10)
                    if not self.answer_received and self.peer_connection:
                        self.logger.warning("No answer received within timeout, connection may have failed")
                
                self.create_tracked_task(check_answer())
            
        except Exception as e:
            self.logger.error(f"Error starting stream: {e}")
            self.connection_status = "Error"
            self.is_streaming = False
    
    # Cleanup function to close connections
    async def cleanup(self):
        # cancel all tasks
        for task in self._tasks:
            if not task.done():
                task.cancel()
        
        # wait for all tasks to finish
        if self._tasks:
            await asyncio.gather(*self._tasks, return_exceptions=True)
        
        # close peer connection and websocket
        if self.peer_connection:
            await self.peer_connection.close()
        
        if self.websocket and self.websocket.open:
            await self.websocket.close()

# Main function to run the client
async def main():
    logger = logging.getLogger("Main")
    logger.info("Starting Camera Client...")
    # Check if token is set before starting
    if not CameraClient.JWT_TOKEN:
        logger.error("JWT_TOKEN is not found on token.cred file or file is not found.")
        return
        
    client = CameraClient()
    logger.info(f"Attempting to connect to {client.server_url} as {client.STREAMER_ID}...")
    
    try:
        await client.connect_websocket()
        logger.info("WebSocket connection established. Starting video stream...")
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        await client.cleanup()

# Run the main function
if __name__ == "__main__":        
    asyncio.run(main())
