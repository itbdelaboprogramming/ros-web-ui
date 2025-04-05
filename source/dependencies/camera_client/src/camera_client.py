import asyncio
import json
import logging
import time
import cv2
import websockets
from datetime import datetime
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, RTCConfiguration, RTCIceServer
from aiortc.mediastreams import VideoStreamTrack
from av import VideoFrame

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


"""WebRTC video streaming client using aiortc and OpenCV."""
class VideoCamera(VideoStreamTrack):
    """Camera video track that captures from a camera."""
    

    def __init__(self):
        lowres = [640,480]
        highres = [1280,720]
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, highres[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, highres[1])

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        ret, frame = self.cap.read()
        if not ret:
            logging.error("Could not read from camera")
            # Preload fallback frame jika belum tersedia
            if not hasattr(self, 'fallback_frame'):
                black_path = cv2.samples.findFile("black.png")
                black_frame = cv2.imread(black_path, cv2.IMREAD_COLOR)
                # Gambar fallback diisi dengan rectangle hitam
                self.fallback_frame = cv2.rectangle(black_frame, (0, 0), (640, 480), (0, 0, 0), -1)
            frame = self.fallback_frame

        # Lakukan konversi warna satu kali saja
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def __del__(self):
        self.cap.release()

class CameraClient:
    """Client that connects to a signaling server and streams video via WebRTC."""
    
    STREAMER_ID = "streamer"
    CLIENT_ID = "client"
    # SERVER = "ws://13.215.160.171:3002"
    SERVER = "wss://delabo.asthaweb.com/signalling"
    
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
        self.reconnect_delay = 5  # seconds
        
    async def connect_websocket(self):
        """Connect to the signaling server via WebSocket."""
        self.logger.info(f"Initializing WebSocket connection to {self.server_url}...")
        
        try:
            self.websocket = await websockets.connect(self.server_url)
            self.logger.info("WebSocket connection established")
            self.connection_status = "WebSocket Connected"
            
            # Send registration message

            print("Registering streamer...")
            await self.send_message("register", {
                "type": "register", 
                "id": self.STREAMER_ID
            })
            self.logger.info(f"Registered as {self.STREAMER_ID}")
            
            # Start ping task to keep connection alive
            asyncio.create_task(self.ping_server())
            
            # Start message handling loop
            await self.message_loop()
            
        except Exception as e:
            self.logger.error(f"WebSocket connection error: {e}")
            self.connection_status = "WebSocket Error"
            await self.try_reconnect()
    
    async def try_reconnect(self):
        """Attempt to reconnect to the WebSocket server."""
        if self.reconnect_attempts < self.max_reconnect_attempts:
            self.reconnect_attempts += 1
            self.logger.info(f"Attempting to reconnect ({self.reconnect_attempts}/{self.max_reconnect_attempts})...")
            await asyncio.sleep(self.reconnect_delay)
            await self.connect_websocket()
        else:
            self.logger.error("Max reconnection attempts reached")
    
    async def ping_server(self):
        """Send periodic pings to keep the connection alive."""
        while True:
            if self.websocket:
                try:
                    await self.send_message("ping", {"type": "ping"})
                    await asyncio.sleep(15)  # Every 15 seconds
                except websockets.exceptions.ConnectionClosed:
                    self.logger.warning("WebSocket closed during ping")
                    break
            else:
                break
    
    async def send_message(self, message_type, data):
        """Send a message to the signaling server."""
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
    
    async def message_loop(self):
        """Handle incoming WebSocket messages."""
        try:
            async for message in self.websocket:
                try:
                    data = json.loads(message)
                    self.logger.info(f"Received message: {data.get('type')} from {data.get('sender', 'unknown')}")
                    
                    # Handle different message types
                    if data.get("type") == "error":
                        self.logger.error(f"Error from server: {data.get('message')} (related to {data.get('originalType')})")
                    
                    elif data.get("type") == "pong":
                        pass  # Ignore pong responses
                    
                    elif data.get("type") == "confirm_received":
                        await self.send_message("confirmation", {
                            "type": "confirm_received",
                            "messageType": data.get("messageType"),
                            "from": data.get("from"),
                            "id": self.STREAMER_ID
                        })
                    
                    elif data.get("type") == "client_ready":
                        # Start streaming when a client indicates they're ready
                        self.logger.info("Client ready to receive stream, starting...")
                        await self.start_stream()
                    
                    elif data.get("type") == "answer":
                        await self.handle_answer(data)
                    
                    elif data.get("type") == "candidate":
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
    
    async def handle_answer(self, data):
        """Process SDP answer from client."""
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
            # Try to restart the connection
            self.logger.info("Attempting to restart the stream connection...")
            await asyncio.sleep(2)
            if self.is_streaming:
                await self.start_stream()
    
    async def handle_ice_candidate(self, data):
        """Add ICE candidate received from client."""
        self.logger.info(f"Received ICE candidate from client {data.get('sender')}")
        try:
            if not self.peer_connection:
                self.logger.error("Error: No RTCPeerConnection when receiving ICE candidate")
                return
            
            candidate = RTCIceCandidate(
                sdpMLineIndex=data["candidate"].get("sdpMLineIndex"),
                sdpMid=data["candidate"].get("sdpMid"),
                candidate=data["candidate"].get("candidate")
            )
            
            await self.peer_connection.addIceCandidate(candidate)
            self.logger.info("ICE candidate added successfully")
            
        except Exception as e:
            self.logger.error(f"Error adding ICE candidate: {e}")
    
    async def start_stream(self):
        """Initialize WebRTC peer connection and start streaming."""
        # Reset answer received state
        self.answer_received = False
        
        try:
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
            
            self.logger.info("Starting media stream...")
            
            # Create video track
            self.video_track = VideoCamera()
            
            # Create a new RTCPeerConnection with proper RTCConfiguration
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
                elif state == "disconnected" or state == "closed":
                    self.is_streaming = False
            
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
                
                asyncio.create_task(check_answer())
            
        except Exception as e:
            self.logger.error(f"Error starting stream: {e}")
            self.connection_status = "Error"
            self.is_streaming = False
    
    async def request_answer(self):
        """Request the client to resend their answer."""
        if not self.websocket or not self.websocket.open:
            self.logger.error("WebSocket not connected, cannot request answer")
            return
        
        self.logger.info("Requesting client to resend answer")
        await self.send_message("request_answer", {
            "type": "request_answer",
            "target": self.CLIENT_ID,
            "id": self.STREAMER_ID
        })
        
    async def cleanup(self):
        """Clean up resources."""
        if self.peer_connection:
            await self.peer_connection.close()
        
        if self.websocket and self.websocket.open:
            await self.websocket.close()

async def main():
    print("Starting Camera Client...")
    client = CameraClient()
    print("Connecting to WebSocket server...")
    
    try:
        await client.connect_websocket()
        print("WebSocket connection established. Starting video stream...")
    except KeyboardInterrupt:
        logging.info("Interrupted by user")
    finally:
        await client.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
