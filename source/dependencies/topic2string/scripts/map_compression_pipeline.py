#!/usr/bin/env python3
import rospy
import json
import zlib
import base64
import time
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from rospy_message_converter import message_converter

class MapCompressor:
    """
    This class is responsible for:
      - Receiving OccupancyGrid messages from the /map topic and updating the latest_map variable
      - At fixed intervals (configurable, e.g. 0.5 Hz), converting the latest map to JSON
      - Compressing the JSON using zlib and performing base64 encoding
      - Publishing the result to the /string/map topic
    """
    def __init__(self):
        self.debug = rospy.get_param("~debug", False)
        self.publish_period = rospy.get_param("~publish_period", 5)  # Publish rate in Hz (default: 0.5 Hz)
        self.publisher = rospy.Publisher("/string/map", String, queue_size=10)
        self.subscriber = rospy.Subscriber("/map", OccupancyGrid, self.callback)
        
        # Variable to hold the latest received map message
        self.latest_map = None
        
        # Timer to process and publish map data at the specified interval
        timer_period = self.publish_period
        self.timer = rospy.Timer(rospy.Duration(timer_period), self.timer_callback)
        
        rospy.loginfo("MapCompressor has been initialized with publish period: {} Hz".format(self.publish_period))
    
    def callback(self, msg):
        # Update the latest_map variable with the incoming message
        self.latest_map = msg
        if self.debug:
            rospy.loginfo("Received new map update.")
    
    def timer_callback(self, event):
        # Process and publish only if a map has been received
        if self.latest_map is None:
            if self.debug:
                rospy.loginfo("No map received yet; skipping compression.")
            return
        
        try:
            start_time = time.time()
            
            # Convert the latest OccupancyGrid message to dictionary
            msg_dict = message_converter.convert_ros_message_to_dictionary(self.latest_map)
            # Serialize dictionary to JSON string
            json_str = json.dumps(msg_dict)
            # Compress the JSON string using zlib (lossless compression)
            compressed_bytes = zlib.compress(json_str.encode('utf-8'), level=9)
            # Encode compressed data to base64 to ensure it's a safe string for MQTT
            b64_encoded = base64.b64encode(compressed_bytes).decode('utf-8')
            
            execution_time = time.time() - start_time
            
            # Publish the compressed string
            self.publisher.publish(String(data=b64_encoded))
            
            if self.debug:
                rospy.loginfo("Original size: {} bytes, Compressed size: {} bytes".format(len(json_str), len(b64_encoded)))
                rospy.loginfo("Compression execution time: {:.6f} seconds".format(execution_time))
        except Exception as e:
            rospy.logerr("Error during map compression process: {}".format(e))

class MapDecompressor:
    """
    This class is responsible for:
      - Receiving compressed strings from the /string/map topic
      - Performing base64 decoding and decompression to retrieve the original JSON string
      - Converting the JSON string to a dictionary and transforming it into an OccupancyGrid message
      - Publishing the OccupancyGrid message to the /server/slam/map topic
    """
    def __init__(self):
        # Debug parameter
        self.debug = rospy.get_param("~debug", False)
        self.publisher = rospy.Publisher("/server/slam/map", OccupancyGrid, queue_size=10)
        self.subscriber = rospy.Subscriber("/string/map", String, self.callback)
        rospy.loginfo("MapDecompressor has been initialized.")
        
    def callback(self, msg):
        try:
            # Decode string from base64
            compressed_bytes = base64.b64decode(msg.data)
            # Decompress using zlib
            json_str = zlib.decompress(compressed_bytes).decode('utf-8')
            # Convert JSON string to dictionary
            msg_dict = json.loads(json_str)
            # Convert dictionary to OccupancyGrid message
            occupancy_msg = message_converter.convert_dictionary_to_ros_message('nav_msgs/OccupancyGrid', msg_dict)
            self.publisher.publish(occupancy_msg)
            if self.debug:
                rospy.loginfo("Decompressed and published map message.")
        except Exception as e:
            rospy.logerr("Error during map decompression process: {}".format(e))

def main():
    rospy.init_node("map_compression_node", anonymous=False)
    # The mode parameter determines whether the node operates as a compressor or decompressor
    mode = rospy.get_param("~mode", "compress")
    
    if mode == "compress":
        rospy.loginfo("Node is running in Compressor mode.")
        MapCompressor()
    elif mode == "decompress":
        rospy.loginfo("Node is running in Decompressor mode.")
        MapDecompressor()
    else:
        rospy.logerr("Invalid mode parameter. Use 'compress' or 'decompress'.")
        return
    rospy.spin()

if __name__ == "__main__":
    main()
