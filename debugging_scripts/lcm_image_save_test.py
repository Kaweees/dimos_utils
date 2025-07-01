#!/usr/bin/env python3
"""
LCM Image Saver - Subscribes to an LCM Image topic and saves received images as PNG files.
Usage: python3 lcm_image_save_test.py [topic_name]
Default topic: head_cam_rgb#sensor_msgs.Image
"""

import os
import sys
import time
import signal
import threading
import argparse
import cv2
import numpy as np
import lcm
from datetime import datetime

# Import Image message type
from lcm_msgs.sensor_msgs import Image

class LcmImageSaver:
    def __init__(self, topic_name="head_cam_rgb#sensor_msgs.Image", output_dir=None):
        """
        Initialize the LCM image saver
        
        Args:
            topic_name: LCM topic to subscribe to
            output_dir: Directory to save images (default: current directory)
        """
        self.topic_name = topic_name
        self.running = True
        self.received_images = 0
        
        # Initialize LCM
        self.lc = lcm.LCM()
        
        # Set up output directory
        if output_dir:
            self.output_dir = output_dir
        else:
            self.output_dir = os.getcwd()
            
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Setup signal handling for graceful exit
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def image_callback(self, channel, data):
        """
        Callback when an LCM image message is received
        
        Args:
            channel: LCM channel name
            data: Raw LCM message data
        """
        try:
            # Decode the image message
            msg = Image.decode(data)
            
            # Generate a filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = f"{self.output_dir}/lcm_image_{timestamp}.png"
            
            # Get image dimensions and encoding
            height = msg.height
            width = msg.width
            encoding = msg.encoding
            
            print(f"\n[RECEIVED] Image on channel: {channel}")
            print(f"Dimensions: {width}x{height}, Encoding: {encoding}")
            
            # Convert the data to a numpy array based on encoding
            try:
                if encoding.startswith('mono') or encoding == 'uint8':
                    # Grayscale image
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width))
                elif encoding in ['rgb8', 'bgr8']:
                    # 3-channel color image
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                    # Convert if needed
                    if encoding == 'rgb8':
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                elif encoding == 'bgra8' or encoding == 'rgba8':
                    # 4-channel color image with alpha
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 4))
                    # Convert if needed
                    if encoding == 'rgba8':
                        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA)
                else:
                    # Default to bgr8
                    print(f"Warning: Unknown encoding: {encoding}, defaulting to bgr8")
                    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))
                    
                # Save the image
                cv2.imwrite(filename, img)
                self.received_images += 1
                print(f"Saved image as: {filename}")
                
            except ValueError as e:
                print(f"Error converting image data: {e}")
                print(f"Message info: {width}x{height}, encoding={encoding}, data_length={msg.data_length}")
                
        except Exception as e:
            print(f"Error processing image on channel {channel}: {e}")
            import traceback
            traceback.print_exc()
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C for clean exit"""
        print("\nExiting gracefully...")
        self.running = False
        
    def start(self):
        """Start listening for images"""
        print(f"LCM Image Saver started. Listening for images on topic: {self.topic_name}")
        print(f"Images will be saved to: {self.output_dir}")
        print("Press Ctrl+C to exit")
        
        # Subscribe to the specified topic
        self.lc.subscribe(self.topic_name, self.image_callback)
        
        # Create a separate thread for LCM handling
        thread = threading.Thread(target=self._loop)
        thread.daemon = True
        thread.start()
        
        # Keep the main thread running
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
            
        print(f"\nTotal images saved: {self.received_images}")
        
    def _loop(self):
        """LCM message handling loop"""
        while self.running:
            try:
                self.lc.handle()
            except Exception as e:
                print(f"Error in LCM handling: {e}")

def main():
    parser = argparse.ArgumentParser(description='Save LCM Image messages as PNG files')
    parser.add_argument('topic', nargs='?', default="head_cam_rgb#sensor_msgs.Image",
                      help='LCM topic to subscribe to (default: head_cam_rgb#sensor_msgs.Image)')
    parser.add_argument('--output-dir', '-o', default=None,
                      help='Directory to save images (default: current directory)')
    args = parser.parse_args()
    
    # Create and start the image saver
    saver = LcmImageSaver(topic_name=args.topic, output_dir=args.output_dir)
    saver.start()
    
if __name__ == "__main__":
    main()
