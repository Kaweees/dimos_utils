#!/usr/bin/env python3
import time
import lcm
import tkinter as tk
from tkinter import ttk
import random
import argparse
import xml.etree.ElementTree as ET
from lcm_msgs.sensor_msgs import JointState
import os

class JointStatePublisher:
    def __init__(self, urdf_path):
        self.urdf_path = urdf_path
        self.joints = {}
        self.sliders = []
        self.labels = []
        self.msg = None
        self.lc = lcm.LCM()
        self.root = None
        
        self.parse_urdf()
        self.create_joint_state_msg()
        self.create_gui()
    
    def parse_urdf(self):
        try:
            tree = ET.parse(self.urdf_path)
            root = tree.getroot()
            
            for joint_elem in root.findall('.//joint'):
                joint_name = joint_elem.get('name')
                joint_type = joint_elem.get('type')
                
                if joint_type not in ['revolute', 'continuous', 'prismatic']:
                    continue
                
                limit_lower = -3.14159  # Default for revolute/continuous
                limit_upper = 3.14159
                
                if joint_type == 'prismatic':
                    limit_lower = -1.0  # Default for prismatic
                    limit_upper = 1.0
                
                limit_elem = joint_elem.find('limit')
                if limit_elem is not None:
                    if 'lower' in limit_elem.attrib:
                        limit_lower = float(limit_elem.get('lower'))
                    if 'upper' in limit_elem.attrib:
                        limit_upper = float(limit_elem.get('upper'))
                
                self.joints[joint_name] = {
                    'type': joint_type,
                    'lower': limit_lower,
                    'upper': limit_upper,
                    'position': 0.0
                }
            
            print(f"Parsed {len(self.joints)} controllable joints from URDF:")
            for name, joint in self.joints.items():
                print(f"  {name}: {joint['type']} [{joint['lower']:.3f}, {joint['upper']:.3f}]")
                
        except Exception as e:
            print(f"Error parsing URDF file: {e}")
            self.joints = {}
    
    def create_joint_state_msg(self):
        self.msg = JointState()
        self.msg.header.stamp.sec = int(time.time())
        self.msg.header.stamp.nsec = int((time.time() - int(time.time())) * 1e9)
        self.msg.header.frame_id = "base_link"
        
        joint_names = list(self.joints.keys())
        self.msg.name = joint_names
        self.msg.position = [0.0] * len(joint_names)
        self.msg.velocity = []
        self.msg.effort = []
        self.msg.name_length = len(joint_names)
        self.msg.position_length = len(self.msg.position)
        self.msg.velocity_length = 0
        self.msg.effort_length = 0
    
    def create_gui(self):
        self.root = tk.Tk()
        self.root.title(f"Joint State Publisher - {self.urdf_path}")
        
        if not self.joints:
            error_label = ttk.Label(self.root, text="No controllable joints found in URDF!")
            error_label.pack(padx=20, pady=20)
            return
        
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        for i, (joint_name, joint_info) in enumerate(self.joints.items()):
            frame = ttk.Frame(main_frame, padding="5")
            frame.grid(row=i, column=0, sticky=(tk.W, tk.E))
            
            label_text = f"{joint_name} ({joint_info['type']})"
            ttk.Label(frame, text=label_text, width=25).grid(column=0, row=0, sticky=tk.W, padx=5)
            
            slider = ttk.Scale(
                frame, 
                from_=joint_info['lower'], 
                to=joint_info['upper'], 
                orient=tk.HORIZONTAL, 
                length=300
            )
            slider.grid(column=1, row=0, sticky=(tk.W, tk.E))
            slider.set(0.0)
            
            value_label = ttk.Label(frame, text="0.000", width=8)
            value_label.grid(column=2, row=0, sticky=tk.E, padx=5)
            
            limit_label = ttk.Label(frame, text=f"[{joint_info['lower']:.2f}, {joint_info['upper']:.2f}]", width=15)
            limit_label.grid(column=3, row=0, sticky=tk.E, padx=5)
            
            def create_update_func(label, idx):
                def update_func(val):
                    label.config(text=f"{float(val):.3f}")
                    self.msg.position[idx] = float(val)
                return update_func
            
            update_func = create_update_func(value_label, i)
            slider.config(command=update_func)
            
            self.sliders.append(slider)
            self.labels.append(value_label)
        
        button_frame = ttk.Frame(main_frame, padding="5")
        button_frame.grid(row=len(self.joints), column=0, sticky=(tk.W, tk.E))
        
        reset_button = ttk.Button(button_frame, text="Reset to Zero", command=self.reset_positions)
        reset_button.grid(column=0, row=0, padx=5, pady=10)
        
        center_button = ttk.Button(button_frame, text="Center Positions", command=self.center_positions)
        center_button.grid(column=1, row=0, padx=5, pady=10)
        
        randomize_button = ttk.Button(button_frame, text="Randomize", command=self.randomize_positions)
        randomize_button.grid(column=2, row=0, padx=5, pady=10)
        
        quit_button = ttk.Button(button_frame, text="Quit", command=self.root.destroy)
        quit_button.grid(column=3, row=0, padx=5, pady=10)
        
        self.update_positions()
    
    def update_positions(self):
        self.msg.header.stamp.sec = int(time.time())
        self.msg.header.stamp.nsec = int((time.time() - int(time.time())) * 1e9)
        
        try:
            self.lc.publish("joint_states#sensor_msgs.JointState", self.msg.encode())
        except Exception as e:
            print(f"Error publishing joint state: {e}")
        
        if self.root:
            self.root.after(50, self.update_positions)
    
    def reset_positions(self):
        for i, slider in enumerate(self.sliders):
            slider.set(0.0)
            self.msg.position[i] = 0.0
    
    def center_positions(self):
        for i, (joint_name, joint_info) in enumerate(self.joints.items()):
            center_value = (joint_info['lower'] + joint_info['upper']) / 2.0
            self.sliders[i].set(center_value)
            self.msg.position[i] = center_value
    
    def randomize_positions(self):
        for i, (joint_name, joint_info) in enumerate(self.joints.items()):
            random_value = random.uniform(joint_info['lower'], joint_info['upper'])
            self.sliders[i].set(random_value)
            self.msg.position[i] = random_value
    
    def run(self):
        if self.root:
            self.root.mainloop()

def main():
    parser = argparse.ArgumentParser(description='Joint State Publisher with URDF-based limits')
    parser.add_argument('--urdf', help='Path to URDF file')

    args = parser.parse_args()

    # If URDF is not specified, look for common locations
    urdf_path = args.urdf
    file_dir = os.path.dirname(os.path.abspath(__file__))
    if not urdf_path:
        # Try to find a URDF file in common locations
        potential_paths = [
            os.path.join(file_dir, "../assets/xarm_devkit_base_descr.urdf"),
        ]
        
        for path in potential_paths:
            try:
                with open(path, 'r') as f:
                    urdf_path = path
                    print(f"Found URDF file at: {urdf_path}")
                    break
            except FileNotFoundError:
                continue
    
    if not urdf_path:
        print("Error: No URDF file specified and none found in common locations.")
        print("Please specify a URDF file using the --urdf argument.")
        return
    
    try:
        publisher = JointStatePublisher(urdf_path)
        publisher.run()
    except KeyboardInterrupt:
        print("\nShutting down joint state publisher.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()