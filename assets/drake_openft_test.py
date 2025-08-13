#!/usr/bin/env python3

import numpy as np
import argparse
import time
import os
from pydrake.all import (
    MultibodyPlant,
    Parser,
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    MeshcatVisualizer,
    StartMeshcat,
    RigidTransform,
    RotationMatrix,
    RollPitchYaw,
    Simulator,
    DoDifferentialInverseKinematics,
    DifferentialInverseKinematicsStatus,
    DifferentialInverseKinematicsParameters,
    InverseKinematics,
    Solve,
    Box,
    Sphere,
    Cylinder,
    Rgba,
)


class XArmOpenFTController:
    def __init__(self, use_ik_control=False):
        # Start meshcat first
        self.meshcat = StartMeshcat()
        
        # Store control mode
        self.use_ik_control = use_ik_control
        
        # Setup the simulation
        self.setup_simulation()
        
        # Initial publish to show robot
        self.diagram.ForcedPublish(self.diagram_context)
        
        # Setup controls based on mode
        if self.use_ik_control:
            self.setup_ik_controls()
        else:
            self.setup_joint_controls()
        
        # Create visualizations for both modes
        self.create_gripper_com_visualization()
        self.create_openft_frame_visualization()
        self.create_world_frame_visualization()
        self.create_force_vector_visualization()
            
        # Publish again after setting up controls
        self.diagram.ForcedPublish(self.diagram_context)
        
        print(f"\n{'='*60}")
        print(f"Meshcat URL: {self.meshcat.web_url()}")
        print(f"Open this URL in your browser to view the robot")
        print(f"{'='*60}\n")
    
    def setup_simulation(self):
        """Setup Drake simulation with the xarm6_openft_gripper robot."""
        # Clear meshcat
        self.meshcat.Delete()
        self.meshcat.DeleteAddedControls()
        
        # Create diagram builder
        self.builder = DiagramBuilder()
        
        # Create plant and scene graph
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            self.builder, time_step=0.001  # Discrete time for mimic joints
        )
        
        # Parse URDF
        parser = Parser(self.plant)
        package_path = os.path.dirname(os.path.abspath(__file__))
        
        # Add package path for mesh loading
        parser.package_map().Add("dim_cpp", os.path.join(package_path, "dim_cpp"))
        
        # Load the URDF (using the version with .obj files)
        urdf_path = os.path.join(package_path, "xarm6_openft_gripper.urdf")
        self.model_instances = parser.AddModels(urdf_path)
        self.model_instance = self.model_instances[0] if self.model_instances else None
        
        # The URDF already has a world_joint connecting to world, so no need to weld
        
        # Get end-effector frame
        try:
            self.end_effector_frame = self.plant.GetFrameByName("link_tcp")
            self.end_effector_body = self.plant.GetBodyByName("link_tcp")
            print("Using link_tcp as end-effector frame")
        except:
            try:
                self.end_effector_frame = self.plant.GetFrameByName("link6")
                self.end_effector_body = self.plant.GetBodyByName("link6")
                print("Using link6 as end-effector frame")
            except:
                print("Warning: Could not find end-effector frame")
                self.end_effector_frame = None
                self.end_effector_body = None
        
        # Get gripper COM frame for visualization
        try:
            self.gripper_com_body = self.plant.GetBodyByName("xarm_gripper_com")
            print("Found xarm_gripper_com link for visualization")
        except:
            print("Warning: Could not find xarm_gripper_com link")
            self.gripper_com_body = None
        
        # Get link_openft for force/torque calculations
        try:
            self.openft_body = self.plant.GetBodyByName("link_openft")
            print("Found link_openft for force/torque calculations")
        except:
            print("Warning: Could not find link_openft")
            self.openft_body = None
        
        # Gripper parameters
        self.gripper_mass = 0.806  # kg
        self.gravity = np.array([0, 0, -9.81])  # m/s^2 in world frame
        
        # Finalize the plant
        self.plant.Finalize()
        
        # Add visualizer
        self.visualizer = MeshcatVisualizer.AddToBuilder(
            self.builder, self.scene_graph, self.meshcat
        )
        
        # Build the diagram
        self.diagram = self.builder.Build()
        
        # Create contexts
        self.diagram_context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyContextFromRoot(self.diagram_context)
        
        # Get joint names
        self.arm_joint_names = [f"joint{i+1}" for i in range(6)]
        self.gripper_joint_name = "drive_joint"
        
        # Set initial positions to avoid singularities
        initial_positions = np.zeros(self.plant.num_positions())
        # Set specific initial configuration to avoid gimbal lock
        initial_joint_angles = [0.0, -0.5, -0.5, 0.0, -0.5, 0.0]  # Slightly bent elbow configuration
        for i, joint_name in enumerate(self.arm_joint_names):
            try:
                joint = self.plant.GetJointByName(joint_name)
                joint_index = joint.position_start()
                if i < len(initial_joint_angles):
                    initial_positions[joint_index] = initial_joint_angles[i]
            except:
                pass
        
        self.plant.SetPositions(self.plant_context, initial_positions)
        
        # Reset camera for better view
        self.meshcat.SetCameraPose(
            camera_in_world=[2.0, 2.0, 1.5],
            target_in_world=[0.0, 0.0, 0.3]
        )
    
    def setup_joint_controls(self):
        """Setup sliders for direct joint control."""
        print("Setting up joint control sliders...")
        
        # Create joint sliders
        for i, joint_name in enumerate(self.arm_joint_names):
            joint = self.plant.GetJointByName(joint_name)
            lower = joint.position_lower_limit()
            upper = joint.position_upper_limit()
            # Handle single DOF joints
            if hasattr(lower, '__getitem__'):
                lower = lower[0]
                upper = upper[0]
            
            # Get current position
            current_pos = self.plant.GetPositions(self.plant_context)[joint.position_start()]
            
            # Add slider with current position as default
            self.meshcat.AddSlider(
                name=joint_name,
                min=lower,
                max=upper,
                step=0.01,
                value=current_pos
            )
            print(f"  Added slider for {joint_name}: [{lower:.2f}, {upper:.2f}]")
        
        # Gripper slider
        gripper_joint = self.plant.GetJointByName(self.gripper_joint_name)
        g_lower = gripper_joint.position_lower_limit()
        g_upper = gripper_joint.position_upper_limit()
        if hasattr(g_lower, '__getitem__'):
            g_lower = g_lower[0]
            g_upper = g_upper[0]
        
        self.meshcat.AddSlider(
            name=self.gripper_joint_name,
            min=g_lower,
            max=g_upper,
            step=0.01,
            value=0.0
        )
        print(f"  Added slider for {self.gripper_joint_name}: [{g_lower:.2f}, {g_upper:.2f}]")
    
    def setup_ik_controls(self):
        """Setup sliders for inverse kinematics control."""
        print("Setting up IK control sliders...")
        
        # Get current end-effector pose
        if self.end_effector_body:
            ee_pose = self.plant.EvalBodyPoseInWorld(
                self.plant_context, self.end_effector_body
            )
            current_pos = ee_pose.translation()
            rpy = RollPitchYaw(ee_pose.rotation())
        else:
            current_pos = np.array([0.3, 0.0, 0.3])
            rpy = RollPitchYaw(0, 0, 0)
        
        # Position sliders
        self.meshcat.AddSlider("target_x", -0.8, 0.8, 0.01, current_pos[0])
        self.meshcat.AddSlider("target_y", -0.8, 0.8, 0.01, current_pos[1])
        self.meshcat.AddSlider("target_z", 0.0, 1.0, 0.01, current_pos[2])
        print(f"  Added position sliders (x,y,z)")
        
        # Orientation sliders (in radians)
        self.meshcat.AddSlider("target_roll", -np.pi, np.pi, 0.01, rpy.roll_angle())
        self.meshcat.AddSlider("target_pitch", -np.pi, np.pi, 0.01, rpy.pitch_angle())
        self.meshcat.AddSlider("target_yaw", -np.pi, np.pi, 0.01, rpy.yaw_angle())
        print(f"  Added orientation sliders (roll,pitch,yaw)")
        
        # Gripper slider
        self.meshcat.AddSlider("gripper", 0.0, 0.85, 0.01, 0.0)
        print(f"  Added gripper slider")
        
        # Setup differential IK parameters
        self.diff_ik_params = DifferentialInverseKinematicsParameters(
            self.plant.num_positions(),
            self.plant.num_velocities()
        )
        
        # Set timestep
        self.dt = 0.05
        self.diff_ik_params.set_time_step(self.dt)
        
        # Set joint limits
        q_lower = self.plant.GetPositionLowerLimits()
        q_upper = self.plant.GetPositionUpperLimits()
        self.diff_ik_params.set_joint_position_limits((q_lower, q_upper))
        
        # Set velocity limits
        v_lower = self.plant.GetVelocityLowerLimits()
        v_upper = self.plant.GetVelocityUpperLimits()
        self.diff_ik_params.set_joint_velocity_limits((v_lower, v_upper))
        
        # Set end-effector velocity gain (higher = more aggressive tracking)
        self.diff_ik_params.set_end_effector_velocity_gain(2.0)
        
        # Enable all 6 DOF for end-effector control (3 translation + 3 rotation)
        # This ensures all rotational DOF are properly controlled
        velocity_flag = np.ones(6, dtype=bool)  # [angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]
        self.diff_ik_params.set_end_effector_velocity_flag(velocity_flag)
        
        # Create target visualization
        self.create_target_visualization()
    
    def create_target_visualization(self):
        """Create visualization for the target pose."""
        axis_length = 0.15
        axis_radius = 0.005
        
        # X-axis (red)
        self.meshcat.SetObject(
            "target_frame/x_axis",
            Box([axis_length, axis_radius * 2, axis_radius * 2]),
            Rgba(1, 0, 0, 0.7)
        )
        self.meshcat.SetTransform(
            "target_frame/x_axis",
            RigidTransform([axis_length/2, 0, 0])
        )
        
        # Y-axis (green)
        self.meshcat.SetObject(
            "target_frame/y_axis",
            Box([axis_radius * 2, axis_length, axis_radius * 2]),
            Rgba(0, 1, 0, 0.7)
        )
        self.meshcat.SetTransform(
            "target_frame/y_axis",
            RigidTransform([0, axis_length/2, 0])
        )
        
        # Z-axis (blue)
        self.meshcat.SetObject(
            "target_frame/z_axis",
            Box([axis_radius * 2, axis_radius * 2, axis_length]),
            Rgba(0, 0, 1, 0.7)
        )
        self.meshcat.SetTransform(
            "target_frame/z_axis",
            RigidTransform([0, 0, axis_length/2])
        )
        
        # Origin cube
        self.meshcat.SetObject(
            "target_frame/origin",
            Box([0.02, 0.02, 0.02]),
            Rgba(0.5, 0.5, 0.5, 0.5)
        )
    
    def create_gripper_com_visualization(self):
        """Create visualization axes for the xarm_gripper_com link."""
        if not self.gripper_com_body:
            return
            
        axis_length = 0.1
        axis_radius = 0.003
        
        # X-axis (red) - slightly darker/different shade to distinguish from target
        self.meshcat.SetObject(
            "gripper_com_frame/x_axis",
            Box([axis_length, axis_radius * 2, axis_radius * 2]),
            Rgba(0.8, 0.2, 0.2, 0.9)
        )
        self.meshcat.SetTransform(
            "gripper_com_frame/x_axis",
            RigidTransform([axis_length/2, 0, 0])
        )
        
        # Y-axis (green) - slightly darker/different shade
        self.meshcat.SetObject(
            "gripper_com_frame/y_axis",
            Box([axis_radius * 2, axis_length, axis_radius * 2]),
            Rgba(0.2, 0.8, 0.2, 0.9)
        )
        self.meshcat.SetTransform(
            "gripper_com_frame/y_axis",
            RigidTransform([0, axis_length/2, 0])
        )
        
        # Z-axis (blue) - slightly darker/different shade
        self.meshcat.SetObject(
            "gripper_com_frame/z_axis",
            Box([axis_radius * 2, axis_radius * 2, axis_length]),
            Rgba(0.2, 0.2, 0.8, 0.9)
        )
        self.meshcat.SetTransform(
            "gripper_com_frame/z_axis",
            RigidTransform([0, 0, axis_length/2])
        )
        
        # Small sphere at origin to mark the COM position
        self.meshcat.SetObject(
            "gripper_com_frame/origin",
            Sphere(0.01),
            Rgba(1.0, 1.0, 0.0, 0.8)  # Yellow sphere
        )
        
        print("Created gripper COM axes visualization (darker colors)")
    
    def create_openft_frame_visualization(self):
        """Create visualization axes for the link_openft frame."""
        if not self.openft_body:
            return
            
        axis_length = 0.12
        axis_radius = 0.004
        
        # X-axis (red) - distinct shade for OpenFT
        self.meshcat.SetObject(
            "openft_frame/x_axis",
            Box([axis_length, axis_radius * 2, axis_radius * 2]),
            Rgba(1.0, 0.3, 0.3, 0.9)
        )
        self.meshcat.SetTransform(
            "openft_frame/x_axis",
            RigidTransform([axis_length/2, 0, 0])
        )
        
        # Y-axis (green) - distinct shade for OpenFT
        self.meshcat.SetObject(
            "openft_frame/y_axis",
            Box([axis_radius * 2, axis_length, axis_radius * 2]),
            Rgba(0.3, 1.0, 0.3, 0.9)
        )
        self.meshcat.SetTransform(
            "openft_frame/y_axis",
            RigidTransform([0, axis_length/2, 0])
        )
        
        # Z-axis (blue) - distinct shade for OpenFT
        self.meshcat.SetObject(
            "openft_frame/z_axis",
            Box([axis_radius * 2, axis_radius * 2, axis_length]),
            Rgba(0.3, 0.3, 1.0, 0.9)
        )
        self.meshcat.SetTransform(
            "openft_frame/z_axis",
            RigidTransform([0, 0, axis_length/2])
        )
        
        # Small cube at origin to mark the OpenFT sensor location
        self.meshcat.SetObject(
            "openft_frame/origin",
            Box([0.015, 0.015, 0.015]),
            Rgba(0.7, 0.7, 0.7, 0.8)  # Gray cube
        )
        
        print("Created OpenFT frame axes visualization (lighter colors)")
    
    def create_world_frame_visualization(self):
        """Create visualization axes for the world frame at origin."""
        axis_length = 0.3
        axis_radius = 0.005
        
        # X-axis (red) - thicker for world frame
        self.meshcat.SetObject(
            "world_frame/x_axis",
            Box([axis_length, axis_radius * 2, axis_radius * 2]),
            Rgba(1, 0, 0, 0.5)
        )
        self.meshcat.SetTransform(
            "world_frame/x_axis",
            RigidTransform([axis_length/2, 0, 0])
        )
        
        # Y-axis (green) - thicker for world frame
        self.meshcat.SetObject(
            "world_frame/y_axis",
            Box([axis_radius * 2, axis_length, axis_radius * 2]),
            Rgba(0, 1, 0, 0.5)
        )
        self.meshcat.SetTransform(
            "world_frame/y_axis",
            RigidTransform([0, axis_length/2, 0])
        )
        
        # Z-axis (blue) - thicker for world frame
        self.meshcat.SetObject(
            "world_frame/z_axis",
            Box([axis_radius * 2, axis_radius * 2, axis_length]),
            Rgba(0, 0, 1, 0.5)
        )
        self.meshcat.SetTransform(
            "world_frame/z_axis",
            RigidTransform([0, 0, axis_length/2])
        )
        
        # Add labels
        print("Created world frame axes visualization at origin")
    
    def create_force_vector_visualization(self):
        """Create visualization for the gravity force vector."""
        # Force vector (pointing down from COM)
        self.force_scale = 0.025  # Scale factor: 0.025 m per Newton (so ~0.2m for 8N)
        force_length = abs(self.gripper_mass * self.gravity[2]) * self.force_scale  # Length proportional to force
        
        # Create arrow shaft (cylinder)
        self.meshcat.SetObject(
            "force_vector/arrow",
            Cylinder(0.004, force_length),  # radius, length
            Rgba(1.0, 0.5, 0.0, 0.8)  # Orange color for force
        )
        
        # Create arrow head (cone using a tapered box or sphere)
        self.meshcat.SetObject(
            "force_vector/head",
            Sphere(0.01),  # Small sphere for arrow tip
            Rgba(1.0, 0.5, 0.0, 0.9)  # Slightly more opaque orange
        )
        
        print(f"Created force vector visualization (scaled: {self.force_scale} m/N)")
    
    def update_force_vector(self):
        """Update the position and orientation of the force vector."""
        if not self.gripper_com_body:
            return
            
        # Get COM position
        com_pose_world = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.gripper_com_body
        )
        com_pos = com_pose_world.translation()
        
        # Calculate force vector length based on actual force magnitude
        force_magnitude = abs(self.gripper_mass * self.gravity[2])  # Should be ~7.9 N
        force_length = force_magnitude * self.force_scale
        
        # Force vector points straight down (gravity)
        # The cylinder's default orientation is along the Z-axis
        # We need to position it to start at COM and extend downward
        
        # Position the cylinder's center halfway down from COM
        arrow_center = com_pos + np.array([0, 0, -force_length/2])
        
        # No rotation needed - cylinder default orientation is along Z
        # Just translate it to the right position
        force_transform = RigidTransform(
            RotationMatrix(),  # Identity rotation (cylinder already points along Z)
            arrow_center
        )
        self.meshcat.SetTransform("force_vector/arrow", force_transform)
        
        # Position arrow head at the tip (end of force vector)
        head_position = com_pos + np.array([0, 0, -force_length])
        head_transform = RigidTransform(
            RotationMatrix(),
            head_position
        )
        self.meshcat.SetTransform("force_vector/head", head_transform)
    
    def calculate_openft_wrench(self):
        """Calculate forces and torques on link_openft due to gripper weight at COM.
        
        Note: The force in world frame is always [0, 0, -mg] because gravity 
        always points straight down regardless of robot orientation. The torque
        changes based on the lever arm from link_openft to the COM.
        """
        if not self.gripper_com_body or not self.openft_body:
            return None, None
        
        # Get poses in world frame
        com_pose_world = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.gripper_com_body
        )
        openft_pose_world = self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.openft_body
        )
        
        # Force due to gravity (in world frame)
        # This is ALWAYS [0, 0, -mg] in world frame because gravity points down
        force_world = self.gripper_mass * self.gravity  # F = mg = [0, 0, -7.907] N
        
        # Position of COM relative to openft origin (in world frame)
        r_com_to_openft = com_pose_world.translation() - openft_pose_world.translation()
        
        # Torque = r × F (cross product of position vector and force)
        # This changes based on robot configuration
        torque_world = np.cross(r_com_to_openft, force_world)
        
        return force_world, torque_world
    
    def run_joint_control(self):
        """Run the joint control mode."""
        print("\n" + "="*60)
        print("Joint Control Mode Active")
        print("Use the sliders in Meshcat to control individual joints")
        print("Press Ctrl+C to exit")
        print("="*60 + "\n")
        
        iteration_count = 0
        print_interval = 100  # Print every 100 iterations (about 1 second at 10ms delay)
        
        try:
            while True:
                iteration_count += 1
                # Get current positions (all DOF)
                current_positions = self.plant.GetPositions(self.plant_context)
                
                # Update arm joint positions from sliders
                for joint_name in self.arm_joint_names:
                    try:
                        joint = self.plant.GetJointByName(joint_name)
                        joint_index = joint.position_start()
                        value = self.meshcat.GetSliderValue(joint_name)
                        current_positions[joint_index] = value
                    except:
                        pass
                
                # Update gripper position
                try:
                    gripper_joint = self.plant.GetJointByName(self.gripper_joint_name)
                    gripper_index = gripper_joint.position_start()
                    gripper_value = self.meshcat.GetSliderValue(self.gripper_joint_name)
                    current_positions[gripper_index] = gripper_value
                except:
                    pass
                
                # Set all positions
                self.plant.SetPositions(self.plant_context, current_positions)
                
                # Update gripper COM axes visualization
                if self.gripper_com_body:
                    gripper_com_pose = self.plant.EvalBodyPoseInWorld(
                        self.plant_context, self.gripper_com_body
                    )
                    self.meshcat.SetTransform("gripper_com_frame", gripper_com_pose)
                    
                    # Update force vector visualization
                    self.update_force_vector()
                
                # Update OpenFT frame axes visualization
                if self.openft_body:
                    openft_pose = self.plant.EvalBodyPoseInWorld(
                        self.plant_context, self.openft_body
                    )
                    self.meshcat.SetTransform("openft_frame", openft_pose)
                
                # Calculate and print forces/torques periodically
                if iteration_count % print_interval == 0:
                    force_world, torque_world = self.calculate_openft_wrench()
                    if force_world is not None and self.openft_body:
                        print(f"\n--- OpenFT Forces & Torques ---")
                        print(f"WORLD FRAME:")
                        print(f"  Force:  [{force_world[0]:7.3f}, {force_world[1]:7.3f}, {force_world[2]:7.3f}] N")
                        print(f"  Torque: [{torque_world[0]:7.4f}, {torque_world[1]:7.4f}, {torque_world[2]:7.4f}] N⋅m")
                        
                        # Transform to OpenFT local frame
                        openft_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.openft_body)
                        R_world_to_openft = openft_pose.rotation().transpose()
                        force_openft = R_world_to_openft @ force_world
                        torque_openft = R_world_to_openft @ torque_world
                        
                        print(f"OPENFT LOCAL FRAME:")
                        print(f"  Force:  [{force_openft[0]:7.3f}, {force_openft[1]:7.3f}, {force_openft[2]:7.3f}] N")
                        print(f"  Torque: [{torque_openft[0]:7.4f}, {torque_openft[1]:7.4f}, {torque_openft[2]:7.4f}] N⋅m")
                        print(f"  Force magnitude:  {np.linalg.norm(force_openft):.3f} N")
                        print(f"  Torque magnitude: {np.linalg.norm(torque_openft):.4f} N⋅m")
                
                # Publish visualization
                self.diagram.ForcedPublish(self.diagram_context)
                
                # Small delay
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nShutting down...")
    
    def run_ik_control(self):
        """Run the inverse kinematics control mode."""
        print("\n" + "="*60)
        print("Inverse Kinematics Control Mode Active")
        print("Use the target pose sliders in Meshcat to control the end-effector")
        print("  - target_x/y/z: End-effector position (meters)")
        print("  - target_roll/pitch/yaw: End-effector orientation (radians)")
        print("  - gripper: Gripper opening (0=closed, 0.85=open)")
        print("Press Ctrl+C to exit")
        print("="*60 + "\n")
        
        if not self.end_effector_frame:
            print("Error: No end-effector frame found!")
            return
        
        iteration_count = 0
        print_interval = 20  # Print every 20 iterations (about 1 second at 50ms delay)
        
        try:
            while True:
                iteration_count += 1
                # Read target pose from sliders
                target_x = self.meshcat.GetSliderValue("target_x")
                target_y = self.meshcat.GetSliderValue("target_y")
                target_z = self.meshcat.GetSliderValue("target_z")
                target_roll = self.meshcat.GetSliderValue("target_roll")
                target_pitch = self.meshcat.GetSliderValue("target_pitch")
                target_yaw = self.meshcat.GetSliderValue("target_yaw")
                gripper_value = self.meshcat.GetSliderValue("gripper")
                
                # Create target transform
                target_position = np.array([target_x, target_y, target_z])
                target_rotation = RotationMatrix(RollPitchYaw(target_roll, target_pitch, target_yaw))
                target_transform = RigidTransform(target_rotation, target_position)
                
                # Update target visualization
                self.meshcat.SetTransform("target_frame", target_transform)
                
                # Debug: Check if we're in a singularity or gimbal lock situation
                if iteration_count % print_interval == 0:
                    ee_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.end_effector_body)
                    ee_rpy = RollPitchYaw(ee_pose.rotation())
                    print(f"\n[DEBUG] Current EE: R={ee_rpy.roll_angle():.3f}, P={ee_rpy.pitch_angle():.3f}, Y={ee_rpy.yaw_angle():.3f}")
                    print(f"[DEBUG] Target:     R={target_roll:.3f}, P={target_pitch:.3f}, Y={target_yaw:.3f}")
                    
                    # Check joint configuration
                    q = self.plant.GetPositions(self.plant_context)
                    print(f"[DEBUG] Joint angles: {q[:6]}")
                
                # Use differential IK to move towards target
                result = DoDifferentialInverseKinematics(
                    self.plant,
                    self.plant_context,
                    target_transform,
                    self.end_effector_frame,
                    self.diff_ik_params
                )
                
                if result.status == DifferentialInverseKinematicsStatus.kSolutionFound:
                    # Get current positions
                    q_current = self.plant.GetPositions(self.plant_context)
                    
                    # Integrate velocities
                    v_sol = result.joint_velocities
                    q_new = q_current + v_sol.flatten() * self.dt
                    
                    # Set gripper separately (at its correct index)
                    try:
                        gripper_joint = self.plant.GetJointByName(self.gripper_joint_name)
                        gripper_index = gripper_joint.position_start()
                        q_new[gripper_index] = gripper_value
                    except:
                        pass
                    
                    # Apply joint limits
                    q_lower = self.plant.GetPositionLowerLimits()
                    q_upper = self.plant.GetPositionUpperLimits()
                    q_new = np.clip(q_new, q_lower, q_upper)
                    
                    # Set new positions
                    self.plant.SetPositions(self.plant_context, q_new)
                
                # Update gripper COM axes visualization
                if self.gripper_com_body:
                    gripper_com_pose = self.plant.EvalBodyPoseInWorld(
                        self.plant_context, self.gripper_com_body
                    )
                    self.meshcat.SetTransform("gripper_com_frame", gripper_com_pose)
                    
                    # Update force vector visualization
                    self.update_force_vector()
                
                # Update OpenFT frame axes visualization
                if self.openft_body:
                    openft_pose = self.plant.EvalBodyPoseInWorld(
                        self.plant_context, self.openft_body
                    )
                    self.meshcat.SetTransform("openft_frame", openft_pose)
                
                # Calculate and print forces/torques periodically
                if iteration_count % print_interval == 0:
                    force_world, torque_world = self.calculate_openft_wrench()
                    if force_world is not None and self.openft_body:
                        print(f"\n--- OpenFT Forces & Torques ---")
                        print(f"WORLD FRAME:")
                        print(f"  Force:  [{force_world[0]:7.3f}, {force_world[1]:7.3f}, {force_world[2]:7.3f}] N")
                        print(f"  Torque: [{torque_world[0]:7.4f}, {torque_world[1]:7.4f}, {torque_world[2]:7.4f}] N⋅m")
                        
                        # Transform to OpenFT local frame
                        openft_pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.openft_body)
                        R_world_to_openft = openft_pose.rotation().transpose()
                        force_openft = R_world_to_openft @ force_world
                        torque_openft = R_world_to_openft @ torque_world
                        
                        print(f"OPENFT LOCAL FRAME:")
                        print(f"  Force:  [{force_openft[0]:7.3f}, {force_openft[1]:7.3f}, {force_openft[2]:7.3f}] N")
                        print(f"  Torque: [{torque_openft[0]:7.4f}, {torque_openft[1]:7.4f}, {torque_openft[2]:7.4f}] N⋅m")
                        print(f"  Force magnitude:  {np.linalg.norm(force_openft):.3f} N")
                        print(f"  Torque magnitude: {np.linalg.norm(torque_openft):.4f} N⋅m")
                
                # Publish visualization
                self.diagram.ForcedPublish(self.diagram_context)
                
                # Small delay
                time.sleep(self.dt)
                
        except KeyboardInterrupt:
            print("\nShutting down...")
    
    def run(self):
        """Main run method."""
        if self.use_ik_control:
            self.run_ik_control()
        else:
            self.run_joint_control()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Drake xARM OpenFT Control Test")
    parser.add_argument(
        "--ik_control",
        action="store_true",
        help="Use inverse kinematics control mode (default: joint control)"
    )
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("Drake xARM6 OpenFT Gripper Control")
    print("="*60)
    
    # Create and run controller
    controller = XArmOpenFTController(use_ik_control=args.ik_control)
    controller.run()


if __name__ == "__main__":
    main()