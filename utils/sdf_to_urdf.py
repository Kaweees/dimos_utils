#!/usr/bin/env python3
"""
Script to convert an SDF file to URDF format, specifically focusing on
converting mesh tags with URI attributes to mesh tags with filename attributes.
"""

import argparse
import os
import xml.etree.ElementTree as ET
import re

def convert_sdf_to_urdf(sdf_file, output_file=None):
    """
    Convert SDF file to URDF format.
    
    Args:
        sdf_file (str): Path to the SDF file
        output_file (str, optional): Path to output URDF file. If None, creates a file
                                    with the same name but .urdf extension
    
    Returns:
        str: Path to the output file
    """
    if output_file is None:
        output_file = os.path.splitext(sdf_file)[0] + '.urdf'
    
    # Register drake namespace
    ET.register_namespace('drake', 'drake.mit.edu')
    
    # Parse the SDF file
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    
    # Create the URDF structure
    urdf_root = ET.Element('robot')
    model = root.find('model')
    urdf_root.set('name', model.get('name'))
    
    # Process links
    for link in model.findall('link'):
        urdf_link = ET.SubElement(urdf_root, 'link')
        urdf_link.set('name', link.get('name'))
        
        # Process inertial
        inertial = link.find('inertial')
        if inertial is not None:
            urdf_inertial = ET.SubElement(urdf_link, 'inertial')
            
            # Process mass
            mass = inertial.find('mass')
            if mass is not None:
                urdf_mass = ET.SubElement(urdf_inertial, 'mass')
                urdf_mass.set('value', mass.text.strip())
            
            # Process inertia
            inertia = inertial.find('inertia')
            if inertia is not None:
                urdf_inertia = ET.SubElement(urdf_inertial, 'inertia')
                for prop in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']:
                    value = inertia.find(prop)
                    if value is not None:
                        # Convert scientific notation to standard format if needed
                        val_text = value.text.strip()
                        if 'E' in val_text or 'e' in val_text:
                            val_text = str(float(val_text))
                        urdf_inertia.set(prop, val_text)
        
        # Process pose (becomes origin in URDF)
        pose = link.find('pose')
        if pose is not None:
            pose_values = pose.text.strip().split()
            # URDF origin format: x y z roll pitch yaw
            if len(pose_values) == 6:
                for visual in link.findall('visual'):
                    urdf_visual = process_visual_or_collision(visual, urdf_link, 'visual', pose_values)
                for collision in link.findall('collision'):
                    urdf_collision = process_visual_or_collision(collision, urdf_link, 'collision', pose_values)
        else:
            # No pose element, just process visuals and collisions
            for visual in link.findall('visual'):
                urdf_visual = process_visual_or_collision(visual, urdf_link, 'visual')
            for collision in link.findall('collision'):
                urdf_collision = process_visual_or_collision(collision, urdf_link, 'collision')
    
    # Write the URDF to file with proper indentation
    # Format the XML with proper indentation for readability
    rough_string = ET.tostring(urdf_root, 'utf-8')
    
    # Use a proper XML pretty printer for correct indentation
    from xml.dom import minidom
    reparsed = minidom.parseString(rough_string)
    # Fix for minidom's formatting issues with whitespace
    formatted_xml = reparsed.toprettyxml(indent='  ')
    # Remove extra blank lines that minidom sometimes adds
    formatted_xml = re.sub(r'\n\s*\n', '\n', formatted_xml)
    
    with open(output_file, 'w') as f:
        f.write(formatted_xml)
    
    return output_file

def process_visual_or_collision(element, parent, element_type, pose_values=None):
    """
    Process visual or collision elements.
    
    Args:
        element: The visual or collision XML element
        parent: The parent link element in URDF
        element_type: 'visual' or 'collision'
        pose_values: Optional pose values to use
        
    Returns:
        The created URDF element
    """
    urdf_element = ET.SubElement(parent, element_type)
    
    # Set the name if it exists
    if 'name' in element.attrib:
        urdf_element.set('name', element.get('name'))
    
    # Set origin if pose values are available
    if pose_values:
        origin = ET.SubElement(urdf_element, 'origin')
        origin.set('xyz', f"{pose_values[0]} {pose_values[1]} {pose_values[2]}")
        origin.set('rpy', f"{pose_values[3]} {pose_values[4]} {pose_values[5]}")
    
    # Process geometry
    geometry_elem = element.find('geometry')
    if geometry_elem is not None:
        urdf_geometry = ET.SubElement(urdf_element, 'geometry')
        
        # Process mesh
        mesh_elem = geometry_elem.find('mesh')
        if mesh_elem is not None:
            urdf_mesh = ET.SubElement(urdf_geometry, 'mesh')
            
            # Convert URI to filename
            uri_elem = mesh_elem.find('uri')
            if uri_elem is not None:
                urdf_mesh.set('filename', uri_elem.text.strip())
            
            # Handle scale
            scale_elem = mesh_elem.find('scale')
            if scale_elem is not None:
                scale_values = scale_elem.text.strip().split()
                if len(scale_values) == 3:
                    urdf_mesh.set('scale', f"{scale_values[0]} {scale_values[1]} {scale_values[2]}")
            
            # If there's a drake:declare_convex, we need to handle it specially
            # In URDF, we can use a drake tag for this
            convex_elem = mesh_elem.find('{drake.mit.edu}declare_convex')
            if convex_elem is not None:
                drake_convex = ET.SubElement(urdf_mesh, '{drake.mit.edu}declare_convex')
    
    return urdf_element

def main():
    parser = argparse.ArgumentParser(description='Convert SDF to URDF format')
    parser.add_argument('sdf_file', help='Path to the SDF file')
    parser.add_argument('--output', '-o', help='Path to output URDF file')
    args = parser.parse_args()
    
    output_file = convert_sdf_to_urdf(args.sdf_file, args.output)
    print(f"SDF file converted to URDF: {output_file}")

if __name__ == '__main__':
    main()
