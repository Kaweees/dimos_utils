#!/usr/bin/env python3

import os
import struct
import sys
from pathlib import Path

def read_stl_binary(file_path):
    """Read binary STL file and return triangles."""
    triangles = []
    
    with open(file_path, 'rb') as f:
        header = f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]
        
        for _ in range(num_triangles):
            normal = struct.unpack('<3f', f.read(12))
            v1 = struct.unpack('<3f', f.read(12))
            v2 = struct.unpack('<3f', f.read(12))
            v3 = struct.unpack('<3f', f.read(12))
            attribute = struct.unpack('<H', f.read(2))[0]
            
            triangles.append({
                'normal': normal,
                'vertices': [v1, v2, v3]
            })
    
    return triangles

def read_stl_ascii(file_path):
    """Read ASCII STL file and return triangles."""
    triangles = []
    
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        if line.startswith('facet normal'):
            normal = [float(x) for x in line.split()[2:5]]
            vertices = []
            
            i += 1  # Skip "outer loop"
            i += 1
            
            # Read 3 vertices
            for _ in range(3):
                vertex_line = lines[i].strip()
                if vertex_line.startswith('vertex'):
                    vertex = [float(x) for x in vertex_line.split()[1:4]]
                    vertices.append(vertex)
                i += 1
            
            triangles.append({
                'normal': normal,
                'vertices': vertices
            })
            
            i += 1  # Skip "endloop"
        
        i += 1
    
    return triangles

def is_binary_stl(file_path):
    """Check if STL file is binary or ASCII."""
    try:
        with open(file_path, 'rb') as f:
            header = f.read(80)
            # Check if it starts with "solid" but isn't actually ASCII
            if header.startswith(b'solid'):
                # Try to read as ASCII first
                with open(file_path, 'r', encoding='utf-8', errors='ignore') as f_text:
                    first_line = f_text.readline().strip()
                    if first_line.startswith('solid'):
                        return False  # ASCII
            return True  # Binary
    except:
        return False

def read_stl(file_path):
    """Read STL file (binary or ASCII) and return triangles."""
    if is_binary_stl(file_path):
        return read_stl_binary(file_path)
    else:
        return read_stl_ascii(file_path)

def write_obj(triangles, output_path):
    """Write triangles to OBJ file."""
    vertices = []
    faces = []
    vertex_map = {}
    vertex_index = 1
    
    # Extract unique vertices
    for triangle in triangles:
        face_indices = []
        for vertex in triangle['vertices']:
            vertex_tuple = tuple(vertex)
            if vertex_tuple not in vertex_map:
                vertices.append(vertex)
                vertex_map[vertex_tuple] = vertex_index
                vertex_index += 1
            face_indices.append(vertex_map[vertex_tuple])
        faces.append(face_indices)
    
    # Write OBJ file
    with open(output_path, 'w') as f:
        # Write header
        f.write(f"# OBJ file converted from STL\n")
        f.write(f"# Vertices: {len(vertices)}\n")
        f.write(f"# Faces: {len(faces)}\n\n")
        
        # Write vertices
        for vertex in vertices:
            f.write(f"v {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")
        
        f.write("\n")
        
        # Write faces
        for face in faces:
            f.write(f"f {face[0]} {face[1]} {face[2]}\n")

def convert_stl_to_obj(stl_path, obj_path):
    """Convert STL file to OBJ file."""
    try:
        triangles = read_stl(stl_path)
        write_obj(triangles, obj_path)
        return True
    except Exception as e:
        print(f"Error converting {stl_path}: {e}")
        return False

def main():
    """Main function to recursively convert STL files to OBJ."""
    current_dir = Path.cwd()
    converted_count = 0
    skipped_count = 0
    error_count = 0
    
    print(f"Searching for STL files in: {current_dir}")
    
    # Find all STL files recursively
    stl_files = list(current_dir.glob("**/*.stl"))
    
    if not stl_files:
        print("No STL files found.")
        return
    
    print(f"Found {len(stl_files)} STL files")
    
    for stl_file in stl_files:
        # Generate OBJ file path
        obj_file = stl_file.with_suffix('.obj')
        
        # Check if OBJ file already exists
        if obj_file.exists():
            print(f"Skipping {stl_file.name} - {obj_file.name} already exists")
            skipped_count += 1
            continue
        
        print(f"Converting {stl_file.relative_to(current_dir)} -> {obj_file.relative_to(current_dir)}")
        
        # Convert STL to OBJ
        if convert_stl_to_obj(stl_file, obj_file):
            converted_count += 1
        else:
            error_count += 1
    
    print(f"\nConversion complete:")
    print(f"  Converted: {converted_count}")
    print(f"  Skipped: {skipped_count}")
    print(f"  Errors: {error_count}")

if __name__ == "__main__":
    main()