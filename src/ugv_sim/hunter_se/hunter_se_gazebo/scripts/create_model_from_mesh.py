#!/usr/bin/env python3
"""
Script to automatically create Gazebo model from STL or DAE mesh files
Usage: python3 create_model_from_mesh.py <mesh_file_path> [model_name] [static]
"""

import os
import sys
import argparse
from pathlib import Path

def create_sdf_content(model_name, mesh_uri, is_static=True, mesh_type="stl"):
    """Create SDF content for the model"""
    
    # Set material based on mesh type
    if mesh_type.lower() == "stl":
        material = """
          <script>
            <name>Gazebo/Grey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
          <ambient>0.5 0.5 0.5 1.0</ambient>
          <diffuse>0.8 0.8 0.8 1.0</diffuse>
          <specular>0.1 0.1 0.1 1.0</specular>"""
    else:  # dae
        material = """
          <script>
            <name>Gazebo/Wood</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
          <ambient>0.6 0.4 0.2 1.0</ambient>
          <diffuse>0.8 0.6 0.4 1.0</diffuse>
          <specular>0.1 0.1 0.1 1.0</specular>"""
    
    static_str = "true" if is_static else "false"
    
    sdf_content = f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{model_name}">
    <static>{static_str}</static>
    <pose>0 0 0 0 0 0</pose>
    
    <link name="link">
      <!-- Visual -->
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>{mesh_uri}</uri>
            <scale>1.0 1.0 1.0</scale>
          </mesh>
        </geometry>
        <material>{material}
        </material>
      </visual>
      
      <!-- Collision -->
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>{mesh_uri}</uri>
            <scale>1.0 1.0 1.0</scale>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      
      <!-- Inertial -->
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>"""
    return sdf_content

def create_config_content(model_name, description=""):
    """Create model.config content"""
    config_content = f"""<?xml version="1.0"?>
<model>
  <name>{model_name.replace('_', ' ').title()}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  
  <author>
    <name>Hunter SE Project</name>
    <email>hunter@project.com</email>
  </author>
  
  <description>
    {description if description else f"Custom model loaded from mesh file"}
  </description>
</model>"""
    return config_content

def main():
    parser = argparse.ArgumentParser(description='Create Gazebo model from mesh file')
    parser.add_argument('mesh_file', help='Path to STL or DAE mesh file')
    parser.add_argument('--name', '-n', help='Model name (default: mesh filename)')
    parser.add_argument('--static', '-s', action='store_true', default=True, help='Make model static (default: True)')
    parser.add_argument('--description', '-d', help='Model description')
    
    args = parser.parse_args()
    
    # Validate mesh file
    mesh_path = Path(args.mesh_file)
    if not mesh_path.exists():
        print(f"Error: Mesh file {args.mesh_file} does not exist")
        return 1
    
    # Get mesh type
    mesh_type = mesh_path.suffix.lower()[1:]  # Remove the dot
    if mesh_type not in ['stl', 'dae']:
        print(f"Error: Unsupported mesh type {mesh_type}. Only STL and DAE are supported.")
        return 1
    
    # Determine model name
    model_name = args.name if args.name else mesh_path.stem
    
    # Create model directory
    script_dir = Path(__file__).parent
    models_dir = script_dir.parent / "models"
    model_dir = models_dir / model_name
    model_dir.mkdir(parents=True, exist_ok=True)
    
    # Create mesh URI
    mesh_uri = f"package://hunter_se_gazebo/meshes/{mesh_type}/{mesh_path.name}"
    
    # Create SDF file
    sdf_content = create_sdf_content(model_name, mesh_uri, args.static, mesh_type)
    sdf_file = model_dir / "model.sdf"
    with open(sdf_file, 'w') as f:
        f.write(sdf_content)
    
    # Create config file
    config_content = create_config_content(model_name, args.description)
    config_file = model_dir / "model.config"
    with open(config_file, 'w') as f:
        f.write(config_content)
    
    print(f"Created model '{model_name}' in {model_dir}")
    print(f"SDF file: {sdf_file}")
    print(f"Config file: {config_file}")
    print(f"Mesh URI: {mesh_uri}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())