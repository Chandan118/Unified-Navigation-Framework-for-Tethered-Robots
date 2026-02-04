#!/usr/bin/env python3
"""
Procedural World Generator for ATLAS-T Simulation
Generates complex warehouse environments with dynamic obstacles using Perlin noise
"""

import numpy as np
from noise import pnoise2
import random

class WorldGenerator:
    def __init__(self, width=50, height=50, scale=10.0):
        self.width = width
        self.height = height
        self.scale = scale
        self.octaves = 6
        self.persistence = 0.5
        self.lacunarity = 2.0
        self.seed = random.randint(0, 1000)
        
    def generate_perlin_map(self):
        """Generate 2D Perlin noise map for obstacle placement"""
        noise_map = np.zeros((self.height, self.width))
        for i in range(self.height):
            for j in range(self.width):
                noise_map[i][j] = pnoise2(
                    i/self.scale,
                    j/self.scale,
                    octaves=self.octaves,
                    persistence=self.persistence,
                    lacunarity=self.lacunarity,
                    repeatx=self.width,
                    repeaty=self.height,
                    base=self.seed
                )
        return noise_map
    
    def create_world_file(self, filename="warehouse_world.world"):
        """Generate Gazebo world file with procedural obstacles"""
        noise_map = self.generate_perlin_map()
        
        world_header = """<?xml version="1.0"?>
<sdf version="1.6">
  <world name="warehouse_world">
    
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Base Station (Tether anchor point) -->
    <model name="base_station">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="base_station_link">
        <visual>
          <geometry>
            <box size="0.5 0.5 1.0</box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <collision>
          <geometry>
            <box size="0.5 0.5 1.0</box>
          </geometry>
        </collision>
      </link>
    </model>

"""
        
        # Add procedural obstacles
        obstacle_models = ""
        obstacle_count = 0
        
        for i in range(0, self.height, 3):
            for j in range(0, self.width, 3):
                noise_value = noise_map[i, j]
                
                # Place obstacle if noise value exceeds threshold
                if noise_value > 0.3:  # Threshold for obstacle placement
                    x = (j - self.width/2) * 2  # Scale to meters
                    y = (i - self.height/2) * 2
                    
                    # Vary obstacle size based on noise
                    size = 0.5 + abs(noise_value) * 2
                    height = 0.5 + abs(noise_value) * 1.5
                    
                    obstacle_models += f"""
    <!-- Obstacle {obstacle_count} -->
    <model name="obstacle_{obstacle_count}">
      <static>true</static>
      <pose>{x} {y} {height/2} 0 0 0</pose>
      <link name="link">
        <visual>
          <geometry>
            <box size="{size} {size} {height}</box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <collision>
          <geometry>
            <box size="{size} {size} {height}</box>
          </geometry>
        </collision>
      </link>
    </model>
"""
                    obstacle_count += 1
        
        # Add dynamic actors (moving obstacles)
        actor_models = ""
        for actor_id in range(5):  # 5 dynamic obstacles
            start_x = random.uniform(-20, 20)
            start_y = random.uniform(-20, 20)
            
            actor_models += f"""
    <!-- Dynamic Actor {actor_id} -->
    <actor name="actor_{actor_id}">
      <skin>
        <filename>model://person_standing/meshes/standing.dae</filename>
      </skin>
      <pose>{start_x} {start_y} 1.0 0 0 0</pose>
      <animation name="walking">
        <filename>model://person_walking/meshes/walking.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <plugin name="actor_{actor_id}_plugin" filename="libgazebo_ros_actor.so">
        <target_x>{random.uniform(-20, 20)}</target_x>
        <target_y>{random.uniform(-20, 20)}</target_y>
        <velocity>0.5</velocity>
        <obstacle_margin>1.0</obstacle_margin>
        <animation_factor>5.0</animation_factor>
      </plugin>
    </actor>
"""
        
        world_footer = """
  </world>
</sdf>
"""
        
        # Combine all parts
        world_content = world_header + obstacle_models + actor_models + world_footer
        
        # Write to file
        with open(filename, 'w') as f:
            f.write(world_content)
        
        print(f"Generated world file: {filename}")
        print(f"Total static obstacles: {obstacle_count}")
        print(f"Total dynamic actors: 5")
        
if __name__ == "__main__":
    import sys
    import os
    
    # Get output directory
    if len(sys.argv) > 1:
        output_file = sys.argv[1]
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_file = os.path.join(script_dir, "../worlds/warehouse_world.world")
    
    # Generate world
    generator = WorldGenerator(width=50, height=50, scale=10.0)
    generator.create_world_file(output_file)
