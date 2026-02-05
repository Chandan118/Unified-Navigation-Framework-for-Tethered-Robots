#!/usr/bin/env python3
"""
Genetic Algorithm Optimizer for Parameter Tuning (ROS 2 version)
Optimizes fuzzy controller parameters based on fitness function (Eq. 17 from paper)
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Path as NavPath
from hybrid_navigation_msgs.msg import TetherStatus
import numpy as np
import random
import os
import json
import time
import math

class GAOptimizer(Node):
    def __init__(self):
        super().__init__('ga_optimizer_node')
        
        # GA Parameters
        self.declare_parameter('population_size', 20)
        self.declare_parameter('mutation_rate', 0.1)
        self.declare_parameter('crossover_rate', 0.7)
        self.declare_parameter('num_generations', 10)
        self.declare_parameter('optimization_interval', 60.0)
        
        self.population_size = self.get_parameter('population_size').get_parameter_value().integer_value
        self.mutation_rate = self.get_parameter('mutation_rate').get_parameter_value().double_value
        self.crossover_rate = self.get_parameter('crossover_rate').get_parameter_value().double_value
        self.num_generations = self.get_parameter('num_generations').get_parameter_value().integer_value
        self.optimization_interval = self.get_parameter('optimization_interval').get_parameter_value().double_value
        
        # Parameter bounds (fuzzy controller parameters)
        self.param_bounds = {
            'linear_speed': (0.2, 1.0),
            'max_angular_speed': (0.5, 1.5),
            'safety_distance': (0.5, 2.0),
        }
        
        # Fitness tracking
        self.current_path_length = 0.0
        self.current_tether_stress = 0.0
        
        # Population
        self.population = self.initialize_population()
        self.generation = 0
        
        # Subscribers
        self.create_subscription(NavPath, '/move_base/NavfnROS/plan', self.path_callback, 10)
        self.create_subscription(TetherStatus, '/tether_status', self.tether_callback, 10)
        
        # Timer for GA evolution
        self.timer = self.create_timer(self.optimization_interval, self.evolve_population)
        
        self.get_logger().info(f"GA Optimizer initialized with population size={self.population_size}")
    
    def initialize_population(self):
        """Initialize random population of parameter sets"""
        population = []
        for _ in range(self.population_size):
            individual = {}
            for param, (min_val, max_val) in self.param_bounds.items():
                individual[param] = random.uniform(min_val, max_val)
            population.append(individual)
        return population
    
    def path_callback(self, msg):
        """Track path length"""
        if len(msg.poses) > 1:
            length = 0.0
            for i in range(len(msg.poses) - 1):
                p1 = msg.poses[i].pose.position
                p2 = msg.poses[i+1].pose.position
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                length += math.sqrt(dx*dx + dy*dy)
            self.current_path_length = length
    
    def tether_callback(self, msg):
        """Track tether stress"""
        normalized_length = msg.length / msg.max_length
        normalized_tension = msg.tension / 50.0
        self.current_tether_stress = (normalized_length + normalized_tension) / 2.0
    
    def calculate_fitness(self):
        """Calculate fitness (Eq. 17 from paper)"""
        w1, w2 = 0.6, 0.4
        cost = w1 * self.current_path_length + w2 * self.current_tether_stress * 100
        fitness = 1.0 / (cost + 0.001)
        return fitness
    
    def select_parents(self, fitnesses):
        """Tournament selection"""
        tournament_size = 3
        parents = []
        for _ in range(2):
            tournament = random.sample(list(enumerate(fitnesses)), tournament_size)
            winner = max(tournament, key=lambda x: x[1])
            parents.append(self.population[winner[0]])
        return parents
    
    def crossover(self, parent1, parent2):
        """Single-point crossover"""
        if random.random() > self.crossover_rate:
            return parent1.copy(), parent2.copy()
        
        child1, child2 = {}, {}
        params = list(parent1.keys())
        cp = random.randint(1, len(params) - 1)
        for i, p in enumerate(params):
            if i < cp:
                child1[p], child2[p] = parent1[p], parent2[p]
            else:
                child1[p], child2[p] = parent2[p], parent1[p]
        return child1, child2
    
    def mutate(self, individual):
        """Gaussian mutation"""
        mutated = individual.copy()
        for p, v in mutated.items():
            if random.random() < self.mutation_rate:
                min_v, max_v = self.param_bounds[p]
                mutation = np.random.normal(0, (max_v - min_v) * 0.1)
                mutated[p] = np.clip(v + mutation, min_v, max_v)
        return mutated
    
    def apply_parameters(self, individual):
        """Apply parameter set (using rclpy node parameters)"""
        # Note: In a real multi-node setup, this should use a ParameterClient
        # or services to set parameters on the fuzzy_controller_node.
        # For simplicity, we just log it as applied.
        for p, v in individual.items():
            self.get_logger().info(f"Targeting /fuzzy_controller_node setting {p} = {v:.3f}")
            # Note: Set local parameters if this node was integrated or use service call
    
    def evolve_population(self):
        """Main GA evolution loop"""
        self.get_logger().info(f"Generation {self.generation}: Evaluating fitness...")
        
        fitnesses = []
        for ind in self.population:
            self.apply_parameters(ind)
            time.sleep(1.0) # stabilization
            fitnesses.append(self.calculate_fitness())
        
        best_idx = np.argmax(fitnesses)
        self.get_logger().info(f"Best fitness: {fitnesses[best_idx]:.6f}")
        
        new_pop = [self.population[best_idx].copy()] # Elitism
        while len(new_pop) < self.population_size:
            p1, p2 = self.select_parents(fitnesses)
            c1, c2 = self.crossover(p1, p2)
            new_pop.append(self.mutate(c1))
            if len(new_pop) < self.population_size:
                new_pop.append(self.mutate(c2))
        
        self.population = new_pop
        self.generation += 1
        self.save_results(fitnesses[best_idx])
    
    def save_results(self, best_fitness):
        """Save best results to file"""
        try:
            results_dir = os.path.expanduser('~/atlas_ws/results')
            os.makedirs(results_dir, exist_ok=True)
            with open(os.path.join(results_dir, 'best_parameters.json'), 'w') as f:
                json.dump({'gen': self.generation, 'best_fitness': best_fitness, 'best_params': self.population[0]}, f, indent=4)
        except Exception as e:
            self.get_logger().error(f"Failed to save results: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = GAOptimizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
