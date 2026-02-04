#!/usr/bin/env python3
"""
Genetic Algorithm Optimizer for Parameter Tuning
Optimizes fuzzy controller parameters based on fitness function (Eq. 17 from paper)
"""

import rospy
import numpy as np
import random
import os
from dynamic_reconfigure.client import Client
from nav_msgs.msg import Path
from hybrid_navigation.msg import TetherStatus

class GAOptimizer:
    def __init__(self):
        rospy.init_node('ga_optimizer_node', anonymous=False)
        
        # GA Parameters
        self.population_size = rospy.get_param('~population_size', 20)
        self.mutation_rate = rospy.get_param('~mutation_rate', 0.1)
        self.crossover_rate = rospy.get_param('~crossover_rate', 0.7)
        self.num_generations = rospy.get_param('~num_generations', 10)
        
        # Parameter bounds (fuzzy controller parameters)
        self.param_bounds = {
            'linear_speed': (0.2, 1.0),
            'max_angular_speed': (0.5, 1.5),
            'safety_distance': (0.5, 2.0),
        }
        
        # Fitness tracking
        self.current_path_length = 0.0
        self.current_tether_stress = 0.0
        self.path_history = []
        
        # Population
        self.population = self.initialize_population()
        self.current_individual_index = 0
        self.generation = 0
        
        # Subscribers for fitness calculation
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)
        rospy.Subscriber('/tether_status', TetherStatus, self.tether_callback)
        
        # Timer for GA evolution
        self.optimization_interval = rospy.get_param('~optimization_interval', 60)  # seconds
        rospy.Timer(rospy.Duration(self.optimization_interval), self.evolve_population)
        
        rospy.loginfo("GA Optimizer initialized with population size=%d", self.population_size)
    
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
                length += np.sqrt(dx*dx + dy*dy)
            self.current_path_length = length
    
    def tether_callback(self, msg):
        """Track tether stress"""
        # Stress is combination of tension and length
        normalized_length = msg.length / msg.max_length
        normalized_tension = msg.tension / 50.0  # Max 50N
        self.current_tether_stress = (normalized_length + normalized_tension) / 2.0
    
    def calculate_fitness(self):
        """
        Calculate fitness based on Eq. 17 from paper:
        Fitness = 1 / (w1 * Path_Length + w2 * Tether_Stress)
        Lower is better, so we invert
        """
        w1 = 0.6  # Weight for path length
        w2 = 0.4  # Weight for tether stress
        
        # Avoid division by zero
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
        crossover_point = random.randint(1, len(params) - 1)
        
        for i, param in enumerate(params):
            if i < crossover_point:
                child1[param] = parent1[param]
                child2[param] = parent2[param]
            else:
                child1[param] = parent2[param]
                child2[param] = parent1[param]
        
        return child1, child2
    
    def mutate(self, individual):
        """Gaussian mutation"""
        mutated = individual.copy()
        
        for param, value in mutated.items():
            if random.random() < self.mutation_rate:
                min_val, max_val = self.param_bounds[param]
                # Gaussian mutation with 10% std
                mutation = np.random.normal(0, (max_val - min_val) * 0.1)
                mutated[param] = np.clip(value + mutation, min_val, max_val)
        
        return mutated
    
    def apply_parameters(self, individual):
        """Apply parameter set to ROS parameter server"""
        for param, value in individual.items():
            param_name = '/fuzzy_controller_node/' + param
            rospy.set_param(param_name, float(value))
            rospy.loginfo("Set %s = %.3f", param_name, value)
    
    def evolve_population(self, event):
        """Main GA evolution loop"""
        rospy.loginfo("Generation %d: Evaluating fitness...", self.generation)
        
        # Evaluate fitness for all individuals
        fitnesses = []
        for individual in self.population:
            self.apply_parameters(individual)
            rospy.sleep(2.0)  # Let system stabilize
            fitness = self.calculate_fitness()
            fitnesses.append(fitness)
        
        # Log best individual
        best_idx = np.argmax(fitnesses)
        rospy.loginfo("Best fitness: %.6f with params: %s", 
                     fitnesses[best_idx], self.population[best_idx])
        
        # Create new population
        new_population = []
        
        # Elitism: keep best individual
        new_population.append(self.population[best_idx].copy())
        
        # Generate rest of population
        while len(new_population) < self.population_size:
            # Selection
            parent1, parent2 = self.select_parents(fitnesses)
            
            # Crossover
            child1, child2 = self.crossover(parent1, parent2)
            
            # Mutation
            child1 = self.mutate(child1)
            child2 = self.mutate(child2)
            
            new_population.append(child1)
            if len(new_population) < self.population_size:
                new_population.append(child2)
        
        self.population = new_population
        self.generation += 1
        
        # Apply best parameters
        self.apply_parameters(self.population[0])
        
        rospy.loginfo("Generation %d complete. Applied best parameters.", self.generation)
        
        # Save best parameters to results directory
        try:
            results_dir = os.path.join(os.environ['HOME'], 'atlas_ws/results')
            if not os.path.exists(results_dir):
                os.makedirs(results_dir)
            
            best_param_file = os.path.join(results_dir, 'best_parameters.json')
            import json
            with open(best_param_file, 'w') as f:
                json.dump({
                    'generation': self.generation,
                    'fitness': float(fitnesses[best_idx]),
                    'parameters': self.population[best_idx]
                }, f, indent=4)
            rospy.loginfo("Saved best parameters to %s", best_param_file)
        except Exception as e:
            rospy.logerr("Failed to save best parameters: %s", str(e))

if __name__ == '__main__':
    try:
        optimizer = GAOptimizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
