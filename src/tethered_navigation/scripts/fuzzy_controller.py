#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

# This is the core library for fuzzy logic in Python.
# You must install it: pip install -U scikit-fuzzy
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Import ROS 2 message types
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class FuzzyControllerNode(Node):
    """
    This node implements a fuzzy logic controller to provide fine-tuned
    adjustments to the robot's velocity commands based on sensor inputs.
    """
    def __init__(self):
        super().__init__('fuzzy_controller')
        
        # --- 1. Define Fuzzy Inputs (Antecedents) ---
        # These are the inputs to our fuzzy system. We define their range of possible values.
        # Obstacle distance from 0 to 5 meters.
        self.obstacle_distance = ctrl.Antecedent(np.arange(0, 5.1, 0.1), 'obstacle_distance')
        # Tether tension from 0 to 50 Newtons (as per paper).
        self.tether_tension = ctrl.Antecedent(np.arange(0, 51, 1), 'tether_tension')

        # --- 2. Define Fuzzy Outputs (Consequents) ---
        # These are the outputs of our fuzzy system. The values are normalized from -1 to 1.
        # A negative value means decrease, positive means increase.
        self.velocity_adj = ctrl.Consequent(np.arange(-1.0, 1.1, 0.1), 'velocity_adj')
        self.steering_adj = ctrl.Consequent(np.arange(-1.0, 1.1, 0.1), 'steering_adj')

        # --- 3. Define Membership Functions ---
        # This is where we define the linguistic terms like "near", "medium", "far".
        # We use triangular membership functions (trimf) for simplicity.
        
        # Membership functions for obstacle_distance
        self.obstacle_distance['near'] = fuzz.trimf(self.obstacle_distance.universe, [0, 0, 1.0])
        self.obstacle_distance['medium'] = fuzz.trimf(self.obstacle_distance.universe, [0.5, 1.5, 2.5])
        self.obstacle_distance['far'] = fuzz.trimf(self.obstacle_distance.universe, [2.0, 5.0, 5.0])

        # Membership functions for tether_tension
        self.tether_tension['low'] = fuzz.trimf(self.tether_tension.universe, [0, 0, 20])
        self.tether_tension['high'] = fuzz.trimf(self.tether_tension.universe, [15, 50, 50])

        # Membership functions for the output adjustments
        self.velocity_adj['reduce'] = fuzz.trimf(self.velocity_adj.universe, [-1.0, -1.0, 0])
        self.velocity_adj['maintain'] = fuzz.trimf(self.velocity_adj.universe, [-0.2, 0, 0.2])
        self.velocity_adj['increase'] = fuzz.trimf(self.velocity_adj.universe, [0, 1.0, 1.0])

        self.steering_adj['left'] = fuzz.trimf(self.steering_adj.universe, [-1.0, -1.0, 0])
        self.steering_adj['none'] = fuzz.trimf(self.steering_adj.universe, [-0.2, 0, 0.2])
        self.steering_adj['right'] = fuzz.trimf(self.steering_adj.universe, [0, 1.0, 1.0])

        # --- 4. Define Fuzzy Rules ---
        # This is the "knowledge base" of the system, written in an intuitive way.
        rule1 = ctrl.Rule(self.obstacle_distance['near'] & self.tether_tension['high'], 
                          (self.velocity_adj['reduce'], self.steering_adj['left']))
        
        rule2 = ctrl.Rule(self.obstacle_distance['near'] & self.tether_tension['low'], 
                          (self.velocity_adj['reduce'], self.steering_adj['right']))
        
        rule3 = ctrl.Rule(self.obstacle_distance['medium'], 
                          (self.velocity_adj['maintain'], self.steering_adj['none']))
        
        rule4 = ctrl.Rule(self.obstacle_distance['far'], 
                          (self.velocity_adj['increase'], self.steering_adj['none']))
        
        # --- 5. Create the Control System ---
        self.control_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4])
        self.controller = ctrl.ControlSystemSimulation(self.control_system)

        # --- ROS 2 Communication Setup ---
        # Subscriber to get the raw input values from the main navigator node
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'fuzzy_input',
            self.listener_callback,
            10)
            
        # Publisher to send the calculated adjustments back to the main navigator
        self.publisher_ = self.create_publisher(Twist, 'fuzzy_adjustment', 10)
        
        self.get_logger().info('Fuzzy Controller Node is running and ready.')

    def listener_callback(self, msg: Float32MultiArray):
        """
        This function is called every time the navigator publishes new sensor data.
        """
        # --- 6. Fuzzification and Inference ---
        # Pass the crisp input values to the controller
        self.controller.input['obstacle_distance'] = msg.data[0]
        self.controller.input['tether_tension'] = msg.data[1]
        
        # Compute the result
        try:
            self.controller.compute()
            
            # --- 7. Defuzzification ---
            # Create a Twist message to hold the crisp output values
            adjustment = Twist()
            adjustment.linear.x = self.controller.output['velocity_adj']
            adjustment.angular.z = self.controller.output['steering_adj']
            
            # Publish the adjustment
            self.publisher_.publish(adjustment)
        except Exception as e:
            # This can happen if an input is outside the defined universe (e.g., distance < 0)
            self.get_logger().warn(f'Could not compute fuzzy logic: {e}')

def main(args=None):
    rclpy.init(args=args)
    fuzzy_controller = FuzzyControllerNode()
    try:
        rclpy.spin(fuzzy_controller)
    except KeyboardInterrupt:
        pass
    finally:
        fuzzy_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()