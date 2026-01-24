#!/usr/bin/env python3
"""
Quick test script to verify all nodes can be imported
"""

import sys

def test_imports():
    """Test that all required Python modules are available"""
    print("Testing Python dependencies...")
    
    modules = {
        'numpy': 'NumPy',
        'scipy': 'SciPy',
        'skfuzzy': 'scikit-fuzzy',
        'cv2': 'OpenCV',
        'noise': 'Perlin Noise',
    }
    
    failed = []
    for module, name in modules.items():
        try:
            __import__(module)
            print(f"  ✓ {name}")
        except ImportError:
            print(f"  ✗ {name} - NOT FOUND")
            failed.append(name)
    
    if failed:
        print("\nMissing dependencies:")
        for name in failed:
            print(f"  - {name}")
        print("\nInstall with: pip3 install -r requirements.txt")
        return False
    
    print("\n✓ All Python dependencies found!")
    return True

def test_ros():
    """Test ROS installation"""
    print("\nTesting ROS environment...")
    
    import os
    ros_distro = os.environ.get('ROS_DISTRO')
    
    if not ros_distro:
        print("  ✗ ROS not sourced")
        print("  Run: source /opt/ros/noetic/setup.bash")
        return False
    
    print(f"  ✓ ROS {ros_distro} found")
    
    try:
        import rospy
        print("  ✓ rospy importable")
    except ImportError:
        print("  ✗ rospy not found")
        return False
    
    return True

def test_nodes():
    """Test that all node files exist and are executable"""
    import os
    print("\nTesting node files...")
    
    base_path = os.path.dirname(__file__)
    nodes_path = os.path.join(base_path, 'src/hybrid_navigation/nodes')
    
    required_nodes = [
        'sensor_fusion_node.py',
        'scene_recognition_node.py',
        'hybrid_planner_node.py',
        'fuzzy_controller_node.py',
        'tether_tension_node.py',
        'ga_optimizer_node.py',
    ]
    
    if not os.path.exists(nodes_path):
        print(f"  ✗ Nodes directory not found: {nodes_path}")
        return False
    
    missing = []
    for node in required_nodes:
        node_path = os.path.join(nodes_path, node)
        if os.path.exists(node_path):
            if os.access(node_path, os.X_OK):
                print(f"  ✓ {node}")
            else:
                print(f"  ! {node} exists but not executable")
                print(f"    Run: chmod +x {node_path}")
        else:
            print(f"  ✗ {node} - NOT FOUND")
            missing.append(node)
    
    if missing:
        return False
    
    print("\n✓ All nodes found!")
    return True

if __name__ == '__main__':
    print("="*50)
    print("ATLAS-T Simulation Environment Test")
    print("="*50)
    
    results = []
    
    results.append(("Python Dependencies", test_imports()))
    results.append(("ROS Environment", test_ros()))
    results.append(("Node Files", test_nodes()))
    
    print("\n" + "="*50)
    print("Test Summary")
    print("="*50)
    
    all_passed = True
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        symbol = "✓" if passed else "✗"
        print(f"{symbol} {name}: {status}")
        all_passed = all_passed and passed
    
    print("="*50)
    
    if all_passed:
        print("\n✓ All tests passed! Ready to build.")
        print("\nNext steps:")
        print("  1. Run: ./setup.sh")
        print("  2. Run: source devel/setup.bash")
        print("  3. Launch: roslaunch hybrid_navigation master.launch")
        sys.exit(0)
    else:
        print("\n✗ Some tests failed. Fix issues above before building.")
        sys.exit(1)
