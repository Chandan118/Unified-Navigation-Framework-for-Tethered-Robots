from setuptools import setup

package_name = 'unitree_aliengo_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chandan',
    maintainer_email='chandan@formica.bot',
    description='ROS2 bridge for Unitree AlienGo yielding behavior',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aliengo_bridge_node = unitree_aliengo_bridge.aliengo_bridge_node:main',
        ],
    },
)
