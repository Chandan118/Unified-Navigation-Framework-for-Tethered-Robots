from setuptools import setup

package_name = 'tether_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chandan',
    maintainer_email='chandan@formica.bot',
    description='LiDAR-based tether detection for dynamic avoidance',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tether_detector_node = tether_detection.tether_detector_node:main',
        ],
    },
)
