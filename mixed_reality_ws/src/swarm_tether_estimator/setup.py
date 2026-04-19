from setuptools import setup

package_name = 'swarm_tether_estimator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chandan',
    maintainer_email='chandan@formica.bot',
    description='Swarm tether estimator for mixed-reality experiments',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tether_estimator_node = swarm_tether_estimator.tether_publisher:main',
        ],
    },
)
