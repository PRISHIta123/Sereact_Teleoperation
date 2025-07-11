from setuptools import find_packages, setup

package_name = 'teleop_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prishita',
    maintainer_email='prishita@r85@gmail.com',
    description='Teleop node to republish joint angles',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'joint_state_republisher = teleop_node.joint_state_publisher:main',
    ],
    },
)
