from setuptools import find_packages, setup

package_name = 'acc_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS 2 Developer',
    maintainer_email='developer@example.com',
    description='ACC/AEB simulation without camera',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'acc_sim_node = acc_sim.acc_sim_node:main',
        ],
    },
)
