from setuptools import setup
from glob import glob
import os

package_name = 'igo_nav2_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), 
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        ('share/' + package_name + '/description', ['description/igo_description.urdf']),
        ('share/' + package_name + '/worlds', ['worlds/my_world-01.world', 'worlds/my_world-02.world']),
        ('share/' + package_name + "/config", ['config/igo_laser_filter.yaml', 'config/igo_lds_2d.lua']),
        ('share/' + package_name + "/params", ['params/igo_navigation.yaml']),
        ('share/' + package_name + "/maps", ['maps/sim_warehouse.yaml', 'maps/sim_warehouse.pgm']),
        ('share/' + package_name + "/maps", ['maps/real_warehouse.yaml', 'maps/real_warehouse.pgm']),
   ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ralf Koenig',
    maintainer_email='ralf.koenig@kiongroup.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'igo_waypoint_pilot = igo_nav2_sim.igo_waypoint_pilot:main'
         ],
    },
)
