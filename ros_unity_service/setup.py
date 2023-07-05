from setuptools import setup

package_name = 'ros_unity_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='msunkararos2',
    maintainer_email='mohith.sunkara@digitaltwin.technology',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "default_server_endpoint = ros_unity_service.default_server_endpoint:main"
        ],
    },
)
