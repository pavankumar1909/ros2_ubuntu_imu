from setuptools import find_packages, setup

package_name = 'imu_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', ['launch/imu_launch.py']),
       
        # Install the parameter YAML file 
        ('share/' + package_name + '/config', ['config/imu_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavan',
    maintainer_email='pavankumarangajala09@gmai.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'imu_node = imu_bringup.imu_node:main',
        ],
    },
)
