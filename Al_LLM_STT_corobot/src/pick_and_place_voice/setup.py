from setuptools import find_packages, setup
import glob
import os
package_name = 'pick_and_place_voice'
setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=find_packages(include=[
        'robot_control',
        'voice_processing',
        'object_detection'
        'project',
    ]),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob('resource/*')),
        ('share/' + package_name + '/resource', glob.glob('resource/.env')),
        # ('share/ament_index/resource_index/packages',['resource/' + 'voice_processing']),
        # ('share/voice_processing', ['package.xml']),
        # ('share/object_detection', ['package.xml']),
        # ('share/robot_control', ['package.xml']),
        # ('share/' + package_name + '/launch', glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey4090',
    maintainer_email='rokey4090@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'robot_control = robot_control.robot_control:main',
            'object_detection = object_detection.detection:main',
            'get_keyword= voice_processing.get_keyword_custom:main',
            'keyword_extraction_3 = voice_processing.getword_extraction_3:main',
            'gas_station_yolo_stt_mapping = voice_processing.gas_station_yolo_stt_mapping:main',
            'onrobot_robot_control = voice_processing.onrobot_robot_control:main',
            'gui = voice_processing.gui:main',
            'yolo_gas_station_custom = voice_processing.yolo_gas_station_custom:main',

        ],
    },
)