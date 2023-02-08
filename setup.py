from setuptools import setup

package_name = 'ros2_voice_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['vc_cmd.dic']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zachary Hillman',
    maintainer_email='zachhillman@gmail.com',
    description='Ros2 Voice Controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'audio_stream = ros2_voice_control.audio_stream:main',
            'decoder = ros2_voice_control.decoder:main',
        	'relay = ros2_voice_control.command_relay:main'
        ],
    },
)
