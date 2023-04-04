from setuptools import setup

package_name = 'ros2_voice_control'

setup(
    name=package_name,
    version='0.4.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['vc_cmd.dic']),
        ('share/' + package_name, ['vc_cmd.kwlist']),
        ('share/' + package_name, ['corpus.txt']),
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
        	'decoder = ros2_voice_control.voice_decoder:main',
        	'translator = ros2_voice_control.voice_translator:main'
        ],
    },
)
