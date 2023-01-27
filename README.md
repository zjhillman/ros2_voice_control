# ros2_voice_control
ROS2 Voice Control is a ros2 package written in python. This package let's you give voice commands to a turtlebot or any topic you choose

## Ubuntu Installation

### Install dependencies
Use the following commands to ensure you have python3 and the tools to build pocketshinx. The voice controller uses pocketsphinx and pyaudio to get live input
```
sudo apt install python3 cython3
sudo apt-get install python3-pip python3-pyaudio libportaudio2
pip3 install pocketsphinx
```

### Install ROS2
Currently, ROS Humble is supported but any version of ROS2 should work.
This script will not work if ROS2 is not installed

## Running the Script

The voice controller uses two nodes to run:  
&emsp;voice_detector - run pocketsphinx, get voice commands as strings and publish to topic  
&emsp;voice_reporter - listens to voice_detector's topic and publish to '/cmd_vel'  
  
In one terminal, run 
```
ros2 run ros2_voice_control voice_detector
```
And in another, enter
```
ros2 run ros2_voice_control voice_reporter
```

### Defaults

By default, voice_detector uses the US English language model that comes preinstalled with pocketsphinx but can be changed to any language model with the model argument:
```
--model [PATH_TO_MODEL]
```

Two files come pre-packeged with the script, the dictionary and the keyword list. This script will not run without a .dic and .kwlist file.
To add a custom dictionary file, use the argument:
```
--lexicon [PATH_TO_DICTIONARY]
```
and to add a custom keyword list, use:
```
--kwlist [PATH_TO_KEYWORD_LIST]
```
