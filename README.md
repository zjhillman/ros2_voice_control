# ros2_voice_controller
ROS2 Voice Controller is a python script that publishes commands under ROS2 and uses pocketsphinx 5.0.0 to give commands in English.

## Install Dependencies

### Linux Dependencies
Use the following commands to ensure you have the following python packages installed
```
sudo apt install python3 cython3
sudo apt-get install python3-pip python3-pyaudio
pip3 install pocketsphinx
```
If  you are using linux you must install PortAudio for LiveSpeech functions
```
sudo apt-get install libportaudio2
```

### Install ROS2
Currently, ROS Humble is supported but any version of ROS2 should work.
This script will not work if ROS2 is not installed

## Running the Script

### Defaults

By default, the script uses the US English language model that comes preinstalled with pocketsphinx but can be changed to any language model with the model argument:
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
