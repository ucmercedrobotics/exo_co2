# EXO-CO2 driver
A ROS2 wrapper for the AtlasScientific EXO-CO2 sensor driver (via atlas_i2c library)

## Install
(from your venv)
```
pip install -r requirements.txt
colcon build
source install/setup.zsh
```

## Run
```
ros2 run exo_co2 exo_co2_node

ros2 topic echo /exoco2/ppm
```

## Dependencies
This depends on the following python packages:
- [`atlas_i2c`](https://github.com/adafruit/CircuitPython_NAU7802/) 
Package for communicating with Atlas Scientific devices over I2C 
- [`empy`](https://pypi.org/project/empy/) version 3.3.4 
(required to workaround [this issue](https://github.com/colcon/colcon-core/issues/602) 
until `colcon` supports empy v4)