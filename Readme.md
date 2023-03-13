# LDS-006-Lidar-Sensor-Library

Folks, from software side of things, [this](https://www.jentsch.io/lds-006-lidar-sensor-reverse-engineering/) is a stub. So I brought i some upgrades. To conduct tests I am using Raspberry Pi Model 3/4. Feel free to use the library with any other hardware.

## How to connect hardware

```
black: GND
red: VCC (5V)
blue: UART TX (3v3, Pin 10 @ RPI)
green: UART RX (3v3, Pin 8 @ RPI)
```

## How to build library

In this directory: setup a Virtual Environment, then install library
```
sudo apt-get install python3-venv python3-numpy
python3 -m venv ./venv
. venv/bin/activate
pip install -r requirements.txt
python setup.py bdist_wheel
pip install dist/lds006*.whl
```

Numpy is used as a system requirement because RPI lacks libraries when installing via pip.

## How to use  library

In this directory: start Virtual Environment, then run 'main.py' **Caution: requires access to hardware**

```
. venv/bin/activate
python examples/simple.py /dev/serial0
```