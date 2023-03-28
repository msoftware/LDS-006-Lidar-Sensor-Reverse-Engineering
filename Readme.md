# LDS-006-Lidar-Sensor-Library

Greetings programs, [this](https://www.jentsch.io/lds-006-lidar-sensor-reverse-engineering/) is a high quality article about the sensor I recently found in an old vacuum cleaner. Sadly, from software side of things it is a stub.

So I brought some upgrades. To conduct tests I am using Raspberry Pi Model 3/4. Feel free to use the library with any other hardware.

## How to connect hardware

```
black: GND
red: VCC (5V)
blue: UART TX (3v3, Pin 10 @ RPI)
green: UART RX (3v3, Pin 8 @ RPI)
```

## How to build library

In this directory: setup a virtual environment, then install library
```
sudo apt-get install python3-venv
python3 -m venv ./venv
source venv/bin/activate
pip install -r requirements.txt
python setup.py bdist_wheel
pip install dist/lds006*.whl
```

## How to use  library

In this directory: start virtual environment, then run 'main.py'.

**Caution: requires access to hardware** Best practice is to add user to group *dialout*.

I would like to introduce a clean interface, thus use [Protocol Buffers](https://protobuf.dev/getting-started/pythontutorial/). Any subscriber message is returned via message encoding specified in `msgLds.proto`.

Example can be run by:

```
. venv/bin/activate
python examples/simple.py /dev/serial0
python examples/callback.py /dev/serial0
python examples/website/callback.py /dev/serial0
```

## Thougts on used software

In the latest patch I completely removed filtering (so NumPy is not used). Filtering data seems to be not required. I reverted to distinguish good from bad data by using the `reflectivity`-factor. Still, there appear runaway data points, which are marked invalid if they exceed standard deviation.

Website example includes [protobuf.js](https://github.com/protobufjs/protobuf.js/) v7.2.2. For my environment is not always connected to the internet, I included source code in this repository.