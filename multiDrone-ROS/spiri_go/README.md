# SpiriGo

SpiriGo is a ROS package for performing autonomous flights with a MAVLink UAV. It is built upon [`mavros`](http://wiki.ros.org/mavros) and [ArduPilot](https://github.com/diydrones/ardupilot).

SpiriGo is currently in very early stages of development. We have some features planned for later:

- Trajectory planning for goals
- SLAM
- Depth mapping

But for now it provides some basic convenience methods for `mavros` use.

## Installation and Usage

### On the Jetson TK1

SpiriGo is primarily being developed for use on the [Jetson TK1](http://www.nvidia.ca/object/jetson-tk1-embedded-dev-kit.html) for now. We have thus created an [automated install script](https://github.com/Pleiades-Spiri/spiri_go/blob/master/install-spirigo.sh) that will install necessary components on the Jetson for autonomous UAV development. This includes:

- [ROS Indigo](http://wiki.ros.org/indigo), 
- [CUDA 6.5](https://developer.nvidia.com/cuda-toolkit-65), and
- [OpenCV4Tegra](http://elinux.org/Jetson/Computer_Vision_Performance#Hardware_Acceleration_of_OpenCV).

Although it is [generally recognized that ROS is incompatible with OpenCV4Tegra](http://wiki.ros.org/NvidiaJetsonTK1), our install script correctly works around the issue via [methods outlined in this post](https://devtalk.nvidia.com/default/topic/835118/embedded-systems/incorrect-configuration-in-opencv4tegra-debian-packages-and-solution). To install:

1. Flash your Jetson with the latest L4T using [Jetpack](https://developer.nvidia.com/embedded/jetson-development-pack-archive);
2. Install the [Grinch kernel](https://devtalk.nvidia.com/default/topic/766303/embedded-systems/-customkernel-the-grinch-19-3-8-for-jetson-tk1-developed/) for your version of L4T; then
3. Get the installation script and run it *without* root:

```bash
$ wget https://raw.githubusercontent.com/Pleiades-Spiri/spiri_go/master/install-spirigo.sh
$ chmod +x install-spirigo.sh
$ ./install-spirigo.sh
```

Note: you will be prompted for your password right away even though it's without `sudo`. 

#### Connect the Jetson to the Pixhawk via serial/UART connection

1. Connect the Jetson's UART ports to an added voltage converter (1.8 V to 5V);
2. Connect the voltage converter to the Pixhawk;
3. Make sure that the serial connection is established by checking `/dev/ttyTHS0`;
4. Validate the Jetson to Pixhawk communication using maximum baud rate (921600).

| TK1           | TXB0104     | Pixhawk |
| ------------- | ----------- | ------- |
| 1.8V: P37     | VCCA - VCCB | 5V: P1  |
| GND: P38      | GND - GND   | GND: P6 |
| TXd1: P41     | A1 - B1     | TX1: P2 |
| RXd1: P44     | A2 - B2     | RX1: P3 |

See [this diagram](https://drive.google.com/open?id=0BxXn6LyBxnG6b01mc1N5X2diVlU) for pinout details.

#### Running the launch file

Use this command to launch SpiriGo with serial communication: 

```
$ roslaunch spiri_go jetson.launch
```

If you see 

```
$ [ INFO] [1447370932.205419674]: CON: Got HEARTBEAT, connected.
```

It means you've succeeded. Note the baud rate is set to `921600` because that's the highest allowed by `mavros`. We're looking into increasing this to `1500000` maybe if we really need to.

### On Ubuntu 14.04 desktop (for simulator)

To run the simulator, clone a copy of the ardupilot project from ArduPilot. Follow the directions at [ArduPilot SITL](http://dev.ardupilot.com/wiki/sitl-simulator-software-in-the-loop) to set up the simulator. There is also a script, `initalization/apm_sim` that will do this set-up automatically. 

This script is meant to be ran from a fresh Ubuntu 14.04 installation. Please proceed with caution if you plan on installing on a system with ROS already installed.

```bash
$ wget https://raw.githubusercontent.com/Pleiades-Spiri/spiri_go/master/initialize/apm_sim
$ chmod +x apm_sim
$ ./apm_sim
```

#### Running the simulator and the launch file

Start the simulator with:

```bash
$ sim_vehicle.sh --map --console --aircraft test
```

Next launch the sitl node for spiri:

```bash
$ roslaunch spiri_go sitl.launch
```

Note: if you're running this the first time, you may need to disable RC calibration pre-arm check via the `mavproxy` console.

If you are using a ground control station such as APM Planner 2, it should connect on UDP port `14551`.

### Python API

We also provide a python API to control Spiri, using the services and actions provided by the ROS library.

To install this on your system, run:

```bash
$ python setup.py install
```

from the root of this repository.

To use this in a python script, include the following line:

```python
from spiripy import api
spiri = api.SpiriGo()
```

`spiri` will then be a SpiriGo instance with the following methods:

```python
wait_for_ros([timeout])
wait_for_fcu([timeout])
get_state()
get_local_position()
takeoff([height])
land()
```

Your script can be launched as a normal python scripts, but `spiri_go`, and `mavros` packages must be running in the background for it to control a quadcopter.

Detailed API documentation coming soon.

## Generating documentation

TODO

## Developers

### Running tests

To test the python API, run:

```bash
$ python test/api.py
```

To test those parts that require the simulator running, use:

```bash
$ python test/sim.py
```

Any new methods that are made MUST have a corresponding unit test, and if possible should have a corresponding unit test with the simulator. It is good practice to write the test before implementing the method.
