# Virginia Motorsports Dashboard

Welcome to Virginia Motorsports Dashboard, a project that aims to provide real-time data visualization for motorsports enthusiasts. This project is built on ROS2 Humble and several Python packages. It also requires SSH access to a Raspberry Pi.

## Installation Instructions

To get started with Virginia Motorsports Dashboard, you need to install ROS2 Humble, Python packages, and set up SSH access to your Raspberry Pi.

### ROS2 Humble Installation

To install ROS2 Humble, follow the instructions given in the [official ROS2 documentation](https://docs.ros.org/en/galactic/Installation.html). Make sure you install all the required dependencies before installing ROS2.

### Python Packages Installation

The following Python packages are required to run Virginia Motorsports Dashboard:

- `numpy`
- `pandas`
- `matplotlib`
- `seaborn`
- `bokeh`

To install these packages, run the following command:

```
pip install numpy pandas matplotlib seaborn bokeh
```

### SSH Access to Raspberry Pi

To set up SSH access to your Raspberry Pi, follow these steps:

1. Connect your Raspberry Pi to your local network.
2. Find the IP address of your Raspberry Pi by running the `ifconfig` command on your Raspberry Pi.
3. On your local machine, open a terminal window and run the following command:

```
ssh pi@<RASPBERRY_PI_IP_ADDRESS>
```

Replace `<RASPBERRY_PI_IP_ADDRESS>` with the actual IP address of your Raspberry Pi. You will be prompted for the password for the `pi` user. Enter the default password `raspberry` when prompted.

## Usage

To run Virginia Motorsports Dashboard, follow these steps:

1. Clone this repository to your local machine:

```
git clone https://github.com/yourusername/virginia-motorsports-dashboard.git
```

2. Change to the `virginia-motorsports-dashboard` directory:

```
cd virginia-motorsports-dashboard
```

3. Build the project using `colcon build`:

```
colcon build
```

4. Source the project:

```
. install/setup.bash
```

5. Launch the dashboard:

```
ros2 launch virginia_motorsports_dashboard dashboard.launch.py
```

## Contributing

If you'd like to contribute to Virginia Motorsports Dashboard, please fork this repository and submit a pull request.


## Broken bag files
```
sqlite3 rosbag2_recording.db3 .recover > data.sql
mv rosbag2_recording.db3 rosbag2_recording.db3.bak
sqliite3 rosbag2_recording.db3 < data.sql
```