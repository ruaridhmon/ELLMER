# netft_rdt_driver

Catkinized version of [netft](https://github.com/mcubelab/netft). Provides ROS interface for using the [ATI Network Force/Torque Sensor System](https://www.ati-ia.com/products/ft/sensors.aspx). 

## Initial Setup

The various hardware setups are detailed in the sensor's [manual](https://www.ati-ia.com/app_content/documents/9620-05-NET%20FT.pdf). Typically in MCube we have the cable connecting the transducer to the netbox and the netbox connected to the computer via Ethernet. Importantly, this setup assumes PoE (power over ethernet) and in MCube we typically route the ethernet from the netbox through a network switch to the computer. Once plugged in, the lights should flash on. If after a few seconds the red lights switch to green, the sensor should be ready to go. The manual has a more detailed breakdown of the light patterns for debugging. 

Each of the netboxes in MCube are labeled with their IP address. They typically have the form `192.168.0.A` where A is some number under 255. In order to connect, set your IPv4 method to `Manual` and set the address to `192.168.1.B` (where B is any number under 255 such that A != B).  and the netmask to `24`. Once connected you should be able to ping the netbox IP address. 


## Launch 

This repo has a launch file. Note that you will need adjust the address to match the IP address on the netbox. 

```
roslaunch netft_rdt_driver netft.launch
```

## ROS Services and Topics

Once launched, there is one rostopic for reading the state of the sensor (`/netft/netft_data` which outputs a rostopic of type geometry_msgs/WrenchStamped), one rosservice for zeroing the readings of the sensor (`/netft/zero` which takes no arguments) and two rosservices related to logging (`/netft/netft/get_loggers` and 
`/netft/netft/set_logger_level`).

## Scripts

The `scripts/` folder has a few files for using the sensor, mainly to serve as an example/jumping-off point. 

- `run_ft.py` is a script with two main functions: one to zero the sensor (`zeroFTSensor()`) and one to record data for a fixed number of seconds (`record()`). Recoding creates a bag file in a folder called `tests`. 
- `bag2csv.sh` coverts the bag files of the force torque sensor recordings, for example those recorded by the previous script, and coverts them to csvs. For example, having used the previous script, we would run `./bag2csv.sh tests'`. 
- `parse_csv_log.py` reads in the csv files and parses them into a Nx6 array, where N is the number of timesteps. It then uses matplotlib to plot the wrenches over time, splitting forces and torques into two side-by-side plots. 
