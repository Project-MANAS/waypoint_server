# Waypoint Server

 :round_pushpin: Waypoint server ROS package with Google Maps integration.

#### Setup
Clone the repository
```bash
git clone
```

Install Geographic Library

```bash
pip install geopy
```

#### Usage
- Use catkin to build the ROS package.
- Inside the launch file set the following parameters:  
  - Threshold distance (meters)
  - GPS topic: `gps_topic`
  - Odometry topic:`odom_topic`
  - Generate Google Maps waypoints: Enable this to generate waypoint using Google Maps
- Run the rosnode  

```bash
roslaunch waypoint_server waypoint_server.launch
```

#### Google Maps integration
Create a Google Maps Developer account and generate an **API Key** with **Maps** and **Places** options selected.  
Inside the repository type the following:  

```bash
echo "YOUR_API_KEY" > waypoint_viewer/api_key.txt
```

Install Google Maps API for python

```bash
pip install googlemaps
```
