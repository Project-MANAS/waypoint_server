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

The waypoint server also implements service calls.  
- Pose Waypoint  
    Service call on the topic `pose_waypoint` sets pose waypoint with respect to the map frame. This accepts waypoint as a `PoseStamped` message type.  
- Geo Waypoint  
Service call on the topic `geo_waypoint` sets geo waypoint with respect to the map frame. This accepts waypoint as a `NavSatFix` message type.

The service calls return `1` is the waypoint was successfully published. Refer the srv files for more info. 

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
