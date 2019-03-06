# Waypoint Server

 :round_pushpin: Waypoint server ROS package with Google Maps integration.

#### Google Maps integration
Create a Google Maps Developer account and generate an **API Key** with **Maps** and **Places** options selected.  
Inside the repository type the following:  

    echo "YOUR_API_KEY" > waypoint_viewer/api_key.txt

Install Google Maps API for python

    pip install googlemaps


#### Usage
- Use catkin to build the ROS package.
- Inside the launch file set the following parameters:  
  - Threshold distance (meters)
  - GPS topic: `gps_topic`
  - Odometry topic:`odom_topic`
  - Generate Google Maps waypoints
- Run the rosnode  

      roslaunch waypoint_server waypoint_server.launch
