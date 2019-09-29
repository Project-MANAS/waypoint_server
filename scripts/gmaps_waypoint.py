#!/usr/bin/env python

# Created By Sarathkrishnan Ramesh on 22/01/2019

from __future__ import print_function
import os
import json
import threading
from datetime import datetime
import googlemaps
import rospkg
from math import radians, degrees, asin, sin, cos, sqrt, atan2, pi, sqrt, pow, modf
from geopy import distance


class GMapsWaypoint(object):
    def __init__(self):
        self.pkg = rospkg.RosPack().get_path('waypoint_server')
        self.interval = 10.0
        self.intermediate_waypoints = []

    def get_api_key(self):
        file_name = self.pkg + "/waypoint_viewer/api_key.txt"
        with open(file_name) as infile:
            apikey = infile.readline()
            infile.close()
        return apikey
    
    def bearing(self, wp_source, wp_destination):
        lat_wp, lon_wp, lat_origin, lon_origin = map(
            radians, [wp_destination[0], wp_destination[1], wp_source[0], wp_source[1]])
        d_lon = lon_wp - lon_origin
        return atan2(
            sin(d_lon) * cos(lat_wp),
            ((cos(lat_origin) * sin(lat_wp)) -
             (sin(lat_origin) * cos(lat_wp) * cos(d_lon))))

    def get_destination_coordinates(self, wp_source, wp_bearing, wp_dist):
        lat_source, lon_source = map(radians, [wp_source[0], wp_source[1]])

        delta = wp_dist / (distance.EARTH_RADIUS * 1000)
        lat_wp = asin(
            sin(lat_source) * cos(delta) +
            cos(lat_source) * sin(delta) *
            cos(wp_bearing))

        lon_wp = lon_source + atan2(
            sin(wp_bearing) * sin(delta) * cos(lat_source),
            cos(delta) - sin(lat_source) * sin(lat_wp))

        lat_wp, lon_wp = map(degrees, [lat_wp, lon_wp])

        return (lat_wp, lon_wp)

    def get_intermerdiate_waypoints(self, wp_source, wp_destination):
        bearing_to_destination = self.bearing(wp_source, wp_destination)
        distance_to_destination = distance.distance(wp_source, wp_destination).m
        remaining, dist = modf((distance_to_destination / self.interval))

        self.intermediate_waypoints.append(wp_source)

        coordinates = wp_source
        next_distance = self.interval

        for i in range(int(dist)):
            coordinates = self.get_destination_coordinates(wp_source, bearing_to_destination, next_distance)
            next_distance += self.interval
            self.intermediate_waypoints.append(coordinates)
        
        self.intermediate_waypoints.append(wp_destination)


    def get_gmaps_waypoint(self):
        origin = input("Enter Source: ")
        dest = input("Enter Destination: ")
        self.interval = int(input("Enter waypoint interval (meters): "))

        gmaps = googlemaps.Client(key=self.get_api_key())

        now = datetime.now()
        directions = gmaps.directions(origin, dest, mode="driving", departure_time=now)[0]

        coordinates = self.decode_polyline(directions['overview_polyline']['points'])
        
        for i in range(len(coordinates)-1):
            self.get_intermerdiate_waypoints(coordinates[i], coordinates[i+1])

        print(len(coordinates), len(self.intermediate_waypoints))

        file_name = self.pkg + "/waypoint_viewer/coordinates.json"
        with open(file_name, 'wb') as outfile:
            data = json.dumps(coordinates, indent=4, sort_keys=False)
            outfile.write(data.encode('UTF-8'))
            outfile.flush()

        file_name = self.pkg + "/waypoint_viewer/intermediate_coordinates.json"
        with open(file_name, 'wb') as outfile:
            data = json.dumps(self.intermediate_waypoints, indent=4, sort_keys=False)
            outfile.write(data.encode('UTF-8'))
            outfile.flush()

        # print("Opening Waypoint Viewer...")
        # t = threading.Thread(target=self.open_waypoint_viewer)
        # t.start()

    def open_waypoint_viewer(self):
        os.system("firefox " + self.pkg + "/waypoint_viewer/home.html")

    @staticmethod
    def decode_polyline(polyline_str):
        index, lat, lng = 0, 0, 0
        coordinates = []
        changes = {'latitude': 0, 'longitude': 0}

        while index < len(polyline_str):
            for unit in ['latitude', 'longitude']:
                shift, result = 0, 0

                while True:
                    byte = ord(polyline_str[index]) - 63
                    index += 1
                    result |= (byte & 0x1f) << shift
                    shift += 5
                    if not byte >= 0x20:
                        break

                if result & 1:
                    changes[unit] = ~(result >> 1)
                else:
                    changes[unit] = (result >> 1)

            lat += changes['latitude']
            lng += changes['longitude']

            coordinates.append((lat / 100000.0, lng / 100000.0))

        return coordinates


def main():
    g = GMapsWaypoint()
    g.get_gmaps_waypoint()

if __name__ == "__main__":
    main()
