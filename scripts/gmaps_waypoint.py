#!/usr/bin/env python

# Created By Sarathkrishnan Ramesh on 22/01/2019

from __future__ import print_function
import os
import json
import threading
from datetime import datetime
import googlemaps
import rospkg


class GMapsWaypoint(object):
    def __init__(self):
        self.pkg = rospkg.RosPack().get_path('waypoint_server')
        print(self.pkg)

    def get_api_key(self):
        file_name = self.pkg + "/waypoint_viewer/api_key.txt"
        with open(file_name) as infile:
            apikey = infile.readline()
            infile.close()
        return apikey

    def get_gmaps_waypoint(self, data):
        print("latitude: ", data.latitude)
        print("longitude: ", data.longitude)

        origin = data.latitude, data.longitude
        dest = raw_input("Enter Destination: ")

        gmaps = googlemaps.Client(key=self.get_api_key())

        now = datetime.now()
        directions = gmaps.directions(origin, dest, mode="driving",\
          departure_time=now)[0]

        coordinates = self.decode_polyline(\
          directions['overview_polyline']['points'])

        file_name = self.pkg + "/waypoint_viewer/coordinates.json"
        with open(file_name, 'wb') as outfile:
            data = json.dumps(coordinates, indent=4, sort_keys=False)
            outfile.write(data.encode('UTF-8'))
            outfile.flush()

        print("Opening Waypoint Viewer...")
        t = threading.Thread(target=self.open_waypoint_viewer)
        t.start()

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
