function initMap() 
{
  var loc = {lat:13.34555, lng:74.77146};
  var map = new google.maps.Map(document.getElementById('map'), {
          center: loc,
          zoom: 14//,
          // styles: [
          //   {elementType: 'geometry', stylers: [{color: '#242f3e'}]},
          //   {elementType: 'labels.text.stroke', stylers: [{color: '#242f3e'}]},
          //   {elementType: 'labels.text.fill', stylers: [{color: '#746855'}]},
          //   {
          //     featureType: 'administrative.locality',
          //     elementType: 'labels.text.fill',
          //     stylers: [{color: '#d59563'}]
          //   },
          //   {
          //     featureType: 'poi',
          //     elementType: 'labels.text.fill',
          //     stylers: [{color: '#d59563'}]
          //   },
          //   {
          //     featureType: 'poi.park',
          //     elementType: 'geometry',
          //     stylers: [{color: '#263c3f'}]
          //   },
          //   {
          //     featureType: 'poi.park',
          //     elementType: 'labels.text.fill',
          //     stylers: [{color: '#6b9a76'}]
          //   },
          //   {
          //     featureType: 'road',
          //     elementType: 'geometry',
          //     stylers: [{color: '#38414e'}]
          //   },
          //   {
          //     featureType: 'road',
          //     elementType: 'geometry.stroke',
          //     stylers: [{color: '#212a37'}]
          //   },
          //   {
          //     featureType: 'road',
          //     elementType: 'labels.text.fill',
          //     stylers: [{color: '#9ca5b3'}]
          //   },
          //   {
          //     featureType: 'road.highway',
          //     elementType: 'geometry',
          //     stylers: [{color: '#746855'}]
          //   },
          //   {
          //     featureType: 'road.highway',
          //     elementType: 'geometry.stroke',
          //     stylers: [{color: '#1f2835'}]
          //   },
          //   {
          //     featureType: 'road.highway',
          //     elementType: 'labels.text.fill',
          //     stylers: [{color: '#f3d19c'}]
          //   },
          //   {
          //     featureType: 'transit',
          //     elementType: 'geometry',
          //     stylers: [{color: '#2f3948'}]
          //   },
          //   {
          //     featureType: 'transit.station',
          //     elementType: 'labels.text.fill',
          //     stylers: [{color: '#d59563'}]
          //   },
          //   {
          //     featureType: 'water',
          //     elementType: 'geometry',
          //     stylers: [{color: '#17263c'}]
          //   },
          //   {
          //     featureType: 'water',
          //     elementType: 'labels.text.fill',
          //     stylers: [{color: '#515c6d'}]
          //   },
          //   {
          //     featureType: 'water',
          //     elementType: 'labels.text.stroke',
          //     stylers: [{color: '#17263c'}]
          //   }
          // ]
        });

  var locations;
  var marker, i;

  var botCenter = new google.maps.LatLng(0.00,0.00)

  var botArea = new google.maps.Circle({
        strokeColor: '#004ac1',
        strokeOpacity: 0.2,
        strokeWeight: 2,
        fillColor: '#77abff',
        fillOpacity: 0.35,
        map: map,
        center: botCenter,
        radius: 30
      });

  var botMarker = new google.maps.Marker({
        position: botCenter,
        icon: {
            strokeColor: '#FFFFFF',
            fillColor: '#004ac1',
            fillOpacity: 1,
            strokeWeight: 1.5,
            path: google.maps.SymbolPath.CIRCLE,
            scale: 8
          },
        map: map
      });

  var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/vectornav/GPS',
    messageType : 'sensor_msgs/NavSatFix'
  });

  listener.subscribe(function(message) {
    console.log('GPS Fix received ' + listener.name + ': ' + message.latitude + ", " + message.longitude);
    var newLatLng = new google.maps.LatLng(message.latitude, message.longitude);
    botMarker.setPosition(newLatLng);
    botArea.setCenter(newLatLng);
  });

  
  $.getJSON("./coordinates.json", function(json) {
    locations = json;

    var len = Object.keys(locations).length;
    var poly = [];
    for(i=0;i<len;i++)
    {
      poly.push(new google.maps.LatLng(locations[i][0],locations[i][1]));
    }

    var route = new google.maps.Polyline({
      path: poly,
      geodesic: true,
      strokeColor: '#ffde00',
      strokeOpacity: 1.0,
      strokeWeight: 4
    });
    route.setMap(map);

    var bounds = new google.maps.LatLngBounds();
    for (i = 0; i < len; i++) {
        marker = new google.maps.Marker({
        position: new google.maps.LatLng(locations[i][0], locations[i][1]),
        map: map
      });
      bounds.extend(marker.position);
    }
    map.fitBounds(bounds);
  });

  $.getJSON("./intermediate_coordinates.json", function(json) {
    locations = json;

    var len = Object.keys(locations).length;

    for (i = 0; i < len; i++) {
        marker = new google.maps.Marker({
        position: new google.maps.LatLng(locations[i][0], locations[i][1]),
        map: map,
        icon: {
          path: google.maps.SymbolPath.CIRCLE,
          scale: 2,
          strokeColor: '#0000ff'
        }
      });
    }
  });
}
