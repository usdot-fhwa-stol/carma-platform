/***
 This file shall contain Map related functions.
****/

// Global variables
var map;
var markers=[];
var hostmarker;
var bounds;

function initMap() {
    map= new google.maps.Map(document.getElementById('map'), {
        zoom: 18,
        center: { lat: 38.955097, lng: -77.147190 },
        mapTypeId: 'hybrid'
    });

    //Initialize bounds.
    bounds = new google.maps.LatLngBounds();

    //Display the route on the map.
    setRouteMap(map);

    //Set the markers for the vehicle(s).
    setHostMarker();
    setOtherVehicleMarkers();

    //TODO: Fit the map to the newly inclusive bounds of the route initial load.
    //map.fitBounds(bounds);
}


/*
    Draws the route on the map.
*/
function setRouteMap(map){

        //Get the saved route.
        //Parse and loop through the waypoints to get the coordinates.
        var routePlanCoordinates = sessionStorage.getItem("routePlanCoordinates");
        routePlanCoordinates = JSON.parse(routePlanCoordinates);

        if (sessionStorage.getItem('routePlanCoordinates') == null)
        {
            return;
        };

        //Maps the selected route on the map.
        var routePath = new google.maps.Polyline({
          path: routePlanCoordinates,
          geodesic: true,
          strokeColor: '#6495ed', // cornflowerblue
          strokeOpacity: 1.0,
          strokeWeight: 2
        });

        routePath.setMap(map);
}

/*
    Maps the initial DUMMY position of the host vehicle.
*/
function setHostMarker() {

    //Add host vehicle
    var marker = new google.maps.Marker({
        id: 'mHostVehicle',
        position: { lat: 38.95647, lng: -77.15031 },
        map: map,
        icon: 'http://www.google.com/mapfiles/markerH.png',
        title: 'Host Vehicle',
        zIndex: 1
    });

    markers.push(marker);
    hostmarker = marker;  //store instance of the host marker

    ////extend the bounds to include each marker's position
    //bounds.extend(marker.position);
}

/*
    Maps other DUMMY vehicles on the map.
*/
function setOtherVehicleMarkers() {

    // Data for the markers consisting of a name, a LatLng and a zIndex for the
    // order in which these markers should display on top of each other.
    var vehicles = [
        ['mVehicle2', 'Vehicle 2', 38.955117, -77.147111, 2, 'http://maps.google.com/mapfiles/ms/icons/blue-dot.png',],
        ['mVehicle3', 'Vehicle 3',38.955133, -77.147050, 3, 'http://maps.google.com/mapfiles/ms/icons/purple-dot.png'],
        ['mVehicle4', 'Vehicle 4',38.955164, -77.146971, 4, 'http://maps.google.com/mapfiles/ms/icons/yellow-dot.png'],
        ['mVehicle5', 'Vehicle 5',38.955213, -77.146906, 5, 'http://maps.google.com/mapfiles/ms/icons/green-dot.png']
    ];

    // Adds markers to the map.
    for (var i = 0; i < vehicles.length; i++) {
        var vehicle = vehicles[i];

        var marker = new google.maps.Marker({
            id: vehicle[0],
            position: { lat: vehicle[2], lng: vehicle[3] },
            map: map,
            icon: vehicle[5],
            title: vehicle[1],
            zIndex: vehicle[4]
        });

        ////extend the bounds to include each marker's position
        //bounds.extend(marker.position);
    }
}

/*
    Move a marker.
*/
function moveMarkerWithTimeout( myMarker, newLat, newLong, timeout) {

    window.setTimeout(function() {

        myMarker.setPosition(new google.maps.LatLng(newLat, newLong));

        //Center map based on the marker that's moving, for now it's the host vehicle.
        map.setCenter(myMarker.getPosition());
        //OR map.panTo(marker.getPosition());

    }, timeout); //END setTimeout
}


/*
    Find a marker.
*/
function findMarker(allMarkers, idToFind)
{
  for(var i=0; i<allMarkers.length; i++){
        if(allMarkers[i].id === idToFind){
            return allMarkers[i];
        }
   }
}

/*
    Add a marker.
*/
function addMarkerWithTimeout(position, timeout) {
    window.setTimeout(function() {
      markers.push(new google.maps.Marker({
        position: position,
        map: map,
        //animation: google.maps.Animation.DROP
      }));
    }, timeout);
}

/*
    Delete a marker.
*/
function deleteMarker(allMarkers, idToFind)
{
  for(var i=0; i<allMarkers.length; i++){
        if(allMarkers[i].id === idToFind){
            allMarkers[i].setMap(null);
            break;
        }
   }
}
