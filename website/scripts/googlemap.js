/*
 * Copyright (C) 2017 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

/***
 This file shall contain Map related functions.
****/

// Global variables
var map;
var bounds;
var markers=[];
var hostmarker;

function initMap() {
    map= new google.maps.Map(document.getElementById('map'), {
        zoom: 18,
        center: { lat: 38.955097, lng: -77.147190 },
        mapTypeId: 'hybrid'
    });

    markers =  sessionStorage.getItem('mapMarkers');

    //Display the route on the map.
    setRouteMap(map);

    //Set the markers for the vehicle(s).
    setHostMarker();

}


/*
    Draws the route on the map.
*/
function setRouteMap(map){

        //Get the saved route.
        //Parse and loop through the waypoints to get the coordinates.
        var routePlanCoordinates = sessionStorage.getItem('routePlanCoordinates');
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

    if (markers == null)
        markers = new Array();

    markers.push(marker); // only other vehicles
    hostmarker = marker;  //store instance of the host marker
}

/*
    To paint the pin a particular color.
*/
function pinSymbol(color) {
  return {
    path: 'M 0,0 C -2,-20 -10,-22 -10,-30 A 10,10 0 1,1 10,-30 C 10,-22 2,-20 0,0 z',
    fillColor: color,
    fillOpacity: 1,
    strokeColor: '#000',
    strokeWeight: 2,
    scale: 1
  };
}


/*
    Maps other CAV vehicles on the map.
*/
function setOtherVehicleMarkers(id, latitude, longitude) {

    if (markers == null || markers == 'undefined')
    {
         addMarkerForOtherVehicleWithTimeout(id, latitude, longitude, 0);
         return;
    }

    var targetMarker = findMarker(markers, id);
    if  (targetMarker == null || targetMarker == 'undefined')
    {
        //Vehicle ID is not on the list of markers. Add to the map.
        addMarkerForOtherVehicleWithTimeout(id, latitude, longitude, 0);
        return;
    }
    else{
        //Vehicle ID has been found. Update the location of the marker.
        moveMarkerWithTimeout(targetMarker, latitude, longitude, 0);
        return;
    }

    //update markers
    sessionStorage.setItem("mapMarkers", JSON.stringify(markers));
}

/*
    Find and remove the marker from the Array greater than 3 seconds old.
*/
function removeExpiredMarkers(){
    setTimeout(function () {
        var dateNow = new Date();
        for (var i = 0; i < markers.length; i++) {
            if ( ((dateNow.getTime() - markers[i].dateTimeCreated.getTime())/1000) > 3) {
                //Remove the marker from Map
                markers[i].setMap(null);
                //Remove the marker from array.
                markers.splice(i, 1);
                return;
            }
        }
  }, 3000)//  ..  setTimeout()
}

/*
    Move a marker.
*/
function moveMarkerWithTimeout( myMarker, newLat, newLong, timeout) {

    window.setTimeout(function() {

        myMarker.setPosition(new google.maps.LatLng(newLat, newLong));

        if (myMarker.id == 'mHostVehicle')
        {
            //Center map based on the marker that's moving, for now it's the host vehicle.
            map.setCenter(myMarker.getPosition());
        }
    }, timeout); //END setTimeout
}


/*
    Find a marker.
*/
function findMarker(allMarkers, idToFind)
{
  if (allMarkers == null)
    return;

  for(var i=0; i < allMarkers.length; i++){
        if(allMarkers[i].id == idToFind){
            return allMarkers[i];
        }
   }
}

/*
    Add a marker.
*/
function addMarkerForOtherVehicleWithTimeout(newId, newLat, newLong, timeout) {

    window.setTimeout(function() {
      markers.push(new google.maps.Marker({
        id: newId,
        position: new google.maps.LatLng(newLat, newLong),
        map: map,
        title: newId,
        dateTimeCreated: new Date(),
      }));

    }, timeout);
}

/*
    Add a plain marker.
*/
function addMarkerWithTimeout(position, timeout) {
    window.setTimeout(function() {
      markers.push(new google.maps.Marker({
        position: position,
        map: map,
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

/*
    Delete a marker by ID and removing from array
*/
function DeleteMarker(id) {

    //Find and remove the marker from the Array
    for (var i = 0; i < markers.length; i++) {

        if (markers[i].id == id) {

            //Remove the marker from Map
            markers[i].setMap(null);

            //Remove the marker from array.
            markers.splice(i, 1);
            return;
        }

    }

};