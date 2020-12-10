# carma_wm_ctrl

This packages provides logic for updating a [carma_wm](../carma_wm) compatable lanelet2 map at runtime using [CARMACloud](https://github.com/usdot-fhwa-stol/carma-cloud) geofences.
The carma_wm_broadcaster node sits between the lanelet2 map loader and the rest of the CARMAPlatform system. When a new geofence is recieved, the map is updated and the update is communicated to the rest of the CARMAPlatform.
If a base map is recieved which does not contain the carma_wm compatable regulatory elements, the carma_wm_broadcaster node will make an initial best effort attempt to make the map compatable. There is no guarenetee that this will work so starting with a compatible map is always recommended.
