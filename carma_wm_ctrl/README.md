# carma_wm_ctrl

This packages provides logic for updating a [carma_wm](../carma_wm) compatable lanelet2 map at runtime using [CARMACloud](https://github.com/usdot-fhwa-stol/carma-cloud) geofences.
The carma_wm_broadcaster node sits between the lanelet2 map loader and the rest of the CARMAPlatform system. When a new geofence is received, the map is updated and the update is communicated to the rest of the CARMAPlatform.
If a base map is received which does not contain the carma_wm compatible regulatory elements, the carma_wm_broadcaster node will make an initial best effort attempt to make the map compatible. There is no guarantee that this will work so starting with a compatible map is always recommended.

Design documentation for this package can be found in the carma_wm_ctrl section of [this confluence page.](https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/1324122268/Detailed+Design+-+World+Model+Road+Network+Library)
