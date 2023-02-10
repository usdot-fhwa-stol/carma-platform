# platoon_control

Platooning Control plugin which allows platooning to maintain the gap; moreover, generates longitudinal and lateral control commands to follow the trajectory. The structure of this plugin is very similar to the control plugin for IHP2, so the same design document is included here.
The difference between platoon_control, and platoon_control_ihp is that the IHP plugin includes logic to open or close the gap between platoon members, to allow for a new member to join or an existing memeber to exit the platoon. 

Overview
https://usdot-carma.atlassian.net/wiki/spaces/CUCS/pages/2392981532/Basic+Travel+and+IHP

Design
https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/2138767361/IHP2+Platooning+Plugin
