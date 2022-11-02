# platoon_control_ihp

Control plugin for the Integrated Highway Prototype 2 (IHP2) implementation which allows platooning to maintain the gap; moreover, generates longitudinal and lateral control commands to follow the trajectory. The difference between this plugin, and platoon_control is that this plugin includes logic to open or close the gap between platoon members, to allow for a new member to join or an existing memeber to exit the platoon. 
Note despite the name, as of Aug 2022 testing for this IHP2 implementation was only up to 35mph on a closed test track per limitations of the CARMA Platform system.

Overview
https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/2138079233/Integrated+Highway+Prototype+2+IHP2

Design
https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/2138767361/IHP2+Platooning+Plugin
