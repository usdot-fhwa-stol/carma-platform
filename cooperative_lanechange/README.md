# cooperative_lanechange

The cooperative_lanechange tactical plugin is responsible for converting high-level lane change maneuvers into detailed lane change trajectories. This plugin generates lane change trajectories for two types of scenarios: (1) "Regular Planning" in which no vehicles are near the host vehicle in the target lane, and (2) "Negotiation" in which communication with a neighboring vehicle is required to open a gap for a safe lane change.

Link to detailed design document on Confluence: [Click Here](https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/1318715418/Detailed+Design+-+Cooperative+Lanechange)