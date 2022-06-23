# traffic_incident_parser

The traffic_incident_parser node collects incoming data from the incoming_mobility_operation topic with incident strategy type and parses it. The data received is used to update the necessary lanelets accordingly through CARMA world model API calls to match the received closed lane geometry. Once the lanelet is modified, the sequence of central points is stored in a traffic control message geometry and the traffic control details are updated with lane closed detail. This message is then published to the CARMA world model control node.

Link to detailed design document on Confluence: [Click Here](https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/2172584389/Carma-system-3.10.0+Detailed+Design+-+Traffic+Incident+Parser+Node)
