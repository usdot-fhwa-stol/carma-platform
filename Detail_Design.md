# Detailed Design Page
The Connected and Automated Vehicle (CAV) Platform that is the subject of the STOL II, Task Order 13, and work is being built as a reusable and extensible platform to support research in connected and cooperative vehicle operations over the next several years. The architecture for this platform has been described in the CARMA Platform Architecture Document. This document picks up from that high level description and presents additional details of both the hardware and software design. The design presented here represents an as-built view of the platform, v2.7.2, as of August 2018. [CARMA Detail Design](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)

## Software Package Detailed Designs
The following package diagram is repeated from the Architecture Document for easy reference. It serves as a visual index to the rest of the detailed design. Each package of non-trivial complexity is described in its own separate design document.

![CARMA Arch](docs/image/Software_Designs.png)

## Software Detailed Design Documents
The following table identifies the documents covering each of the packages.  

|Package|Attached Document(s)|
|----|-----------|
|Guidance|[CAV Platform Detailed Design – Guidance](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589642/CAV%20Platform%20Detailed%20Design%20-%20Guidance.docx0)
|Guidance.Arbitrator|[CAV Platform Detailed Design – Arbitrator](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/35323905/CAV%20Platform%20Detailed%20Design%20-%20Arbitrator.docx)
|Guidance.Tracking|[CAV Platform Detailed Design – Guidance.Tracking](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/35618850/CAV%20Platform%20Detailed%20Design%20-%20Guidance.Tracking.docx)
|Guidance.Trajectory|[CAV Platform Detailed Design – Trajectory](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/35717164/CAV%20Platform%20Detailed%20Design%20-%20Trajectory.docx)
|Guidance.Trajectory|[CAV Platform Detailed Design – Trajectory](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Guidance.Maneuvers|[CAV Platform Detailed Design – Maneuvers](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Guidance conflict handling (covers multiple packages)|[CAV Platform Detailed Design – Trajectory](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Guidance.Plugins|[CAV Platform Detailed Design – Plugins](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Guidance.Platooning|[CAV Platform Detailed Design – Plug-ins.Platooning](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Guidance.Lane Change Plugin|[CAV Platform Detailed Design – Plug-ins.Lane Change](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Guidance.Speed Harm Plugin|[CAV Platform Detailed Design – Plugins.SpeedHarmonization](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Transform Sever|[CAV Platform Detailed Design – TransformServer](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Logger|[CAV Platform Detailed Design – Operator UI](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Operator UI|[CAV Platform Detailed Design – Plugins.SpeedHarmonization](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Vehicle Environment.Roadway|[CAV Platform Detailed Design – RoadwayEnvironment](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/35323924/CAV%20Platform%20Detailed%20Design%20-%20RoadwayEnvironment.docx)
|Vehicle Environment.Message|[CAV Platform Detailed Design – Message](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Vehicle Environment.Sensor Fusion|[CAV Platform Detailed Design – Sensor Fusion](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Vehicle Environment.Geometry|[CAV Platform Detailed Design – Geometry](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
||[CAV Platform Detailed Design – Geometry.Cartesian](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
||[CAV Platform Detailed Design – Geometry.Geodesic](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Vehicle Environment.Route|[CAV Platform Detailed Design](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Interface Manager|[CAV Platform Detailed Design – Interface Mgr](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Drivers.Controller|[CAV Platform Detailed Design – SRX Controller Driver](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
||[CAV Platform Detailed Design – Truck Controller Driver](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
||[CAV Platform Detailed Design – XGV Controller Driver](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Drivers.DSRC Comms.|[CAV Platform Detailed Design – DSRC OBU Driver](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Drivers.Position|[CAV Platform Detailed Design – Pinpoint Driver](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Drivers.Sensor|[CAV Platform Detailed Design – SRX Radar Driver](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
||[CAV Platform Detailed Design – Delphi Radar Driver](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|Drivers.CAN|[CAV Platform Detailed Design – SRX CAN Driver](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
||[CAV Platform Detailed Design – Truck CAN Drive](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
|System installation, configuration and startup|[CAV Platform Detailed Design – LaunchScript](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
||[CAV Platform Remote Install and Launch](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
||[CAV Platform Vehicle Configuration](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
||[CAV Platform Vehicle PC Setup](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)
||[CAV Platform Vehicle PC Structure and Use](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)










