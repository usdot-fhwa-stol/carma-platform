# Detailed Design Page
The Connected and Automated Vehicle (CAV) Platform that is the subject of the STOL II, Task Order 13, and work is being built as a reusable and extensible platform to support research in connected and cooperative vehicle operations over the next several years. The architecture for this platform has been described in the CARMA Platform Architecture Document. This document picks up from that high level description and presents additional details of both the hardware and software design. The design presented here represents an as-built view of the platform, v2.7.2, as of August 2018. [CARMA Detail Design](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/29589570/CAV%20Platform%20Detailed%20Design.docx)

## Software Package Detailed Designs
The following package diagram is repeated from the Architecture Document for easy reference. It serves as a visual index to the rest of the detailed design. Each package of non-trivial complexity is described in its own separate design document.

![CARMA Arch](docs/image/Software_Designs.png)

## Software Detailed Design Documents
The following table identifies the documents covering each of the packages.  

|Package|Attached Document(s)|
|----|-----------|
|Guidance|CAV Platform Detailed Design – Guidance.docx|
|Guidance.Arbitrator|CAV Platform Detailed Design – Arbitrator.docx|
|Guidance.Tracking|CAV Platform Detailed Design – Guidance.Tracking.docx|
|Guidance.Trajectory|CAV Platform Detailed Design – Trajectory.docx|
|Guidance.Maneuvers|CAV Platform Detailed Design – Maneuvers.docx|
|Guidance conflict handling - covers multiple packages|CAV Platform Detailed Design – Conflict Handling System.docx|
|Guidance.Plugins|CAV Platform Detailed Design – Plugins.docx|
|Guidance.Platooning|CAV Platform Detailed Design – Plug-ins.Platooning.docx|
|Guidance.Lane Change Plugin|CAV Platform Detailed Design – Plug-ins.Lane Change.docx|
|Guidance.Speed Harm Plugin|CAV Platform Detailed Design – Plugins.SpeedHarmonization.docx|




|Name|Visibility|Description|
|----|----------|-----------|
|[jpo-ode](https://github.com/usdot-jpo-ode/jpo-ode)|public|Contains the public components of the application code.|
|[jpo-cvdp](https://github.com/usdot-jpo-ode/jpo-cvdp)|public|Privacy Protection Module|
|[jpo-security](https://github.com/usdot-jpo-ode/jpo-security)|public|Security dependencies.|
|[asn1_codec](https://github.com/usdot-jpo-ode/asn1_codec)|public|ASN.1 Encoder/Decoder module|
|jpo-ode-private|private|Proprietary dependencies.|
|[jpo-security-svcs](https://github.com/usdot-jpo-ode/jpo-security-svcs)|public|Provides cryptographic services.|
