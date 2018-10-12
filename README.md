![CARMA Arch](docs/image/CARMA2_Cadillacs_platooning.jpg)

**Connected Automated Research for Mobility Applications (CARMA)**, CARMA is a reusable, extensible platform for controlling SAE level 1 connected, automated vehicles (CAVs). It provides a rich, generic API for third party plugins that implement vehicle guidance algorithms. It is written in Java and C++ and runs in a Robot Operating System (ROS) environment. It offers a rich application programming interface (API) for third party authors to develop guidance plugins that can plan vehicle trajectories under certain conditions. The platform can be reused as-is on a variety of properly modified vehicles, with only a need for new device drivers to connect to the vehicle hardware.

## What Is CARMA
![CARMA Arch](docs/image/CARMA2_Architecture.png)

Managing automated vehicle motion involves three aspects. The first is **navigation**, which is the act of determining where the vehicle currently is with respect to the earth and with respect to the desired path of travel (its planned route). The second is **guidance**, which includes the processes of determining how the vehicle is to move from its current location to its destination. The destination and route will be handed to the guidance algorithms, and they then determine how the vehicle’s motion needs to be adjusted at any time in order to follow the route. The third aspect of **automated vehicle motion is control**, which covers the actuation of the vehicle’s physical devices to induce changes in motion (for land vehicles these are typically causing the wheels to rotate faster or slower and turning the steering wheel). Therefore, the navigation solution becomes an input to the guidance function, and the guidance solution becomes an input to the control function. As the vehicle moves, obviously its location changes so that the navigation function constantly needs to update its solution and the cycle iterates as quickly as necessary to produce a smooth and accurate vehicle motion. The rate of iteration is largely determined by the expected speed of the vehicle.

## Documentation

![CARMA Demo](docs/image/CARMA2_Platooning_from_Office.jpg)

### System Specifications
The platform's performance requirements are specified to conform the following performance requirements. Performance requirements shall only be applicable to the platform being developed, including Android tablets and GUIs; computers, software, positioning units, and processors that are not part of the OEM vehicle. The functional requirements are specified to confirm the following functional requirements.  Functional requirements shall be applicable to the platform being developed, including the vehicle's control, MAP and SPAT messages, etc.
[Performance Requirements](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/11304971/CARMA2+Draft+Performance+Requirements) 
[Functional Requirements](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/7864327/CARMA2+Functional+Requirements)

### Release Notes
The current version release of the CARMA software platform. [CARMA Release Notes](<docs/Release_notes.md>)

### Architecture & Design
The documentation describes the architecture of a single CARMA vehicle.  There may be several of these vehicles operating in concert, communicating with each other via DSRC or cellular means.  They can also communicate with roadside infrastructure using DSRC or cellular.  The communication among software components within a single vehicle, however, is mostly done via the Robot Operating System (ROS) framework.  There are no ROS communications between neighboring vehicles. [CAV Platform Architecture Document](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/24838181/CAV%20Platform%20Architecture.docx) 

### Developers Guide 

### Users Guide
This guide describes the PC configuration steps necessary to install and run the CARMA platform. The steps match what the development team performed on their own PCs and some configuration may be different for other users. [CARMA User Guide](https://usdot-carma.atlassian.net/wiki/spaces/CAR/pages/23330913/CARMA+Project+Documentation?preview=/23330913/24805504/CARMA%20User%20Guide.docx) 

### Administrator Guide

#### Contribution
Welcome to the CARMA contributing guide. Please read this guide to learn about our development process, how to propose pull requests and improvements, and how to build and test your changes to this project. [CARMA Contributing Guide](docs/Contributing.md) 

## License
By contributing to the Federal Highway Administration (FHWA) Connected Automated Research for Mobility Applications (CARMA), you agree that your contributions will be licensed under its Apache License 2.0 license. [CARMA License](<docs/License.md>) 

## Contact

![CARMA Image](docs/image/CARMA_icon.png)











