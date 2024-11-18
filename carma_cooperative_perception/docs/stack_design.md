# CARMA cooperative perception - Stack design

The cooperative perception stack can take various forms depending on the deployed-to actor’s abilities. The subsections
below describe different configuration alternatives and what type of actors would most likely use them. For all of this
section’s figures, the gray rounded-boxes represent ROS 2 Nodes. Three main concepts distinguish the configurations:

- **Local perception:** an actor can perceive the environment using only its equipped sensors.
- **Local fusion:** an actor can use its own processing capabilities to combine several data sources about an object
  into a single representation.
- **External reliance:** an actor relies on some external entity to provide environment object information. These
  actors lack local perception, local fusion, or both capabilities.

The following table summarizes the different stack configurations.

| Local perception | Local fusion | External reliance |
| ---------------- | ------------ | ----------------- |
| Yes              | Yes          | No                |
| Yes              | No           | No                |
| No               | Yes          | Depends           |
| No               | No           | Yes               |

We assume all actors can receive incoming messages such as SDSMs, whether they can use that data is a different matter.
We also assume actors with local perception abilities have access to object-level data regarding the environment.

## General approach with notations
The CP stack in general has three key stages:

- Detection Generation: 
  - Collects raw detections (D^i_k) from multiple sources (V2X, or local sensors)
  - Each detection has its own timestamp when recorded
  - All detections are in the vehicle's map frame which is handled by the Detection Generators
- MOT (Multiple Object Tracking) Pipeline: Takes N raw detections with varying timestamps
  - Aligns them temporally to create time-synchronized detections
  - Produces M tracks/estimates (T^i_k) where typically N ≥ M
  - Executes within a fixed time window Δt
> **_NOTE:_**: Since this step's implementation can vary, the specific architecture used in CARMA is provided below in "Multiple Object Tracking Pipeline Archecture" section

- Execution Timeline: System runs in discrete steps (k). During each step:
  - Processes detections collected between k-1 and k
  - Uses previous tracks from step k-1
  - Generates new tracks for step k
  - Small time offset ε accounts for detection timing variations

The pipeline efficiently converts asynchronous sensor data into synchronized object tracks that downstream components can use for vehicle planning and control.

![Alt text](assets/Overall_MOT.png)

## Local perception and fusion

> [!IMPORTANT]\
> This stack configuration is currently unsupported because the required Nodes are unimplemented.

In this stack configuration variant, the host vehicle (HV) or host roadside unit (HRSU) can perceive objects using its
local sensors, and it can fuse remotely-received object data from BSMs and SDSMs. Actors using this configuration
broadcast sensor data sharing messages (SDSMs) containing locally-perceived objects. The figure below shows the Node
diagram for this deployment configuration.

![](assets/carma_cooperative_perception_perception_fusion_node_diagram.png)
![](assets/carma_cooperative_perception_perception_fusion_system_diagram.png)

## Local fusion only

> [!IMPORTANT]\
> This stack configuration is currently unsupported because the required Nodes are unimplemented.

Host vehicles (HVs) or host roadside units (HRSUs) using this deployment configuration are unable to perceive the
environment using only their local abilities (i.e., their own sensors). However, they can fuse object information they
receive from nearby actors into single representations using their own computational power. The figure below shows the
Node diagram for this deployment configuration.

Depending on the application or use case, actors with this stack configuration might rely on an external entity to
provide object data. If neighboring actors…

![](assets/carma_cooperative_perception_no_perception_fusion_node_diagram.png)
![](assets/carma_cooperative_perception_no_perception_fusion_system_diagram.png)

## Local Perception Only

> [!IMPORTANT]\
> This stack configuration is currently unsupported because the required Nodes are unimplemented.

In this stack configuration, the host vehicle (HV) or host roadside unit (HRSU) can use object information only from
its local sensors; it cannot fuse that data with any remotely-received object data. Actors using this stack deployment
variant do not use incoming SDSMs, but they still publish their local sensor data with others through SDSMs. The figure
below shows the Node diagram for this deployment configuration.

![](assets/carma_cooperative_perception_perception_no_fusion_node_diagram.png)
![](assets/carma_cooperative_perception_perception_no_fusion_system_diagram.png)

## No Local Perception or Fusion

> [!IMPORTANT]\
> This stack configuration is currently unsupported because the required Nodes are unimplemented.

> [!WARNING]\
> We do not recommend this stack configuration but included it for completeness. If you intent to use this
> configuration in your application, we suggest you reevaluate your approach.

In this stack configuration, the host vehicle (HV) or host roadside unit (HRSU) can neither perceive objects locally
nor fuse object data from nearby actors. The HV or HRSU is entirely dependent on an external system to provide an
authoritative list of environment objects. Essentially, actors with this configuration receive the all the cooperative
perception benefits without contributing themselves. The figure below shows the Node diagram for this deployment
configuration.

![](assets/carma_cooperative_perception_no_perception_no_fusion_node_diagram.png)
![](assets/carma_cooperative_perception_no_perception_no_fusion_system_diagram.png)

## Multiple Object Tracking Pipeline Archecture

A modular pipeline implmented in CARMA CP stack for processing and fusing multi-source object detections into tracks. 
Individual components utilize library functions available in https://github.com/usdot-fhwa-stol/multiple_object_tracking. In other words, this is just one example that is possible from the library.

Pipeline Main Components, for each k step: 
1. Temporal Alignment
- Synchronizes raw detections D from N sources to the host vehicle timestamp
- Outputs time-aligned detections D̄ to k timestamp
- Ensures consistent temporal reference frame for processing
2. Scoring and Gating
- Computes association scores between detections and existing tracks
- Uses MetricVisitor pattern for customizable scoring strategies
- Generates ScoreMap: (track_id, det_id) -> score
- Takes previous tracks (from k-1 step) T̄ as input
3. Association
- Maps detections to existing tracks using AssociationVisitor
- Produces AssociationMap: track_id -> vector<det_id>
- Handles track initialization and termination
4. Track Maintenance
- Updates track states using associated detections
- Manages track lifecycle (creation, updates, deletion)
- Integrates with prediction system for motion estimation
5. Fusion
- Combines associated detections and track data
- Uses FusionVisitor for customizable fusion strategies
- Outputs final fused tracks T̄ of M size for k step

Supporting Components and Steps:
1. Track Cache
- Stores historical track information
- Enables prediction and temporal consistency
2. Predict Track
- Projects track states forward in time
- Supports gating and association processes


![Alt text](assets/MOT_Detailed_Pipeline.png)
