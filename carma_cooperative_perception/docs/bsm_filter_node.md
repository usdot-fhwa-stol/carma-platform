# BSM filter Node

> [!IMPORTANT]\
> This Node is currently unimplemented.

This Node removes objects from incoming `ExternalObjectList.msg` messages if they correspond to actors that broadcast
BSMs. The SAE J3224 standard requires SDSM-broadcasting actors to exclude object data relating to actors that broadcast
BSMs. Quoting a relevant portion from Section 4.1,

> SAE J3224-capable vehicles and SAE J3224-capable RSUs will not transmit SDSMs about vehicles that they have received
> a BSM from, as such vehicles are already capable of transmitting relevant data about themselves to other V2X vehicles.

Internally, this Node calculates the Euclidean distance between locally-perceived objects and received BSMs and removes
objects that fall below a closeness threshold. Note that other distance metrics can be used to redefine the "closeness"
notion.

## Subscriptions

| Topic                      | Message Type                                   | Description               |
| -------------------------- | ---------------------------------------------- | ------------------------- |
| `~/input/external_objects` | `carma_perception_msgs/ExternalObjectList.msg` | Incoming external objects |
| `~/input/bsms`             | `carma_v2x_msgs/BSM.msg`                       | Incoming BSMs             |

## Publishers

| Topic                       | Message Type                                   | Frequency           | Description |
| --------------------------- | ---------------------------------------------- | ------------------- | ----------- |
| `~/output/external_objects` | `carma_perception_msgs/ExternalObjectList.msg` | Subscription-driven |
| Filtered external objects   |

## Parameters

| Topic                 | Data Type | Default Value | Required | Read Only | Description                                                                      |
| --------------------- | --------- | ------------- | -------- | --------- | -------------------------------------------------------------------------------- |
| `~/istance_threshold` | `float`   | `0.0`         | Yes      | Yes       | Distance below which a detected object and BSM object will be considered related |

## Services

This Node does not provide services.

## Actions

This Node does not provide actions.
