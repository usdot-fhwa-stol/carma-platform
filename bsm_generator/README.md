# bsm_generator

The bsm_generator package contains a node that subscribes to various vehicle data topics in the CARMA System (including topics for vehicle speed, longitudinal acceleration, transmission state, and more). Using this received data, the BSM Generator node composes a BSM message and publishes the message at a fixed rate.