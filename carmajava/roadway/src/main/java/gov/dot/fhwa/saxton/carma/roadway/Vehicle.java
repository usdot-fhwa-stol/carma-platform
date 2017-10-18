package gov.dot.fhwa.saxton.carma.roadway;

/**
 * Class representing a Vehicle on a roadway can be connected or have no communication.
 * In future development Vehicle can be extended to support more specializations such as car or truck.
 */
public class Vehicle extends Obstacle {
  protected boolean connectedVehicle;
}
