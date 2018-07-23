package gov.dot.fhwa.saxton.carma.plugins.speedharm;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;

/**
 * Helper class to test the SpeedHarmonizationManeuver class with
 */
public class FakeManeuverInputForSpeedHarm implements IManeuverInputs {
  private double distanceFromRouteStart = 0.0;
  private double currentSpeed = 0.0;
  private double responseLag = 0.0;
  private double distanceToFrontVehicle = 50.0;
  private double frontVehicleSpeed = 20.0;

  public void setDistanceFromRouteStart(double distanceFromRouteStart) {
    this.distanceFromRouteStart = distanceFromRouteStart;
  }

  public void setCurrentSpeed(double currentSpeed) {
    this.currentSpeed = currentSpeed;
  }

  public void setResponseLag(double responseLag) {
    this.responseLag = responseLag;
  }

  public void setDistanceToFrontVehicle(double distanceToFrontVehicle) {
    this.distanceToFrontVehicle = distanceToFrontVehicle;
  }

  public void setFrontVehicleSpeed(double frontVehicleSpeed) {
    this.frontVehicleSpeed = frontVehicleSpeed;
  }

  @Override public double getDistanceFromRouteStart() {
    return distanceFromRouteStart;
  }

  @Override public double getCurrentSpeed() {
    return currentSpeed;
  }

  @Override public double getResponseLag() {
    return responseLag;
  }

  @Override public double getDistanceToFrontVehicle() {
    return distanceToFrontVehicle;
  }

  @Override public double getFrontVehicleSpeed() {
    return frontVehicleSpeed;
  }

  @Override
  public int getCurrentLane() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getCrosstrackDistance() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getMaxAccelLimit() {
    // TODO Auto-generated method stub
    return 0;
  }
}
