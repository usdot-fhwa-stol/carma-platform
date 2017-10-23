package gov.dot.fhwa.saxton.carma.roadway;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import org.ros.rosjava_geometry.Transform;

/**
 * TODO implement
 * An implementation of IObstacle with no special attributes.
 */
public class Obstacle implements IObstacle {
  @Override public Vector3D getAcceleration() {
    return null;
  }

  @Override public Vector3D getVelocity() {
    return null;
  }

  @Override public String getReferenceFrameId() {
    return null;
  }

  @Override public Transform getPose() {
    return null;
  }

  @Override public boolean collision(IObstacle obj) {
    return false;
  }

  @Override public boolean collision(IObstacle obj, long timeSpan) {
    return false;
  }
}
