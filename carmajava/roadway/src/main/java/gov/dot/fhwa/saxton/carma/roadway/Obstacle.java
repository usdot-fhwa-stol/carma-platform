package gov.dot.fhwa.saxton.carma.roadway;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import org.ros.rosjava_geometry.Transform;

/**
 * Created by mcconnelms on 10/5/17.
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

  @Override public boolean collision(IObstacle obj, long timeSpan, long timeStep) {
    return false;
  }
}
