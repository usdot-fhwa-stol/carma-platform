package gov.dot.fhwa.saxton.carma.roadway;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import org.ros.rosjava_geometry.Transform;

/**
 * Created by mcconnelms on 10/5/17.
 */
public interface IObstacle {
  Vector3D getAcceleration();
  Vector3D getVelocity();
  String getReferenceFrameId();
  Transform getPose();
  boolean collision(IObstacle obj);
  boolean collision(IObstacle obj, long timeSpan, long timeStep);
}
