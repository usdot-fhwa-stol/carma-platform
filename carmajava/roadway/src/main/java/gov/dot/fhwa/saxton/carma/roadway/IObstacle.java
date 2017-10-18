package gov.dot.fhwa.saxton.carma.roadway;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import org.ros.rosjava_geometry.Transform;

/**
 * A representation of an arbitrary obstacle in the roadway
 * Obstacles have velocity and acceleration and can be checked against for collision
 */
public interface IObstacle {
  /**
   * Get the acceleration of the obstacle in m/s^2
   * @return acceleration
   */
  Vector3D getAcceleration();

  /**
   * Get the velocity of the obstacle in m/s
   * @return the velocity
   */
  Vector3D getVelocity();

  /**
   * Gets the id of the frame relative to which this obstacle is defined
   * @return frame id
   */
  String getReferenceFrameId();

  /**
   * Gets the pose of this obstacle represented as a transform from the reference frame to this obstacles body frame
   * @return transform from reference frame to body frame
   */
  Transform getPose();

  /**
   * True if the specified obj is in collision with this obstacle
   * @param obj Object defined relative to the same frame as this obstacle
   * @return True if in collision
   */
  boolean collision(IObstacle obj);

  /**
   * Checks if an obstacle will collide with this obstacle given a linear interpolation of their positions over time.
   * @param obj The obstacle to compare
   * @param timeSpan The span of time to compare against
   * @param timeStep The resolution of timesteps to check for collusion in
   * @return True if will collide
   */
  boolean collision(IObstacle obj, long timeSpan, long timeStep);
}
