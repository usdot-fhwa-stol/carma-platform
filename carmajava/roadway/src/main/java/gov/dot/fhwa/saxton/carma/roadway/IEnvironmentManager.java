package gov.dot.fhwa.saxton.carma.roadway;

import org.ros.message.Time;
import org.ros.rosjava_geometry.Transform;
import tf2_msgs.TFMessage;

/**
 * Interface defines the needed functions of a route manager
 */
public interface IEnvironmentManager {

  /**
   * Publishes a tf2 transform ros message
   */
  void publishTF(TFMessage tfMessage);

  /**
   * Publishes a system alert ros message
   */
  void publishSystemAlert(cav_msgs.SystemAlert alertMsg);

  /**
   * Publishes a roadway environment ros message
   */
  void publishRoadwayEnvironment(cav_msgs.RoadwayEnvironment roadwayEnvMsg);

  /**
   * Gets the transform of between the requested frames
   */
  Transform getTransform(String parentFrame, String childFrame);

  /**
   * Gets the current time
   *
   * @return The time
   */
  Time getTime();

  /**
   * Safely shutdown the node
   */
  void shutdown();

}
