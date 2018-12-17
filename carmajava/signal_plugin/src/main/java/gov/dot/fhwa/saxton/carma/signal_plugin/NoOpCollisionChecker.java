package gov.dot.fhwa.saxton.carma.signal_plugin;

import java.util.List;

import cav_msgs.RoadwayObstacle;

import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * No-Op Collision Checker for the Traffic Signal Plugin. Allows NCV handling to be effectively turned off by injecting this as the primary ITrafficSignalPluginCollisionChecker
 */
public class NoOpCollisionChecker implements ITrafficSignalPluginCollisionChecker {

  @Override
  public boolean hasCollision(List<Node> trajectory, double timeOffset, double distanceOffset) {
    return false;
  }

  @Override
  public void updateObjects(List<RoadwayObstacle> obstacles) {
    // Do Nothing
  }

  @Override
  public void setHostPlan(List<Node> hostPlan, double startTime, double startDowntrack) {
    // Do Nothing
  }

}
