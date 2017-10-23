package gov.dot.fhwa.saxton.carma.route;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

/**
 * An enumeration which identifies the different types of edges in a lane.
 */
public enum LaneEdgeType {

  /**
   * A solid yellow line which should not be crossed and has opposing traffic on the opposite side
   */
  SOLID_YELLOW(cav_msgs.LaneEdgeType.SOLID_YELLOW),

  /**
   * A solid white line which should not be crossed
   */
  SOLID_WHITE(cav_msgs.LaneEdgeType.SOLID_WHITE),

  /**
   * A broken yellow line which can be crossed to pass but not as a lane merge
   */
  BROKEN_YELLOW(cav_msgs.LaneEdgeType.BROKEN_YELLOW),

  /**
   * A broken white line which can be crossed to pass or merge
   */
  BROKEN_WHITE(cav_msgs.LaneEdgeType.BROKEN_WHITE),

  /**
   * An impassible marking on the road such as construction barriers
   */
  IMPASSIBLE(cav_msgs.LaneEdgeType.IMPASSIBLE);

  private byte value;

  LaneEdgeType(byte value) {
    this.value = value;
  }

  private byte getValue() {
    return value;
  }

  /**
   * Converts an instance of this enum into a ros message
   * @return the instantiated ros message
   */
  public cav_msgs.LaneEdgeType toMessage(){
    MessageFactory factory = NodeConfiguration.newPrivate().getTopicMessageFactory();
    cav_msgs.LaneEdgeType msg = factory.newFromType(cav_msgs.LaneEdgeType._TYPE);
    msg.setType(this.getValue());
    return msg;
  }

  /**
   * Creates an instance of this enum from a ros message
   * @param msg The message to convert
   * @return The equivalent enum
   */
  public static LaneEdgeType fromMessge(cav_msgs.LaneEdgeType msg) {
    switch (msg.getType()) {
      case cav_msgs.LaneEdgeType.SOLID_WHITE:
        return SOLID_WHITE;
      case cav_msgs.LaneEdgeType.SOLID_YELLOW:
        return SOLID_YELLOW;
      case cav_msgs.LaneEdgeType.BROKEN_YELLOW:
        return BROKEN_YELLOW;
      case cav_msgs.LaneEdgeType.BROKEN_WHITE:
        return BROKEN_WHITE;
      default:
        return IMPASSIBLE;
    }
  }

}
