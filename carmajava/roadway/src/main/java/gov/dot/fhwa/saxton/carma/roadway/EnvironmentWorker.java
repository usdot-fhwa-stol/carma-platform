package gov.dot.fhwa.saxton.carma.roadway;

import cav_msgs.RouteSegment;
import cav_msgs.SystemAlert;
import org.apache.commons.logging.Log;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

/**
 * Created by mcconnelms on 10/6/17.
 */
public class EnvironmentWorker {
  Log log;
  IEnvironmentManager envMgr;

  public EnvironmentWorker(IEnvironmentManager envMgr, ConnectedNode connectedNode) {
    this.log = connectedNode.getLog();
    this.envMgr = envMgr;
  }

  public void handleCurrentSegmentMsg(cav_msgs.RouteSegment currentSeg) {

  }

  public void handleHeadingMsg(cav_msgs.HeadingStamped heading) {

  }

  public void handleNavSatFixMsg(sensor_msgs.NavSatFix navSatFix) {

  }

  public void handleOdometryMsg(nav_msgs.Odometry odometry) {

  }

  public void handleExternalObjectsMsg(cav_msgs.ExternalObjectList externalObjects) {

  }

  public void handleVelocityMsg(geometry_msgs.TwistStamped velocity) {

  }

  public void handleSystemAlertMsg(cav_msgs.SystemAlert alert) {
    switch (alert.getType()) {
      case SystemAlert.DRIVERS_READY:
        break;
      case SystemAlert.NOT_READY:
        break;
      case SystemAlert.SHUTDOWN:
        log.info("EnvironmentWorker: Received SHUTDOWN on system_alert");
        envMgr.shutdown();
        break;
      case SystemAlert.FATAL:
        log.info("EnvironmentWorker: Received FATAL on system_alert");
        envMgr.shutdown();
        break;
      default:
        // No need to handle other types of alert
    }
  }
}
