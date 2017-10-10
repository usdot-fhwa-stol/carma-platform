package gov.dot.fhwa.saxton.carma.roadway;

import cav_msgs.RouteSegment;
import cav_msgs.SystemAlert;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.apache.commons.logging.Log;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

/**
 * Created by mcconnelms on 10/6/17.
 */
public class EnvironmentWorker {
  Log log;
  IEnvironmentManager envMgr;
  protected boolean headingRecieved = false;
  protected boolean navSatFixRecieved = false;
  protected Location hostVehicleLocation = null;
  protected double hostVehicleHeading = 0.0; // The heading of the vehicle in degrees east of north in an NED frame.
  protected final String earth_frame = "earth";
  protected final String map_frame = "map";
  protected final String odom_frame = "odom";
  protected final String base_link_frame = "base_link";

  public EnvironmentWorker(IEnvironmentManager envMgr, ConnectedNode connectedNode) {
    this.log = connectedNode.getLog();
    this.envMgr = envMgr;
  }

  public void handleCurrentSegmentMsg(cav_msgs.RouteSegment currentSeg) {
    //TODO update lane geometry here. For now we will need to assume linear interpolation
  }

  public void handleHeadingMsg(cav_msgs.HeadingStamped heading) {
    //TODO update host vehicle transform
    hostVehicleHeading = heading.getHeading();
    headingRecieved = true;
    updateMapToOdom();
  }

  public void handleNavSatFixMsg(sensor_msgs.NavSatFix navSatFix) {
    //TODO update map->odom transform
    // Assign the new host vehicle location
    hostVehicleLocation = new Location(navSatFix.getLatitude(), navSatFix.getLongitude(), navSatFix.getAltitude());
    navSatFixRecieved = true;
    updateMapToOdom();
  }

  protected void updateMapToOdom() {
    if (!navSatFixRecieved || !headingRecieved) {
      return; // If we don't have a heading and a gps fix the map->odom transform cannot be calculated
    }
    GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();
    // Get the vehicle location in the map frame
    Point3D hostPoint = gcc.geodesic2Cartesian(hostVehicleLocation, envMgr.getTransform(earth_frame, map_frame));
    // TODO calculate the rest of the transform
  }

  public void handleOdometryMsg(nav_msgs.Odometry odometry) {
    //TODO update host vehicle transform
  }

  public void handleExternalObjectsMsg(cav_msgs.ExternalObjectList externalObjects) {
    //TODO update list of external objects
  }

  public void handleVelocityMsg(geometry_msgs.TwistStamped velocity) {
    //TODO update host vehicle specification (pose)
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
