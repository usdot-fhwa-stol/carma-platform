package gov.dot.fhwa.saxton.carma.roadway;

import cav_msgs.RouteSegment;
import cav_msgs.SystemAlert;
import geometry_msgs.TransformStamped;
import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.apache.commons.logging.Log;
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;
import std_msgs.Header;
import tf2_msgs.TFMessage;

import java.util.Arrays;

/**
 * Created by mcconnelms on 10/6/17.
 */
public class EnvironmentWorker {
  protected final String earth_frame = "earth";
  protected final String map_frame = "map";
  protected final String odom_frame = "odom";
  protected final String base_link_frame = "base_link";

  protected Log log;
  protected IEnvironmentManager envMgr;
  protected boolean headingRecieved = false;
  protected boolean navSatFixRecieved = false;
  protected Location hostVehicleLocation = null;
  protected double hostVehicleHeading = 0.0;
    // The heading of the vehicle in degrees east of north in an NED frame.

  protected Transform mapToOdom = null;
  protected Transform odomToBaseLink = null;
  protected int tfSequenceCount = 0;

  public EnvironmentWorker(IEnvironmentManager envMgr, ConnectedNode connectedNode) {
    this.log = connectedNode.getLog();
    this.envMgr = envMgr;
  }

  public void handleCurrentSegmentMsg(cav_msgs.RouteSegment currentSeg) {
    //TODO update lane geometry here. For now we will need to assume linear interpolation
  }

  public void handleHeadingMsg(cav_msgs.HeadingStamped heading) {
    //TODO updates
    hostVehicleHeading = heading.getHeading();
    headingRecieved = true;
    Vector3 zAxis = new Vector3(0,0,1);
    Quaternion hostOrientation = Quaternion.fromAxisAngle(zAxis, hostVehicleHeading);
    odomToBaseLink = new Transform(odomToBaseLink.getTranslation(), hostOrientation);
  }

  public void handleNavSatFixMsg(sensor_msgs.NavSatFix navSatFix) {
    // Assign the new host vehicle location
    hostVehicleLocation =
      new Location(navSatFix.getLatitude(), navSatFix.getLongitude(), navSatFix.getAltitude());
    navSatFixRecieved = true;
    mapToOdom = ecefToNEDFromLocaton(hostVehicleLocation);
    publishTF(mapToOdom, map_frame, odom_frame);
  }

  //TODO deprecated
  protected void updateMapToOdom() {
    if (!navSatFixRecieved || !headingRecieved) {
      return; // If we don't have a heading and a gps fix the map->odom transform cannot be calculated
    }

    //mapToOdom = ecefToNEDFromLocaton(hostVehicleLocation);
    // Check if this will be the first calculated transform
    if (mapToOdom == null) {
      // Find an NED frame to identify location of north axis
      Transform initialNED = ecefToNEDFromLocaton(hostVehicleLocation);
      // Rotate NED frame by heading so it lines up with vehicle
      Vector3 zAxis = new Vector3(0,0,1);
      Vector3 identity = new Vector3 (0,0,0);
      Transform rotateByHeading = new Transform(identity, Quaternion.fromAxisAngle(zAxis, hostVehicleHeading));
      // Multiply NED by heading to get starting odom location as Front Right Down (FRD) frame
      mapToOdom = initialNED.multiply(rotateByHeading);
    }

    GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();
    Point3D hostInMap =
      gcc.geodesic2Cartesian(hostVehicleLocation, envMgr.getTransform(map_frame, earth_frame));
    // T_x_y = transform describing location of y with respect to x
    // m = map frame
    // n = nav sat fix frame
    // o = odom frame
    // b = baselink frame (as has been calculated by odometry up to this point)
    // T_n_b = inv(T_m_n) * T_m_o * T_o_b; This is equivalent to the difference between where odom should be and where it is
    Vector3 nTranslation = new Vector3(hostInMap.getX(), hostInMap.getY(), hostInMap.getZ());
    Quaternion hostRotInMap = quaternionFromYPR(hostVehicleHeading.yaw, hostVehicleHeading.pitch, hostVehicleHeading.roll);
    Transform T_m_n = new Transform(nTranslation, hostRotInMap);
    Transform T_m_o = mapToOdom;
    Transform T_o_b = odomToBaseLink;

    Transform T_n_b = T_m_n.invert().multiply(T_m_o.multiply(T_o_b)); // TODO validate that the rosjava transforms uses this order of multiplication

    mapToOdom = mapToOdom.multiply(T_n_b);

//    Vector3 realLocInOldOdom = new Vector3(hostInOldOdom.getX(), hostInOldOdom.getY(), hostInOldOdom.getZ());
//    Vector3 estimatedLocInOldOdom = odomToBaseLink.getTranslation();
//    Vector3 error = realLocInOldOdom.subtract(estimatedLocInOldOdom);
//
//    Quaternion rotationOdomToBaseLink = odomToBaseLink.getRotationAndScale().normalize();
//
//    double rollError = hostVehicleHeading.roll;
//    double pitchError = hostVehicleHeading.pitch;
//    double yawError = hostVehicleHeading.yaw;
  }

  protected Quaternion quaternionFromYPR(double yaw, double pitch, double roll) {
    // Heading pitch bank
    double halfYaw = yaw / 2.0;
    double halfPitch = pitch / 2.0;
    double halfRoll = roll / 2.0;

    double c1 = Math.cos(halfYaw);
    double c2 = Math.cos(halfPitch);
    double c3 = Math.cos(halfRoll);
    double s1 = Math.sin(halfYaw);
    double s2 = Math.sin(halfPitch);
    double s3 = Math.sin(halfRoll);
    double c1c2 = c1*c2; // Might be unneeded optimization
    double s1s2 = s1*s2;
    double s1c2 = s1*c2;
    double c1s2 = c1*s2;

    double w = c1c2*c3 - s1s2*s3;
    double x = s1s2*c3 + c1c2*s3;
    double y = s1c2*c3 + c1s2*s3;
    double z = c1s2*c3 - s1c2*s3;
    return new Quaternion(x,y,z,w);
  }

  //TODO UNIT TEST THIS!!!
  protected Transform ecefToNEDFromLocaton(Location loc) {
    GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();
    Point3D hostInMap =
      gcc.geodesic2Cartesian(loc, envMgr.getTransform(map_frame, earth_frame)); //TODO validate that this works even with an earth map transform

    Vector3 trans = new Vector3(hostInMap.getX(), hostInMap.getY(), hostInMap.getZ());

    // Rotation matrix of north east down frame with respect to ecef
    double sinLat = Math.sin(hostVehicleLocation.getLatRad());
    double sinLon = Math.sin(hostVehicleLocation.getLonRad());
    double cosLat = Math.cos(hostVehicleLocation.getLatRad());
    double cosLon = Math.cos(hostVehicleLocation.getLonRad());
    double[][] R = new double[][] {
      { -sinLat * cosLon, -sinLon,  -cosLat * cosLon },
      { -sinLat * sinLon,  cosLon,  -cosLat * sinLat },
      {           cosLat,       0,           -sinLat }
    };
    double qw = Math.sqrt(1.0 + R[0][0] + R[1][1] + R[2][2]) / 2.0;
    double qw4 = 4.0 * qw;
    double qx = (R[2][1] - R[1][2]) / qw4;
    double qy = (R[0][2] - R[2][0]) / qw4;
    double qz = (R[1][0] - R[0][1]) / qw4;
    Quaternion quat = new Quaternion(qx, qy, qz, qw);

    return new Transform(trans, quat);
  }

  public void handleOdometryMsg(nav_msgs.Odometry odometry) {
    // TODO I drop the covariance here. Does the covariance matter if we are getting this from sensor fusion?
    geometry_msgs.Pose hostPose = odometry.getPose().getPose();
    geometry_msgs.Point hostPoint = hostPose.getPosition();
    Quaternion hostOrientation = Quaternion.fromQuaternionMessage(hostPose.getOrientation());
    Vector3 trans = new Vector3(hostPoint.getX(), hostPoint.getY(), hostPoint.getZ());
    odomToBaseLink = new Transform(trans, hostOrientation);
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

  /**
   * Helper function builds a tf2 message for the given transform between parent and child frames
   * @param tf The transform to publish. Describes the position of the child frame in the parent frame
   * @param parentFrame The name of the parent frame
   * @param childFrame The name of the child frame
   */
  protected void publishTF(Transform tf, String parentFrame, String childFrame) {
    MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();
    TFMessage tf2Msg = messageFactory.newFromType(TFMessage._TYPE);
    geometry_msgs.TransformStamped tfStampedMsg = messageFactory.newFromType(geometry_msgs.TransformStamped._TYPE);
    Header hdr = tfStampedMsg.getHeader();
    hdr.setFrameId(parentFrame);
    hdr.setStamp(envMgr.getTime());
    hdr.setSeq(tfSequenceCount);
    tfStampedMsg.setChildFrameId(childFrame);
    tfStampedMsg.setTransform(tf.toTransformMessage(tfStampedMsg.getTransform()));
    tf2Msg.setTransforms(Arrays.asList(tfStampedMsg));
    envMgr.publishTF(tf2Msg);
    tfSequenceCount++;
  }
}
