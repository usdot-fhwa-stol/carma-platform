package gov.dot.fhwa.saxton.carma.guidance.plugins;

import cav_msgs.Route;
import cav_msgs.RouteSegment;
import cav_msgs.RouteState;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.OnMessageCallback;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.logging.Log;

/**
 * Cruising Plugin
 * </p>
 * Implements the basic behavior of commmanding the speed limit (as specified by RouteManager)
 * at each route segment.
 */
public class CruisingPlugin extends AbstractPlugin {
  protected String name = "Cruising";
  protected String versionId = "v00.00.01";
  protected ISubscriber<RouteState> routeStateSub;
  protected ISubscriber<Route> routeSub;
  protected ISubscriber<RouteSegment> currentSegmentSub;

  protected AtomicReference<Route> currentRoute;
  protected AtomicReference<RouteSegment> currentSegment;
  protected List<SpeedLimit> speedLimits;
  public static final double MPH_TO_MPS = 0.44704; // From Google calculator

  private class SpeedLimit {
    double speedLimit;
    double location;
  }
  
  private class TrajectorySegment {
    double start;
    double end;
  }

  public CruisingPlugin(PluginServiceLocator psl) {
    super(psl);
  }

	@Override
	public void onInitialize() {
    routeStateSub = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
    routeStateSub.registerOnMessageCallback(new OnMessageCallback<RouteState>() {
  		@Override
  		public void onMessage(RouteState msg) {
  		}
    });

    routeSub = pubSubService.getSubscriberForTopic("route", Route._TYPE);
    routeSub.registerOnMessageCallback(new OnMessageCallback<Route>() {
		  @Override
	  	public void onMessage(Route msg) {
        currentRoute.set(msg);
        speedLimits = processSpeedLimits(msg);
	  	}
    });

    currentSegmentSub = pubSubService.getSubscriberForTopic("current_segment", RouteSegment._TYPE);
    currentSegmentSub.registerOnMessageCallback(new OnMessageCallback<RouteSegment>() {
		  @Override
	  	public void onMessage(RouteSegment msg) {
	  		currentSegment.set(msg);
	  	}
    });
	}

	@Override
	public void onResume() {
		
	}

	@Override
	public void loop() throws InterruptedException {
		
	}

	@Override
	public void onSuspend() {
		
	}

	@Override
	public void onTerminate() {
		
  }
  
  private double mphToMps(byte milesPerHour) {
    return milesPerHour * MPH_TO_MPS;
  }

  private List<SpeedLimit> processSpeedLimits(Route route) {
    List<SpeedLimit> limits = new ArrayList<>();

    // Walk the segment list getting the start points speed limits
    double dtdAccum = 0.0;
    for (RouteSegment seg : route.getSegments()) {
      SpeedLimit limit = new SpeedLimit();
      limit.location = dtdAccum;
      limit.speedLimit = mphToMps(seg.getPrevWaypoint().getSpeedLimit());
      limits.add(limit);
      dtdAccum += seg.getLength();
    }

    // If we processed at least one waypoint, get the last waypoint, the endpoint of the last segment
    if (!limits.isEmpty()) {
      List<RouteSegment> segments = route.getSegments();
      RouteSegment lastSeg = segments.get(segments.size());
      SpeedLimit limit = new SpeedLimit();
      limit.location = dtdAccum;
      limit.speedLimit = mphToMps(lastSeg.getWaypoint().getSpeedLimit());
    }

    return limits;
  }

  // TODO: Improve handling of first and last speed limits on boundaries
  private List<SpeedLimit> getSpeedLimits(double startDistance, double endDistance) {
    // Get all the speed limits spanned by [startDistance, endDistance)
    List<SpeedLimit> spanned = new ArrayList<>();
    for (SpeedLimit limit : speedLimits) {
      if (limit.location >= startDistance && limit.location < endDistance) {
        spanned.add(limit);
      }
    }

    return spanned;
  }

  private List<TrajectorySegment> findTrajectoryGaps(Trajectory traj) {
    List<IManeuver> longitudinalManeuvers = traj.getLongitudinalManeuvers();
    longitudinalManeuvers.sort(new Comparator<IManeuver>() {
  		@Override
  		public int compare(IManeuver o1, IManeuver o2) {
  			return Double.compare(o1.getStartLocation(), o2.getStartLocation());
  		}
    });

    List<TrajectorySegment> gaps = new ArrayList<>();
    if (!longitudinalManeuvers.isEmpty() && longitudinalManeuvers.get(0).getStartLocation() != traj.getStartLocation()) {
      TrajectorySegment seg = new TrajectorySegment();
      seg.start = traj.getStartLocation();
      seg.end = longitudinalManeuvers.get(0).getStartLocation();
      gaps.add(seg);
    }

    IManeuver prev = null;
    for (IManeuver maneuver : longitudinalManeuvers) {
      if (prev != null) {
        if (prev.getEndLocation() != maneuver.getStartLocation()) {
          TrajectorySegment seg = new TrajectorySegment();
          seg.start = prev.getEndLocation();
          seg.end = maneuver.getStartLocation();
          gaps.add(seg);
        }
      }
    }

    if (!longitudinalManeuvers.isEmpty() && prev.getEndLocation() != traj.getEndLocation()) {
      TrajectorySegment seg = new TrajectorySegment();
      seg.start = prev.getEndLocation();
      seg.end = traj.getEndLocation();
      gaps.add(seg);
    }

    return gaps;
  }

  private IManeuver generateManeuver(double startDist, double endDist, double startSpeed, double endSpeed) {
    return null; // TODO: Implement pending Maneuvers implementation
  }
  
  @Override
  public void planTrajectory() {
    double trajStartDist = 0.0;
    double trajEndDist = 20.0;
    Trajectory traj = new Trajectory(trajStartDist, trajEndDist);

    List<TrajectorySegment> gaps = findTrajectoryGaps(traj);
    
    for (TrajectorySegment gap : gaps) {
      List<SpeedLimit> limits = getSpeedLimits(gap.start, gap.end);

      double prevDist = gap.start;
      double prevSpeed = limits.get(0).speedLimit;
      for (SpeedLimit limit  : getSpeedLimits(gap.start, gap.end)) {
        traj.addManeuver(generateManeuver(prevDist, limit.location, prevSpeed, limit.speedLimit));
        prevDist = limit.location;
        prevSpeed = limit.speedLimit;
      }
    }
  }
}
