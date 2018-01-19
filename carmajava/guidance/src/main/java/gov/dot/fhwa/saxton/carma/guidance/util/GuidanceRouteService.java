/*
 * Copyright (C) 2017 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.guidance.util;

import cav_msgs.Route;
import cav_msgs.RouteSegment;
import cav_msgs.RouteState;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.ISubscriber;

import java.util.SortedSet;
import java.util.TreeSet;

/**
 * Concrete implementation of RouteService responsible for handling processing of
 * Route data for consumption by plugins and other Guidance classes/components.
 */
public class GuidanceRouteService implements RouteService {
  protected IPubSubService pubSubService;
  protected ISubscriber<Route> routeSubscriber;
  protected Route currentRoute;
  protected RouteSegment currentSegment;
  protected volatile double downtrackDistance;
  protected ISubscriber<RouteState> routeStateSubscriber;
  protected SortedSet<SpeedLimit> limits;
  protected SortedSet<AlgorithmFlags> disabledAlgorithms;
  protected SortedSet<RequiredLane> requiredLanes;

  public GuidanceRouteService(IPubSubService pubSubService) {
    this.pubSubService = pubSubService;
  }

  public void init() {
    limits = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));
    disabledAlgorithms = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));

    routeSubscriber = pubSubService.getSubscriberForTopic("route", Route._TYPE);
    routeSubscriber.registerOnMessageCallback(this::processRoute);

    routeStateSubscriber = pubSubService.getSubscriberForTopic("route_state", RouteState._TYPE);
    routeStateSubscriber.registerOnMessageCallback((state) -> {
      if (currentRoute != null) {
        downtrackDistance = state.getDownTrack();
        currentSegment = getRouteSegmentAtLocation(downtrackDistance);
      }
    });

  }

  private double convertMphToMps(double mph) {
    return mph * 0.44704;
  }

  /**
   * Process the new route data to extract the limits and algorithm data
   */
  private void processRoute(Route newRoute) {
    currentRoute = newRoute;

    limits = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));
    disabledAlgorithms = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));
    requiredLanes = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));

    double dtdAccum = 0;
    for (RouteSegment seg : currentRoute.getSegments()) {
      dtdAccum += seg.getLength();

      SpeedLimit segmentLimit = new SpeedLimit(dtdAccum, convertMphToMps(seg.getWaypoint().getSpeedLimit()));
      limits.add(segmentLimit);

      AlgorithmFlags segmentFlags = new AlgorithmFlags(dtdAccum, seg.getWaypoint().getDisabledGuidanceAlgorithms());
      disabledAlgorithms.add(segmentFlags);

      RequiredLane requiredLane = new RequiredLane(dtdAccum, seg.getWaypoint().getRequiredLaneIndex());
      requiredLanes.add(requiredLane);
    }

    // Remove duplicates to find out where lane changes must occur
    SortedSet<RequiredLane> requiredLaneChanges = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));
    RequiredLane prev = null;
    for (RequiredLane lane : requiredLanes) {
      if (prev != null && prev.getLaneId() != lane.getLaneId()) {
        requiredLanes.add(lane);
      }

      prev = lane;
    }
  }

  @Override
  public RouteSegment getRouteSegmentAtLocation(double location) {
    double dtdAccum = 0;
    RouteSegment out = null;
    for (RouteSegment segment : currentRoute.getSegments()) {
      dtdAccum += segment.getLength();

      if (dtdAccum > location) {
        out = segment;
        break;
      }
    }

    return out;
  }

  @Override
  public Route getCurrentRoute() {
    return currentRoute;
  }

  @Override
  public RouteSegment getCurrentRouteSegment() {
    return currentSegment;
  }

  @Override
  public double getCurrentDowntrackDistance() {
    return downtrackDistance;
  }

  @Override
  public SortedSet<SpeedLimit> getSpeedLimitsOnRoute() {
    return limits;
  }

  @Override
  public SortedSet<AlgorithmFlags> getAlgorithmFlagsOnRoute() {
    return disabledAlgorithms;
  }

  @Override
  public SpeedLimit getSpeedLimitAtLocation(double location) {
    for (SpeedLimit limit : limits) {
      if (limit.getLocation() >= location) {
        return limit;
      }
    }

    return null;
  }

  @Override
  public AlgorithmFlags getAlgorithmFlagsAtLocation(double location) {
    for (AlgorithmFlags flags : disabledAlgorithms) {
      if (flags.getLocation() >= location) {
        return flags;
      }
    }

    return null;
  }

  @Override
  public SortedSet<SpeedLimit> getSpeedLimitsInRange(double start, double end) {
    SortedSet<SpeedLimit> out = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));
    for (SpeedLimit limit : limits) {
      if (limit.getLocation() > start && limit.getLocation() <= end) {
        out.add(limit);
      }

      if (limit.getLocation() > end) {
        break;
      }
    }

    return out;
  }

  @Override
  public SortedSet<AlgorithmFlags> getAlgorithmFlagsInRange(double start, double end) {
    SortedSet<AlgorithmFlags> out = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));
    for (AlgorithmFlags flags : disabledAlgorithms) {
      if (flags.getLocation() > start && flags.getLocation() <= end) {
        out.add(flags);
      }

      if (flags.getLocation() > end) {
        break;
      }
    }

    return out;
  }

  @Override
  public double[] getAlgorithmEnabledWindowInRange(double start, double end, String algorithm) {
    SortedSet<AlgorithmFlags> flags = this.getAlgorithmFlagsInRangeIncludingEnd(start, end);
    // Find the start of earliest window
    double earliestLegalWindow = start;
    for (AlgorithmFlags flagset : flags) {
      if (flagset.getDisabledAlgorithms().contains(algorithm)) {
        earliestLegalWindow = flagset.getLocation();
      } else {
        break;
      }
    }
    // Find the end of that same window
    double endOfWindow = earliestLegalWindow;
    for (AlgorithmFlags flagset : flags) {
      if (flagset.getLocation() > earliestLegalWindow) {
        if (!flagset.getDisabledAlgorithms().contains(algorithm)) {
          endOfWindow = flagset.getLocation();
        } else {
          break;
        }
      }
    }
    endOfWindow = Math.min(endOfWindow, end);
    if (endOfWindow > earliestLegalWindow) {
      return new double[] { earliestLegalWindow, endOfWindow };
    }
    return null;
  }

  @Override
  public boolean isAlgorithmEnabledInRange(double start, double end, String algorithm) {
    SortedSet<AlgorithmFlags> flags = this.getAlgorithmFlagsInRangeIncludingEnd(start, end);
    for (AlgorithmFlags flag : flags) {
      if (!flag.getDisabledAlgorithms().contains(algorithm)) {
        return true;
      }
    }
    return false;
  }

  private SortedSet<AlgorithmFlags> getAlgorithmFlagsInRangeIncludingEnd(double start, double end) {
    SortedSet<AlgorithmFlags> flags = this.getAlgorithmFlagsInRange(start, end);
    AlgorithmFlags flagsAtEnd = this.getAlgorithmFlagsAtLocation(end);
    if (flagsAtEnd != null) {
      flags.add(flagsAtEnd);
    }
    return flags;
  }

  @Override
  public SortedSet<RequiredLane> getRequiredLanesOnRoute() {
    return requiredLanes;
  }

  @Override
  public RequiredLane getRequiredLaneAtLocation(double location) {
    for (RequiredLane lane : requiredLanes) {
      if (lane.getLocation() >= location) {
        return lane;
      }
    }

    return null;
  }

  @Override
  public SortedSet<RequiredLane> getRequiredLanesInRange(double start, double end) {
    SortedSet<RequiredLane> out = new TreeSet<>((a, b) -> Double.compare(a.getLocation(), b.getLocation()));
    for (RequiredLane lane : requiredLanes) {
      if (lane.getLocation() > start && lane.getLocation() <= end) {
        out.add(lane);
      }

      if (lane.getLocation() > end) {
        break;
      }
    }

    return out;
  }
}
