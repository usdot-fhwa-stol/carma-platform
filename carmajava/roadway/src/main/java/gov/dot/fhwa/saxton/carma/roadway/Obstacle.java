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

package gov.dot.fhwa.saxton.carma.roadway;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import java.util.List;
import org.ros.rosjava_geometry.Transform;

/**
 * A basic obstacle in the road.
 * Obstacles are described in a parametric frame relative to the route
 */
public class Obstacle {

  protected int id;
  protected double downtrackDistance;
  protected double crosstrackDistance;
  protected Vector3D velocity;
  protected Vector3D acceleration;
  protected Vector3D size;
  protected Integer primaryLane;
  protected List<Integer> secondaryLanes;

  /**
   * Constructor
   */
  public Obstacle(int id, double downtrackDistance, double crosstrackDistance, Vector3D velocity,
   Vector3D acceleration, Vector3D size, Integer primaryLane) {
    
    this.id = id;
    this.downtrackDistance = downtrackDistance;
    this.crosstrackDistance = crosstrackDistance;
    this.velocity = velocity;
    this.acceleration = acceleration;
    this.size = size;
    this.primaryLane = primaryLane;
  }

  /**
   * Get the unique id of this obstacle
   */
  public int getId() {
    return id;
  }

  /**
   * Get the downtrack distance in m of this obstacle along the route.
   * @return the downtrack distance
   */
  public double getDowntrackDistance() {
    return downtrackDistance;
  }

  /**
   * Get the crosstrack distance in m of this obstacle along the route.
   * @return the crosstrack distance
   */
  public double getCrosstrackDistance() {
    return crosstrackDistance;
  }

  /**
   * TODO THINK ABOUT THIS
   * Get the velocity of the obstacle in m/s
   * Velocity reported in a FRD coordinate frame located at the 
   * X-axis is tangential along route
   * Y-axis is perpendicular to route forming a right handed coordinate frame.
   * Z-axis faces into earth and is not reported
   * @return the velocity
   */
  public Vector3D getVelocity() {
    return velocity;
  }

  /**
   * Get the acceleration of the obstacle in m/s^2
   * TODO THINK ABOUT THIS
   * @return acceleration
   */
  public Vector3D getAcceleration() {
    return acceleration;
  }

  /**
   * Gets the dimensions of the obstacle reported as an axis aligned bounding box.
   * Axis are aligned with the route segment as FRD frame.
   * The FRD frame is centered on the obstacle
   * Demensions therefore are reported as half there actual value.
   * For a vehicle 1 m in +x means the vehicle is 2m long from grill to bumper
   * @return the size vector
   */
  public Vector3D getSize() {
    return size;
  }

  /**
   * Gets the primary lane index of this obstacle
   * @return the primary lane index
   */
  public Integer getPrimaryLane() {
    return primaryLane;
  }

  /**
   * Sets the list of secondary lanes which this obstacle can be considered to intersect
   * @param secondaryLanes The list of lanes
   */
  public void setSecondaryLanes(List<Integer> secondaryLanes) {
    this.secondaryLanes = secondaryLanes;
  }

  /**
   * Gets a list of additionaly lanes which this obstacle can be considered to intersect
   * @return a list of lane indices
   */
  public List<Integer> getSecondaryLanes() {
    return secondaryLanes;
  }

  // /**
  //  * Gets the id of the frame relative to which this obstacle is defined
  //  * @return frame id
  //  */
  // String getReferenceFrameId();

  // /**
  //  * Gets the pose of this obstacle represented as a transform from the reference frame to this obstacles body frame
  //  * @return transform from reference frame to body frame
  //  */
  // Transform getPose();
}
