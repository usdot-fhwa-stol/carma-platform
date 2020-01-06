/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.geometry;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.GreatCircleSegment;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.HaversineStrategy;
import gov.dot.fhwa.saxton.carma.geometry.geodesic.Location;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertTrue;

/**
 * Runs unit tests for the GeodesicCartesianConverter class
 */
public class HaversineStrategyTest {

  Log log;

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(HaversineStrategyTest.class);
    log.info("Setting up tests for HaversineStrategy");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the distanceLoc2Loc function
   * Accuracy checked against online calculator http://www.movable-type.co.uk/scripts/latlong.html
   * @throws Exception
   */
  @Test
  public void testDistanceLoc2Loc() throws Exception {

    log.info("// Entering distanceLoc2Loc test");
    HaversineStrategy haversineStrategy = new HaversineStrategy();

    // Test points on example garage to colonial farm rd route
    Location loc1 = new Location(38.95647,-77.15031, 0);
    Location loc2 = new Location(38.95631, -77.15041, 0);
    double solution = 19.78;
    assertTrue(Math.abs(haversineStrategy.distanceLoc2Loc(loc1, loc2) - solution) < 0.1); // Check accuracy to within .1m

    loc1 = new Location(38.95628,-77.15047, 0);
    loc2 = new Location(38.95613, -77.15101, 0);
    solution = 49.58;
    assertTrue(Math.abs(haversineStrategy.distanceLoc2Loc(loc1, loc2) - solution) < 0.1); // Check accuracy to within .1m

    // Test points about a km apart
    loc1 = new Location(38.942201,-77.160108, 0);
    loc2 = new Location(38.943804, -77.148832, 0);
    solution = 991.4;
    assertTrue(Math.abs(haversineStrategy.distanceLoc2Loc(loc1, loc2) - solution) < 0.1); // Check accuracy to within .1m
  }

  /**
   * Tests the crossTrackDistance function
   * Accuracy checked against online calculator http://www.movable-type.co.uk/scripts/latlong.html
   * @throws Exception
   */
  @Test
  public void testCrossTrackDistance() throws Exception {

    log.info("// Entering crossTrackDistance test");
    HaversineStrategy haversineStrategy = new HaversineStrategy();

    // Test on km long segment (Right side)
    Location loc1 = new Location(38.942201,-77.160108, 0);
    Location loc2 = new Location(38.943804, -77.148832, 0);
    GreatCircleSegment seg = new GreatCircleSegment(loc1, loc2);
    Location sideLoc = new Location(38.942422, -77.154786, 0);
    double solution = 58.59;
    assertTrue(Math.abs(haversineStrategy.crossTrackDistance(sideLoc, seg) - solution) < 0.1); // Check accuracy to within .1m

    // Test on km long segment (Left side)
    sideLoc = new Location(38.94348, -77.15505, 0);
    solution = -61.14;
    assertTrue(Math.abs(haversineStrategy.crossTrackDistance(sideLoc, seg) - solution) < 0.1); // Check accuracy to within .1m


    // Test point on segment start
    loc1 = new Location(38.942201,-77.160108, 0);
    loc2 = new Location(38.943804, -77.148832, 0);
    seg = new GreatCircleSegment(loc1, loc2);
    solution = 0.0;
    assertTrue(Math.abs(haversineStrategy.crossTrackDistance(loc1, seg) - solution) < 0.1); // Check accuracy to within .1m

    // Test point on segment end
    loc1 = new Location(38.942201,-77.160108, 0);
    loc2 = new Location(38.943804, -77.148832, 0);
    seg = new GreatCircleSegment(loc1, loc2);
    solution = 0.0;
    assertTrue(Math.abs(haversineStrategy.crossTrackDistance(loc2, seg) - solution) < 0.1); // Check accuracy to within .1m
  }

  /**
   * Tests the downtrackDistance function
   * Accuracy checked against online calculator http://www.movable-type.co.uk/scripts/latlong.html
   * @throws Exception
   */
  @Test
  public void testDownTrackDistance() throws Exception {

    log.info("// Entering downtrackDistance test");
    HaversineStrategy haversineStrategy = new HaversineStrategy();

    // Test on km long segment
    Location loc1 = new Location(38.942201,-77.160108, 0);
    Location loc2 = new Location(38.943804, -77.148832, 0);
    GreatCircleSegment seg = new GreatCircleSegment(loc1, loc2);
    Location sideLoc = new Location(38.942422, -77.154786, 0);
    double solution = 458.3;
    assertTrue(Math.abs(haversineStrategy.downtrackDistance(sideLoc, seg) - solution) < 0.1); // Check accuracy to within .1m

    // Test point on segment start
    loc1 = new Location(38.942201,-77.160108, 0);
    loc2 = new Location(38.943804, -77.148832, 0);
    seg = new GreatCircleSegment(loc1, loc2);
    solution = 0.0;
    assertTrue(Math.abs(haversineStrategy.downtrackDistance(loc1, seg) - solution) < 0.1); // Check accuracy to within .1m

    // Test point on segment end
    loc1 = new Location(38.942201,-77.160108, 0);
    loc2 = new Location(38.943804, -77.148832, 0);
    seg = new GreatCircleSegment(loc1, loc2);
    solution = 991.4;
    assertTrue(Math.abs(haversineStrategy.downtrackDistance(loc2, seg) - solution) < solution * 0.005); // Check accuracy to within .5% of haversine result
  }

  /**
   * Tests the projectOntoSegment function
   * Accuracy checked against online calculator http://www.movable-type.co.uk/scripts/latlong.html
   * @throws Exception
   */
  @Test
  public void testProjectOntoSegment() throws Exception {

    log.info("// Entering projectOntoSegment test");
    HaversineStrategy haversineStrategy = new HaversineStrategy();

    // Test point on segment start
    Location loc1 = new Location(38.95605, -77.15073, 0);
    Location loc2 = new Location(38.95602, -77.15073, 0);
    GreatCircleSegment seg = new GreatCircleSegment(loc1, loc2);
    Location solution = loc1;
    Location result = haversineStrategy.projectOntoSegment(loc1, seg);
    assertTrue(result.almostEqual(solution, 0.0001, 1.0));

    // Test point on segment end
    seg = new GreatCircleSegment(loc1, loc2);
    solution = loc2;
    result = haversineStrategy.projectOntoSegment(loc2, seg);
    assertTrue(result.almostEqual(solution, 0.0001, 1.0));
    
    // Test point on before segment start
    Location externalPoint = new Location(38.95607, -77.15072, 0);
    solution = new Location(38.95607, -77.15073, 0);
    result = haversineStrategy.projectOntoSegment(externalPoint, seg);
    assertTrue(result.almostEqual(solution, 0.0001, 1.0));  

    // Test point in middle of segment
    externalPoint = new Location(38.95604, -77.15072, 0);
    solution = new Location(38.95604, -77.15074, 0);
    result = haversineStrategy.projectOntoSegment(externalPoint, seg);
    assertTrue(result.almostEqual(solution, 0.0001, 1.0));
    
    // Test point past end of segment
    externalPoint = new Location(38.95601, -77.15075, 0);
    solution = new Location(38.95601, -77.15073, 0);
    result = haversineStrategy.projectOntoSegment(externalPoint, seg);
    assertTrue(result.almostEqual(solution, 0.0001, 1.0)); 
  } 
}
