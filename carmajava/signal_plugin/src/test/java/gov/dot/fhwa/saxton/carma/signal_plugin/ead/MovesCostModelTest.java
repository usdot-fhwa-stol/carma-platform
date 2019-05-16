/*
 * Copyright (C) 2018-2019 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;
import static org.mockito.ArgumentMatchers.*;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mockito;

import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.MovesFuelCostModel;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.Node;

/**
 * Runs unit tests for the MovesCostModel class
 */
public class MovesCostModelTest {

  ILoggerFactory mockFact = mock(ILoggerFactory.class, Mockito.withSettings().stubOnly());
  ILogger log = mock(ILogger.class, Mockito.withSettings().stubOnly());
  IGlidepathAppConfig mockConfig = mock(IGlidepathAppConfig.class, Mockito.withSettings().stubOnly());
  String csvFile = "../launch/params/BaseRateForPassengerTruck.csv";

  @Before
  public void setUp() throws Exception {
    when(mockFact.createLoggerForClass(any())).thenReturn(log);
    LoggerManager.setLoggerFactory(mockFact);
    GlidepathApplicationContext.getInstance().setAppConfigOverride(mockConfig);
    log.info("Setting up tests for MovesCostModel");
  }

  /**
   * Tests Vehicle Specific Power calculation
   * @throws Exception
   */
  @Test
  public void testVSP() throws Exception {
    MovesFuelCostModel costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0, // fuelNormalizationDenominator
      2.0,      // timeNormalizationDenominator
      60.0,     // heuristicWeight
      0.5,      // percentCostForTime
      11.176,   // maxVelocity
      1.5       // maxAccel
    );


    assertEquals(18.380596295383693, costModel.getVSP(1.0, 15.0), 0.0000000001);
  }

  /**
   * Tests OpMode calculation
   * @throws Exception
   */
  @Test
  public void testGetModeConditional() throws Exception {
    MovesFuelCostModel costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0, // fuelNormalizationDenominator
      2.0,      // timeNormalizationDenominator
      60.0,     // heuristicWeight
      0.5,      // percentCostForTime
      11.176,   // maxVelocity
      1.5       // maxAccel
    );



    assertEquals(0, costModel.getModeConditional(18.380596295383693, 15.0, -1.0)); // Only acceleration matters here

    assertEquals(1, costModel.getModeConditional(18.380596295383693, 0.0, 1.0)); // Only velocity matters here

    // Acceleration doesn't matter for remaining tests

    assertEquals(11, costModel.getModeConditional(-1.0, 8.0, 1.0));  

    assertEquals(12, costModel.getModeConditional(1.0, 8.0, 1.0));

    assertEquals(13, costModel.getModeConditional(4.0, 8.0, 1.0));

    assertEquals(14, costModel.getModeConditional(7.0, 8.0, 1.0));

    assertEquals(15, costModel.getModeConditional(10.0, 8.0, 1.0));

    assertEquals(16, costModel.getModeConditional(12.0, 8.0, 1.0));

    assertEquals(21, costModel.getModeConditional(-1.0, 15.0, 1.0));

    assertEquals(22, costModel.getModeConditional(1.0, 15.0, 1.0));

    assertEquals(23, costModel.getModeConditional(4.0, 15.0, 1.0));

    assertEquals(24, costModel.getModeConditional(7.0, 15.0, 1.0));

    assertEquals(25, costModel.getModeConditional(10.0, 15.0, 1.0));

    assertEquals(27, costModel.getModeConditional(16.0, 15.0, 1.0));

    assertEquals(28, costModel.getModeConditional(20.0, 15.0, 1.0));

    assertEquals(29, costModel.getModeConditional(26.0, 15.0, 1.0));

    assertEquals(30, costModel.getModeConditional(30.0, 15.0, 1.0));

    assertEquals(33, costModel.getModeConditional(1.0, 25.0, 1.0));

    assertEquals(35, costModel.getModeConditional(8.0, 25.0, 1.0));

    assertEquals(37, costModel.getModeConditional(14.0, 25.0, 1.0));

    assertEquals(38, costModel.getModeConditional(20.0, 25.0, 1.0));

    assertEquals(39, costModel.getModeConditional(26.0, 25.0, 1.0));

    assertEquals(40, costModel.getModeConditional(30.0, 25.0, 1.0));
  }

  /**
   * Tests Joules conversion
   * @throws Exception
   */
  @Test
  public void testToJPerSec() throws Exception {
    MovesFuelCostModel costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0, // fuelNormalizationDenominator
      2.0,      // timeNormalizationDenominator
      60.0,     // heuristicWeight
      0.5,      // percentCostForTime
      11.176,   // maxVelocity
      1.5       // maxAccel
    );

    assertEquals(19256.305555, costModel.toJPerSec(69322.7), 0.000001);
  }

  /**
   * Tests Joules calculation
   * 
   * NOTE: This test is only valid if testToJPerSec is passing
   * @throws Exception
   */
  @Test
  public void testGetJFromOpMode() throws Exception {
    MovesFuelCostModel costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0, // fuelNormalizationDenominator
      2.0,      // timeNormalizationDenominator
      60.0,     // heuristicWeight
      0.5,       // percentCostForTime
      11.176,   // maxVelocity
      1.5       // maxAccel
    );


    double dt = 1.0;
    assertEquals(costModel.toJPerSec(69322.7) * dt, costModel.getJFromOpMode(0, dt), 0.000001);

    assertEquals(costModel.toJPerSec(53814.0) * dt, costModel.getJFromOpMode(1, dt), 0.000001);

    assertEquals(costModel.toJPerSec(86160.6) * dt, costModel.getJFromOpMode(11, dt), 0.000001);

    assertEquals(costModel.toJPerSec(107935) * dt, costModel.getJFromOpMode(12, dt), 0.000001);

    assertEquals(costModel.toJPerSec(156645) * dt, costModel.getJFromOpMode(13, dt), 0.000001);

    assertEquals(costModel.toJPerSec(199614) * dt, costModel.getJFromOpMode(14, dt), 0.000001);

    assertEquals(costModel.toJPerSec(241463) * dt, costModel.getJFromOpMode(15, dt), 0.000001);

    assertEquals(costModel.toJPerSec(303195) * dt, costModel.getJFromOpMode(16, dt), 0.000001);

    assertEquals(costModel.toJPerSec(114416) * dt, costModel.getJFromOpMode(21, dt), 0.000001);

    assertEquals(costModel.toJPerSec(124211) * dt, costModel.getJFromOpMode(22, dt), 0.000001);

    assertEquals(costModel.toJPerSec(154749) * dt, costModel.getJFromOpMode(23, dt), 0.000001);

    assertEquals(costModel.toJPerSec(201045) * dt, costModel.getJFromOpMode(24, dt), 0.000001);

    assertEquals(costModel.toJPerSec(257884) * dt, costModel.getJFromOpMode(25, dt), 0.000001);

    assertEquals(costModel.toJPerSec(354948) * dt, costModel.getJFromOpMode(27, dt), 0.000001);

    assertEquals(costModel.toJPerSec(473680) * dt, costModel.getJFromOpMode(28, dt), 0.000001);

    assertEquals(costModel.toJPerSec(648656) * dt, costModel.getJFromOpMode(29, dt), 0.000001);

    assertEquals(costModel.toJPerSec(762692) * dt, costModel.getJFromOpMode(30, dt), 0.000001);

    assertEquals(costModel.toJPerSec(167527) * dt, costModel.getJFromOpMode(33, dt), 0.000001);

    assertEquals(costModel.toJPerSec(256632) * dt, costModel.getJFromOpMode(35, dt), 0.000001);

    assertEquals(costModel.toJPerSec(339845) * dt, costModel.getJFromOpMode(37, dt), 0.000001);

    assertEquals(costModel.toJPerSec(440687) * dt, costModel.getJFromOpMode(38, dt), 0.000001);

    assertEquals(costModel.toJPerSec(576356) * dt, costModel.getJFromOpMode(39, dt), 0.000001);

    assertEquals(costModel.toJPerSec(761853) * dt, costModel.getJFromOpMode(40, dt), 0.000001);
  }

  /**
   * Tests Joules calculation
   * 
   * NOTE: Test is only valid if getJFromOpMode test is passing
   * 
   * @throws Exception
   */
  @Test
  public void testCost() throws Exception {

    // Only time cost no normalization
    MovesFuelCostModel costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0, // fuelNormalizationDenominator
      1.0,      // timeNormalizationDenominator
      60.0,     // heuristicWeight
      1.0,      // percentCostForTime
      11.176,   // maxVelocity
      1.5       // maxAccel
    );
    // dist, time, speed
    assertEquals(2.0, costModel.cost(new Node(10, 10, 5), new Node(10, 12, 5)), 0.0000001);

    // Only time cost with normalization
    costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0, // fuelNormalizationDenominator
      2.0,      // timeNormalizationDenominator
      60.0,     // heuristicWeight
      1.0,      // percentCostForTime
      11.176,   // maxVelocity
      1.5       // maxAccel
    );
    // dist, time, speed
    assertEquals(1.0, costModel.cost(new Node(10, 10, 5), new Node(10, 12, 5)), 0.0000001);

    // Only fuel cost without normalization
    costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      1.0,      // fuelNormalizationDenominator
      2.0,      // timeNormalizationDenominator
      60.0,     // heuristicWeight
      0.0,      // percentCostForTime
      11.176,   // maxVelocity
      1.5       // maxAccel
    );
    // dist, time, speed
    assertEquals(costModel.getJFromOpMode(28, 10), costModel.cost(new Node(0, 0, 10), new Node(150, 10, 20)), 0.0000001);

    // Only fuel cost with normalization
    costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0,      // fuelNormalizationDenominator
      2.0,      // timeNormalizationDenominator
      60.0,     // heuristicWeight
      0.0,      // percentCostForTime
      11.176,   // maxVelocity
      1.5       // maxAccel
    );
    // dist, time, speed
    assertEquals(costModel.getJFromOpMode(28, 10) / 425000.0, costModel.cost(new Node(0, 0, 10), new Node(150, 10, 20)), 0.0000001);

    // Combined time and fuel cost
    costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0, // fuelNormalizationDenominator
      10.0,     // timeNormalizationDenominator
      60.0,     // heuristicWeight
      0.5,      // percentCostForTime
      11.176,   // maxVelocity
      1.5       // maxAccel
    );
    // dist, time, speed
    assertEquals((0.5 * costModel.getJFromOpMode(28, 10) / 425000.0) + 0.5, costModel.cost(new Node(0, 0, 10), new Node(150, 10, 20)), 0.0000001);
  }

  /**
   * Tests heuristic calculation
   * 
   * 
   * @throws Exception
   */
  @Test
  public void testHeuristic() throws Exception {

    // Only h value no weight nor normalization only fuel
    MovesFuelCostModel costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      1.0, // fuelNormalizationDenominator
      1.0,      // timeNormalizationDenominator
      1.0,      // heuristicWeight
      0.0,      // percentCostForTime
      10.0,     // maxVelocity
      1.5       // maxAccel
    );
    // dist, time, speed
    costModel.setGoal(new Node(10,10,10));
    costModel.setTolerances(new Node(0,0,0));
    assertEquals(Double.POSITIVE_INFINITY, costModel.heuristic(new Node(0,0,0)), 0.0000001);
    assertEquals(14948.333333333334, costModel.heuristic(new Node(0,0,10)), 0.0000001);
    assertEquals(14948.333333333334 * 0.5, costModel.heuristic(new Node(5,0,10)), 0.0000001);
    assertEquals(Double.POSITIVE_INFINITY, costModel.heuristic(new Node(10,0,0)), 0.0000001);
    assertEquals(0.0, costModel.heuristic(new Node(10,0,10)), 0.0000001);
    assertEquals(Double.POSITIVE_INFINITY, costModel.heuristic(new Node(11,0,0)), 0.0000001);


    // Piece-wise cost
    costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      1.0, // fuelNormalizationDenominator
      1.0,      // timeNormalizationDenominator
      1.0,      // heuristicWeight
      0.0,      // percentCostForTime
      15.0,     // maxVelocity
      1.5       // maxAccel
    );
    // dist, time, speed
    costModel.setGoal(new Node(90,11,15));
    costModel.setTolerances(new Node(0,0,0));
    assertEquals(14948.333333333334 * 11.0, costModel.heuristic(new Node(0,0,0)), 0.0000001);

    // Only h value no weight with normalization only fuel
    costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0, // fuelNormalizationDenominator
      1.0,      // timeNormalizationDenominator
      1.0,      // heuristicWeight
      0.0,      // percentCostForTime
      10.0,     // maxVelocity
      1.5       // maxAccel
    );
    // dist, time, speed
    costModel.setGoal(new Node(10,10,10));
    costModel.setTolerances(new Node(0,0,0));
    assertEquals(0.03517254901, costModel.heuristic(new Node(0,0,10)), 0.0000001);
    assertEquals(0.03517254901 * 0.5, costModel.heuristic(new Node(5,0,10)), 0.0000001);

    // Only h value no weight with both time and fuel
    costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0, // fuelNormalizationDenominator
      1.0,      // timeNormalizationDenominator
      1.0,      // heuristicWeight
      0.5,      // percentCostForTime
      10.0,     // maxVelocity
      1.5       // maxAccel
    );
    // dist, time, speed
    costModel.setGoal(new Node(10,10,10));
    costModel.setTolerances(new Node(0,0,0));
    assertEquals((0.03517254901 * 0.5) + 0.5, costModel.heuristic(new Node(0,0,10)), 0.0000001);
    assertEquals((0.03517254901 * 0.5 + 0.5) * 0.5, costModel.heuristic(new Node(5,0,10)), 0.0000001);

  
    // Weighted h value with both time and fuel
    costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0, // fuelNormalizationDenominator
      1.0,      // timeNormalizationDenominator
      2.0,      // heuristicWeight
      0.5,      // percentCostForTime
      10.0,     // maxVelocity
      1.5       // maxAccel
    );
    // dist, time, speed
    costModel.setGoal(new Node(10,10,10));
    costModel.setTolerances(new Node(0,0,0));
    assertEquals(2.0 * ((0.03517254901 * 0.5) + 0.5), costModel.heuristic(new Node(0,0,10)), 0.0000001);
    assertEquals(2.0 * ((0.03517254901 * 0.5 + 0.5) * 0.5), costModel.heuristic(new Node(5,0,10)), 0.0000001);
  }
  
  /**
   * Tests goal check
   * 
   * 
   * @throws Exception
   */
  @Test
  public void testIsGoal() throws Exception {

    MovesFuelCostModel costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0, // fuelNormalizationDenominator
      2.0,      // timeNormalizationDenominator
      60.0,     // heuristicWeight
      0.5,      // percentCostForTime
      11.176,   // maxVelocity
      1.5       // maxAccel
    );
    // dist, time, speed
    costModel.setGoal(new Node(10,10,10));
    costModel.setTolerances(new Node(1,1,1));
    assertEquals(false, costModel.isGoal(new Node(0,0,0)));
    assertEquals(true, costModel.isGoal(new Node(10,10,10)));
    assertEquals(true, costModel.isGoal(new Node(9,9,9)));
    assertEquals(true, costModel.isGoal(new Node(11,11,11)));
    assertEquals(true, costModel.isGoal(new Node(10,0,10)));
  }

  /**
   * Helper test to compute many different cost values under different conditions. 
   * 
   * 
   * @throws Exception
   */
  @Test
  public void computeManyCosts() throws Exception {
    MovesFuelCostModel costModel = new MovesFuelCostModel(
      0.22112,  // rollingTermA
      0.002838, // rotatingTermB
      0.000698, // dragTermC
      1.86686,  // vehicleMassInTons
      1.86686,  // fixedMassFactor
      csvFile, // baseRateTablePath
      425000.0, // fuelNormalizationDenominator
      2.0,      // timeNormalizationDenominator
      60.0,     // heuristicWeight
      0.0,      // percentCostForTime
      11.176,   // maxVelocity
      1.5       // maxAccel
    );

    double t = 2.0;
    for (double vi = 0; vi < 15; vi++) {
      for (double vf = 0; vf < 15; vf++) {
        double d = ((vf + vi) / 2.0 * t);
        Node n1 = new Node(0,0,vi);
        Node n2 = new Node(d,t,vf);
        double cost = costModel.cost(n1, n2);
        System.out.println("Cost: " + cost + " N1: " + n1 + " N2: " + n2);
      }
    }

  }
}
