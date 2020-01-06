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

package gov.dot.fhwa.saxton.carma.rosutils;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Runs unit tests for the MobilityHelper class
 */
public class MobilityHelperTest {

  Log log;
  NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  @Before
  public void setUp() throws Exception {
    log = LogFactory.getLog(MobilityHelper.class);
    log.info("Setting up tests for MobilityHelper");
  }

  @After
  public void tearDown() throws Exception {
  }

  /**
   * Tests the movement of a host vehicle along a route
   * @throws Exception
   */
  @Test
  public void testExtractStrategyParams() throws Exception {
    String strategyParams = "INFO|RADIUS:5.0,MERGE_DIST:-3.5,MERGE_LENGTH:20.2";
    String INFO_PARAM_TYPE = "INFO";
    List<String> REQUEST_PARAMS_KEYS = new ArrayList<>(Arrays.asList("RADIUS", "MERGE_DIST", "MERGE_LENGTH"));

    List<String> params = new ArrayList<>();
    try {
      params = MobilityHelper.extractStrategyParams(strategyParams, INFO_PARAM_TYPE, REQUEST_PARAMS_KEYS);
    } catch (IllegalArgumentException e) {
      fail();
    }

    assertEquals(5.0, Double.parseDouble(params.get(0)), 0.00000000001);
    assertEquals(-3.5, Double.parseDouble(params.get(1)), 0.00000000001);
    assertEquals(20.2, Double.parseDouble(params.get(2)), 0.00000000001);
  }
}
