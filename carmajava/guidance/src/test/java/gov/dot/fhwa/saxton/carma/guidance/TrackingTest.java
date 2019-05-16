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

package gov.dot.fhwa.saxton.carma.guidance;

import static org.junit.Assert.assertEquals;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import org.junit.Before;
import org.junit.Test;
import org.ros.node.ConnectedNode;

import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

public class TrackingTest {
    
    private GuidanceStateMachine mockGuidanceStateMachine = mock(GuidanceStateMachine.class);
    private IPubSubService mockPubSubService = mock(IPubSubService.class);
    private ConnectedNode mockNode = mock(ConnectedNode.class);
    private Tracking tracking;
    
    @Before
    public void setup() {
        ILoggerFactory mockFact = mock(ILoggerFactory.class);
        ILogger mockLogger = mock(ILogger.class);
        when(mockFact.createLoggerForClass(any())).thenReturn(mockLogger);
        LoggerManager.setLoggerFactory(mockFact);
        tracking = new Tracking(mockGuidanceStateMachine, mockPubSubService, mockNode);
    }
    
    @Test
    public void getBsmId() {
        // create a array of {00000000, 00010001, 00011111, 11111111}
        tracking.random_id = new byte[] {0, 17, 31, -1};
        String bsmId = tracking.getCurrentBSMId();
        assertEquals("00111fff", bsmId);
    }
}
