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

package gov.dot.fhwa.saxton.carma.guidance.plugins;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.ILoggerFactory;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import gov.dot.fhwa.saxton.utils.ComponentVersion;

public class PluginLifecycleHandlerTest {
    @Before public void setUp() throws Exception {
        ILoggerFactory mockFact = mock(ILoggerFactory.class);
        ILogger mockLogger = mock(ILogger.class);
        when(mockFact.createLoggerForClass(any())).thenReturn(mockLogger);
        LoggerManager.setLoggerFactory(mockFact);
        p = mock(IPlugin.class);
        doAnswer((in) -> {
            Thread.sleep(50);
            return null;
        }).when(p).loop();
        ComponentVersion cv = new ComponentVersion();
        when(p.getVersionInfo()).thenReturn(cv);
        handler = new PluginLifecycleHandler(p);
    }

    @After public void tearDown() throws Exception {
        if (running) {
            handler.terminate();
            Thread.sleep(100);
        }

        p = null;
        handler = null;
    }

    // Happy path tests
    @Test public void initialize() throws Exception {
        handler.initialize();
        Thread.sleep(100);
        verify(p).onInitialize();
    }

    @Test public void resume() throws Exception {
        handler.initialize();
        Thread.sleep(100);
        handler.resume();
        Thread.sleep(100);
        verify(p).onResume();
        running = true;
    }

    @Test public void suspend() throws Exception {
        handler.initialize();
        Thread.sleep(100);
        handler.resume();
        Thread.sleep(100);
        running = true;
        handler.suspend();
        running = false;
        Thread.sleep(100);
        verify(p).onSuspend();
    }

    @Test public void terminate() throws Exception {
        handler.initialize();
        Thread.sleep(100);
        handler.resume();
        running = true;
        Thread.sleep(100);
        handler.suspend();
        Thread.sleep(100);
        handler.terminate();
        Thread.sleep(100);
        verify(p).onTerminate();
        running = false;
    }

    // Verify the state machine errors in the correct places
    @Test public void testDoubleInitialize() throws Exception {
        handler.initialize();
        Thread.sleep(100);

        boolean threw = false;
        try {
            handler.initialize();
        } catch (IllegalStateException ise) {
            threw = true;
        }

        assertTrue(threw);
    }

    @Test public void testDoubleResume() throws Exception {
        handler.initialize();
        Thread.sleep(100);
        handler.resume();
        Thread.sleep(100);
        running = true;

        boolean threw = false;
        try {
            handler.resume();
        } catch (IllegalStateException ise) {
            threw = true;
        }

        assertTrue(threw);
    }

    @Test public void testDoubleSuspend() throws Exception {
        handler.initialize();
        Thread.sleep(100);
        handler.resume();
        Thread.sleep(100);
        running = true;
        handler.suspend();
        Thread.sleep(100);
        running = false;

        boolean threw = false;
        try {
            handler.suspend();
        } catch (IllegalStateException ise) {
            threw = true;
        }

        assertTrue(threw);
    }

    @Test public void testDoubleTerminate() throws Exception {
        handler.initialize();
        Thread.sleep(100);
        handler.resume();
        Thread.sleep(100);
        running = true;
        handler.suspend();
        Thread.sleep(100);
        running = false;
        handler.terminate();
        Thread.sleep(100);

        boolean threw = false;
        try {
            handler.terminate();
        } catch (IllegalStateException ise) {
            threw = true;
        }

        assertTrue(threw);
    }

    @Test public void testEarlyResume() throws Exception {
        boolean threw = false;
        try {
            handler.resume();
        } catch (IllegalStateException ise) {
            threw = true;
        }

        assertTrue(threw);
    }

    @Test public void testEarlySuspend() throws Exception {
        boolean threw = false;
        try {
            handler.suspend();
        } catch (IllegalStateException ise) {
            threw = true;
        }

        assertTrue(threw);
    }

    @Test public void testEarlyTerminate() throws Exception {
        boolean threw = false;
        try {
            handler.terminate();
        } catch (IllegalStateException ise) {
            threw = true;
        }

        assertTrue(threw);
    }

    private PluginLifecycleHandler handler;
    private IPlugin p;
    private boolean running;
}