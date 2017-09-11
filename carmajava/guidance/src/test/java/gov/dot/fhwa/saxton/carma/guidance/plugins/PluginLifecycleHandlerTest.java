package gov.dot.fhwa.saxton.carma.guidance.plugins;

import org.apache.commons.logging.Log;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.listeners.InvocationListener;
import org.mockito.listeners.MethodInvocationReport;
import org.mockito.stubbing.Answer;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class PluginLifecycleHandlerTest {
    @Before public void setUp() throws Exception {
        p = mock(IPlugin.class);
        log = mock(Log.class);
        handler = new PluginLifecycleHandler(p, log);
    }

    @After public void tearDown() throws Exception {
        if (running) {
            handler.terminate();
            Thread.sleep(100);
        }

        p = null;
        handler = null;
        log = null;
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
    private Log log;
    private boolean running;
}