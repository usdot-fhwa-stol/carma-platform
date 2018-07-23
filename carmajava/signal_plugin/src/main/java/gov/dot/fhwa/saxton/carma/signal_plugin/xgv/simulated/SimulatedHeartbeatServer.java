package gov.dot.fhwa.saxton.carma.signal_plugin.xgv.simulated;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.messages.JausMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.xgv.XgvConnection;

import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;

/**
 * A simulated XGV Device that sends out a heartbeat pulse once per second.
 */
public class SimulatedHeartbeatServer implements Runnable {
    private int listenPort;
    private long lastPulse;
    private DatagramSocket sock;
    private XgvConnection connection;
    private InetSocketAddress outbound;
    private boolean running = false;
    private int targetJausId;
    private long delayMs = 1000;

    public SimulatedHeartbeatServer(int listenPort, InetSocketAddress outbound) {
        this.listenPort = listenPort;
        this.targetJausId = (GlidepathApplicationContext.getInstance().getAppConfig()).getSoftwareJausId();

        try {
            sock = new DatagramSocket(listenPort);
            connection = new XgvConnection(sock, outbound);
        } catch (SocketException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void run() {
        running = true;
        lastPulse = System.currentTimeMillis();

        // Broadcast a heartbeat pulse every second
        while (running) {

            // Send the message
            try {
                byte[] empty = new byte[0];
                JausMessage m = new JausMessage(false, targetJausId, empty, JausMessage.JausCommandCode.HEARTBEAT.getCode());
                connection.sendJausMessage(m);
            } catch (java.io.IOException e) {
                e.printStackTrace();
            }

            // Sleep until the next tick comes around
            long curTime = System.currentTimeMillis();
            long delay = delayMs - (curTime - lastPulse);
            lastPulse = curTime;
            try {
                Thread.sleep(delay);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void stop() {
        running = false;
    }
}
