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

package gov.dot.fhwa.saxton.carma.guidance.plugins;

import cav_msgs.SystemAlert;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPublisher;
import gov.dot.fhwa.saxton.carma.guidance.trajectory.Trajectory;

/**
 * Base class for Mock plugin implementations
 * <p>
 * Handles logging and message publication for
 */
public abstract class AbstractMockPlugin extends AbstractPlugin {
    protected final String topicPrefix = "system_alert";
    protected final long sleepDuration = 5000;
    protected IPublisher<SystemAlert> publisher;

    public AbstractMockPlugin(PluginServiceLocator pluginServiceLocator) {
        super(pluginServiceLocator);
    }

    @Override public void onInitialize() {
        log.info("Plugin " + getName() + ":" + getVersionId() + "initializing...");
        publisher = pubSubService.getPublisherForTopic(topicPrefix, SystemAlert._TYPE);
        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " initialized");
        publisher.publish(msg);
    }

    @Override public void onResume() {
        log.info("Plugin " + getName() + ":" + getVersionId() + " resuming...");
        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " resumed");
        publisher.publish(msg);
    }

    @Override public void loop() throws InterruptedException {
        log.info("Plugin " + getName() + ":" + getVersionId() + " iterating...");

        if (getActivation()) {
            computeAvailability();

            SystemAlert msg = publisher.newMessage();
            msg.setType(SystemAlert.CAUTION);
            msg.setDescription(
                "Plugin " + getName() + ":" + getVersionId() + " looping. Avail=" + getAvailability());
            publisher.publish(msg);
        }

        Thread.sleep(sleepDuration);
    }

    @Override public void onSuspend() {
        log.info("Plugin " + getName() + ":" + getVersionId() + " suspending...");

        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " suspended");
        publisher.publish(msg);
    }

    @Override public void onTerminate() {
        log.info("Plugin " + getName() + ":" + getVersionId() + " terminating...");

        SystemAlert msg = publisher.newMessage();
        msg.setType(SystemAlert.CAUTION);
        msg.setDescription("Plugin " + getName() + ":" + getVersionId() + " terminated");
        publisher.publish(msg);
    }

    private void computeAvailability() {
        boolean availability = Math.random() > 0.5;
        setAvailability(availability);
    }

    @Override public void planTrajectory(Trajectory traj, double expectedEntrySpeed) {
        // NO-OP
    }
}
