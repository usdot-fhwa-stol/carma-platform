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

package gov.dot.fhwa.saxton.carma.plugins.platooning;

import java.time.Duration;
import java.time.LocalDateTime;

/**
 * This class generate speed commands based on the latest information of platoon member list.
 */
public class CommandGenerator implements Runnable {
    
    protected PlatooningPlugin plugin;
    protected volatile double speedCmd = 0.0;
    protected volatile LocalDateTime speedCmdTimestamp = LocalDateTime.now();
    
    public CommandGenerator(PlatooningPlugin plugin) {
        this.plugin = plugin;
    }

    @Override
    public void run() {
        // TODO Update speed commands and its timestamp based on the list of platoon members 
    }
    
    public double getLastSpeedCommand() {
        return speedCmd;
    }
    
    // return the time duration from last speed command update
    public long getTimeSinceLastUpdate() {
        LocalDateTime now = LocalDateTime.now();
        long millis = Duration.between(speedCmdTimestamp, now).toMillis();
        return millis;
    }
}
