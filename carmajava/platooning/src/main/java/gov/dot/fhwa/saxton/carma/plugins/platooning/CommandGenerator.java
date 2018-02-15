/*
 * Copyright (C) 2018 LEIDOS.
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

/**
 * This class generates speed commands based on the latest information from plugin platoon list.
 */
public class CommandGenerator implements Runnable, IPlatooningCommandInputs {
    
    protected PlatooningPlugin plugin_;
    protected long timestep_;
    protected volatile double speedCmd_ = 0.0;
    
    public CommandGenerator(PlatooningPlugin plugin) {
        this.plugin_ = plugin;
        //timestep_ = plugin.();
    }

    @Override
    public void run() {
        while(true) {
            long tsStart = System.currentTimeMillis();
                // TODO Update speed commands and its timestamp based on the list of platoon members
            long tsEnd = System.currentTimeMillis();
            long sleepDuration = Math.max(timestep_ - (tsEnd - tsStart), 0);
            try {
                Thread.sleep(sleepDuration);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
    
    @Override
    public double getLastSpeedCommand() {
        return speedCmd_;
    }
    
    @Override
    public double getMaxAccelLimit() {
        return plugin_.maxAccel;
    }
    
}
