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

package gov.dot.fhwa.saxton.carma.signal_plugin.ead;

/**
 * Factory method to instantiate an appropriate ITrajectory concrete class
 */
public class TrajectoryFactory {

    /**
     * Reflection helper to create a ITrajectory object based on class name
     *
     * @param trajectoryClassName
     * @return
     */
    public static ITrajectory newInstance(String trajectoryClassName)   {
        @SuppressWarnings("rawtypes")
		Class tClass = null;

        try   {
            tClass = Class.forName(trajectoryClassName);
        }
        catch(ClassNotFoundException cnfe)   {
            //tClass = SimulatedTrajectory.class; TODO deal with this
        }

        Object newObject = null;
        try   {
            newObject = tClass.newInstance();
        }
        catch (InstantiationException e) {
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        return (ITrajectory) newObject;
    }

}
