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

/**
 * Creates a concrete IEad class based on provided class name.
 */
public class EadFactory {
	
	public static IEad newInstance(String className) throws InstantiationException, IllegalAccessException {
		@SuppressWarnings("rawtypes")
		Class tClass;
		
        try   {
            tClass = Class.forName(className);
        }
        catch(ClassNotFoundException cnfe)   {
            tClass = EadAStar.class;
        }

        Object newObject = null;
        try   {
            newObject = tClass.newInstance();
        }
        catch (InstantiationException e) {
            throw e;
        } catch (IllegalAccessException e) {
            throw e;
        }

        return (IEad)newObject;
	}
	
}
