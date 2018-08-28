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
package gov.dot.fhwa.saxton.carma.signal_plugin;
import java.io.IOException;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.IMotionPredictor;
/**
* Factory-pattern class for IMotionPredictor objects
* <p>
* Used to determine how an NCVs motion will be predicted in the Traffic Signal Plugin
*/
public class DefaultMotionPredictorFactory implements IMotionPredictorModelFactory{
  private final IGlidepathAppConfig config;
  /**
  * Constructor
  * 
  * @param config A IGlidepathAppConfig config allowing parameters for requested prediction models to be loaded
  */
 public DefaultMotionPredictorFactory(IGlidepathAppConfig config) {
   this.config = config;
 }
  /**
  * Create a new instance of IMotionPredictor
  * 
  * @param desiredPredictorName The name identifying the model to be used
  * @throws IllegalArgumentException Exception thrown when the specified model cannot be instantiated
  * 
  * @return An initialized IMotionPredictor object
  */
  @Override
 public IMotionPredictor getMotionPredictor(String desiredPredictorName) throws IllegalArgumentException {
   switch(desiredPredictorName) {
     case "SIMPLE_LINEAR_REGRESSION":
      //return new 
        return null; //TODO
     case "UCR_NEURAL_NET": // TODO implement
      throw new IllegalArgumentException("UCR_NEURAL_NET has not yet been implemented ");
       
     default:
       throw new IllegalArgumentException("Provided motion predictor model name is not valid: " + desiredPredictorName);
   }
 }
}