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

import java.io.IOException;

import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.Constants;
import gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.IGlidepathAppConfig;
import gov.dot.fhwa.saxton.carma.signal_plugin.ead.trajectorytree.ICostModel;

/**
 * Factory-pattern class for ICostModel objects
 * <p>
 * Used by the Signal Plugin to determine which cost model is being used for the A-Star search
 */
public class DefaultCostModelFactory implements ICostModelFactory{

  private final IGlidepathAppConfig config;

  /**
   * Constructor
   * 
   * @param config A IGlidepathAppConfig config allowing parameters for requested cost models to be loaded
   */
  public DefaultCostModelFactory(IGlidepathAppConfig config) {
    this.config = config;
  }

  /**
   * Create a new instance of ICostModel
   * 
   * @param desiredModelName The name identifying the model to be used
   * @throws IllegalArgumentException Exception thrown when the specified model cannot be instantiated
   * 
   * @return An initialized ICostModel object
   */
  public ICostModel getCostModel(String desiredModelName) throws IllegalArgumentException {
    switch(desiredModelName) {
      case "DEFAULT":
        double vehicleMass = config.getDoubleValue("ead.vehicleMass");
        double rollingRes = config.getDoubleValue("ead.rollingResistanceOverride");
        double dragCoef = config.getDoubleValue("ead.dragCoefficient");
        double frontalArea = config.getDoubleValue("ead.frontalArea");
        double airDensity = config.getDoubleValue("ead.airDensity");
        double idlePower = config.getDoubleValue("ead.idleCost");
        boolean useIdleMin = config.getBooleanValue("ead.useIdleMin");

        return new FuelCostModel(vehicleMass, rollingRes, dragCoef, frontalArea, airDensity, idlePower, useIdleMin);
      case "MOVES_2010":
        double rollingTermA = config.getDoubleValue("ead.MOVES.rollingTermA");
        double rotatingTermB = config.getDoubleValue("ead.MOVES.rotatingTermB");
        double dragTermC = config.getDoubleValue("ead.MOVES.dragTermC");
        double vehicleMassInTons = config.getDoubleValue("ead.MOVES.vehicleMassInTons");
        double fixedMassFactor = config.getDoubleValue("ead.MOVES.fixedMassFactor");
        String baseRateTablePath = config.getProperty("ead.MOVES.baseRateTablePath");

        double fuelNormalizationDenominator = config.getDoubleValue("ead.MOVES.fuelNormalizationDenominator");
        double timeNormalizationDenominator = config.getDoubleValue("ead.MOVES.timeNormalizationDenominator");
        double heuristicWeight = config.getDoubleValue("ead.MOVES.heuristicWeight");
        double percentTimeCost = config.getDoubleValue("ead.MOVES.percentTimeCost");
        double maxSpeed = ((double)config.getMaximumSpeed(0.0)) / Constants.MPS_TO_MPH;

        try {
          return new MovesFuelCostModel(rollingTermA, rotatingTermB, dragTermC, vehicleMassInTons, fixedMassFactor, baseRateTablePath,
            fuelNormalizationDenominator, timeNormalizationDenominator, heuristicWeight, percentTimeCost, maxSpeed);
        } catch (IOException e) {
          throw new IllegalArgumentException(e.getMessage());
        }
        
      default:
        throw new IllegalArgumentException("Provided cost model name is not valid: " + desiredModelName);
    }
  }
}