/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.message.factory;

import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class DSRCMessageFactory {
    public static IMessage<?> getMessage(String messageType, ConnectedNode node, SaxtonLogger log, MessageFactory factory) {
        switch(messageType) {
        case "BSM":
            return new BSMMessage(node, log, factory);
        case "MobilityRequest":
            return new MobilityRequestMessage(log, factory);
        case "MobilityPath":
            return new MobilityPathMessage(log, factory);
        case "MobilityResponse":
            return new MobilityResponseMessage(log, factory);
        case "MobilityOperation":
            return new MobilityOperationMessage(factory, log);
        case "MAP":
            return new MapMessage(factory, log);
        case "SPAT":
            return new SPATMessage(factory, log);
        default:
            return null;
        }
    }
}
