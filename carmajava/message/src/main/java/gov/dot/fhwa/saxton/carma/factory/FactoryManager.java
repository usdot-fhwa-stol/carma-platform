package gov.dot.fhwa.saxton.carma.factory;

import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class FactoryManager {
    public static IMessageFactory<?> getMessageFactory(String messageType, ConnectedNode node, SaxtonLogger log, MessageFactory factory) {
        switch(messageType) {
        case "BSM":
            return new BSMFactory(node, log, factory);
        }
        return null;
    }
}
