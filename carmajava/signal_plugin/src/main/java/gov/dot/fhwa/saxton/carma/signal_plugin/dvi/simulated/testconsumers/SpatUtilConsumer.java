package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.carma.signal_plugin.asd.AsdMessageType;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.IAsdMessage;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.UdpForwardConsumerConfigured;
import gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat.SpatMessage;

import java.util.List;

public class SpatUtilConsumer extends UdpForwardConsumerConfigured {

    public SpatUtilConsumer()   {
        super();
        setPort(7788);
        setMsgType(AsdMessageType.SPAT_MSG_ID);
    }

    @Override
    public List<IAsdMessage> processMessage(List<IAsdMessage> list, IAsdMessage message) {

        if (message != null  &&  message instanceof SpatMessage)   {
            SpatMessage spatMessage = (SpatMessage) message;
            list.add(spatMessage);
            //for debugging: spatMessage.dumpSpatMessage();
        }

        return list;
    }
}
