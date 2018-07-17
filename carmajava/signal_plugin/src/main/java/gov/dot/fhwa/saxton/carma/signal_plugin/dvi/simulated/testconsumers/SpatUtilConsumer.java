package gov.dot.fhwa.saxton.glidepath.dvi.simulated.testconsumers;

import gov.dot.fhwa.saxton.glidepath.asd.AsdMessageType;
import gov.dot.fhwa.saxton.glidepath.asd.IAsdMessage;
import gov.dot.fhwa.saxton.glidepath.asd.UdpForwardConsumerConfigured;
import gov.dot.fhwa.saxton.glidepath.asd.spat.SpatMessage;

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
