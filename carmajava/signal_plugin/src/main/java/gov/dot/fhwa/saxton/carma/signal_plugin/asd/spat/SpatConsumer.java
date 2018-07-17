package gov.dot.fhwa.saxton.carma.signal_plugin.asd.spat;

import gov.dot.fhwa.saxton.glidepath.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.glidepath.asd.AsdMessageType;
import gov.dot.fhwa.saxton.glidepath.asd.IAsdMessage;
import gov.dot.fhwa.saxton.glidepath.asd.UdpForwardConsumer;

import java.util.List;

/**
 * SPAT Consumer
 *
 * User: ferenced
 * Date: 1/18/15
 * Time: 11:18 PM
 *
 */
public class SpatConsumer extends UdpForwardConsumer {

    public SpatConsumer()   {
        super();
        setPort(Integer.valueOf((GlidepathApplicationContext.getInstance().getAppConfig()).getProperty("asd.spatport")));
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
