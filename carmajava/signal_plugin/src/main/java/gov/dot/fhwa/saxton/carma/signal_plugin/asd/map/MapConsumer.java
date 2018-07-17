package gov.dot.fhwa.saxton.glidepath.asd.map;

import gov.dot.fhwa.saxton.glidepath.appcommon.utils.GlidepathApplicationContext;
import gov.dot.fhwa.saxton.glidepath.asd.AsdMessageType;
import gov.dot.fhwa.saxton.glidepath.asd.IAsdMessage;
import gov.dot.fhwa.saxton.glidepath.asd.UdpForwardConsumer;

import java.util.List;

/**
 * MAP Consumer
 */
public class MapConsumer extends UdpForwardConsumer {

    public MapConsumer()   {
        super();
        setPort(Integer.valueOf((GlidepathApplicationContext.getInstance().getAppConfig()).getProperty("asd.mapport")));
        setMsgType(AsdMessageType.MAP_MSG_ID);
    }

    @Override
    public List<IAsdMessage> processMessage(List<IAsdMessage> list, IAsdMessage message) {

        if (message != null  &&  message instanceof MapMessage) {
            MapMessage mapMessage = (MapMessage) message;
            list.add(mapMessage);
        }

        return list;
    }
}
