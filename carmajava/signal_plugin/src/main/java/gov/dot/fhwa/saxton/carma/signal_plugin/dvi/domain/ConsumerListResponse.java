package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.domain;

import gov.dot.fhwa.saxton.glidepath.dvi.domain.AjaxResponse;

import java.util.List;

/**
 * Prove an ajax response object with a list of IConsumerTask class names
 *
 * Used with consumers.html to provide test capability for individual Consumers
 */
public class ConsumerListResponse  extends AjaxResponse {
    List<String> consumers;

    public ConsumerListResponse(boolean result, String serverMessage, List<String> consumers)   {
        super(result, serverMessage);
        this.consumers = consumers;
    }

    public List<String> getConsumers()   {
        return consumers;
    }

}
