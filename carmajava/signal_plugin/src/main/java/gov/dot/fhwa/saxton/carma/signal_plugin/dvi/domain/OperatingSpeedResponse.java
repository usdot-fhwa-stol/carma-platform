package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.domain;


import gov.dot.fhwa.saxton.carma.signal_plugin.dvi.domain.AjaxResponse;

public class OperatingSpeedResponse extends AjaxResponse {

    String operatingSpeed;

    public OperatingSpeedResponse(boolean result, String serverMessage, String operatingSpeed)   {
        super(result, serverMessage);
        this.operatingSpeed = operatingSpeed;
    }

    public String getOperatingSpeed()   {
        return operatingSpeed;
    }
}
