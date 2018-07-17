package gov.dot.fhwa.saxton.carma.signal_plugin.dvi.domain;


public class AjaxResponse {

    private boolean result;
    private String serverMessage;

    public AjaxResponse(boolean result, String serverMessage)   {
        this.result = result;
        this.serverMessage = serverMessage;
    }

    public boolean getResult()  { return result; }

    public String getServerMessage()   { return serverMessage; }
}
