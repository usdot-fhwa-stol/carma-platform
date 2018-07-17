package gov.dot.fhwa.saxton.glidepath.dvi.simulated.testutils;


import gov.dot.fhwa.saxton.glidepath.appcommon.DataElementHolder;
import static gov.dot.fhwa.saxton.glidepath.appcommon.DataElementKey.*;

import gov.dot.fhwa.saxton.glidepath.appcommon.XgvStatusDataElement;
import gov.dot.fhwa.saxton.glidepath.xgv.XgvStatus;
import gov.dot.fhwa.saxton.glidepath.xgv.messages.ReportDiscreteDevicesMessage.XgvGearState;

import java.util.Random;

public class XgvUtils {

    private static final Random RANDOM = new Random();

    private static int counter = 0;

    public static <T extends Enum<?>> T randomEnum(Class<T> clazz){
        int x = RANDOM.nextInt(clazz.getEnumConstants().length);
        return clazz.getEnumConstants()[x];
    }

    public static DataElementHolder createXgvStatus()   {
        DataElementHolder holder = new DataElementHolder();

        // *****************************************************************
        //
        // for the first 10 seconds, show activation key disabled and park
        // at 10 seconds, enable activiation key
        // at 15 seconds, show neutral.....GO button then displayed
        //
        // *****************************************************************
        XgvStatus xgvStatus = null;
        if (counter < 100)   {
            xgvStatus = new XgvStatus(false, true, false, XgvGearState.PARK);
        }
        else if (counter < 150)   {
            xgvStatus = new XgvStatus(true, false, false, XgvGearState.PARK);
        }
        // UNCOMMENT THIS TO SIMULATE STEPPING ON THE BRAKE
//        else if (counter == 263)   {
//            xgvStatus = new XgvStatus(true, true, false, XgvGearState.NEUTRAL);
//        }
        else   {
            xgvStatus = new XgvStatus(true, false, false, XgvGearState.NEUTRAL);
        }

        counter += 1;
        XgvStatusDataElement element = new XgvStatusDataElement(xgvStatus);
        holder.put(XGV_STATUS, element);

        return holder;
    }

}
