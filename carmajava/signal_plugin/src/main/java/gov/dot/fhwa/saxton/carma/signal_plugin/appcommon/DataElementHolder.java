package gov.dot.fhwa.saxton.glidepath.appcommon;

import gov.dot.fhwa.saxton.glidepath.logger.ILogger;
import gov.dot.fhwa.saxton.glidepath.logger.LoggerManager;
import gov.dot.fhwa.saxton.glidepath.xgv.XgvStatus;

import java.util.*;

import static gov.dot.fhwa.saxton.glidepath.appcommon.DataElementKey.*;

public class DataElementHolder {

    private static ILogger logger = LoggerManager.getLogger(DataElementHolder.class);

    private SortedMap<DataElementKey, DataElement> dataElements = Collections.synchronizedSortedMap(new TreeMap<DataElementKey, DataElement>());

    //the set of keys that are checked to be sure an end-of-cycle data holder is complete
    private static final DataElementKey[] validationKeys = {
            OPERATING_SPEED,				// user-defined desired cruise speed, m/s
            SPEED_COMMAND,					// command to be sent to the XGV, m/s
            SMOOTHED_SPEED,                 // computed speed via filtering based on configured filter
            SPEED,							// actual current vehicle speed, m/s
            ACCELERATION,                   // acceleration in m/s squared
            JERK,                           // rate of acceleration  m/s -3
            LATITUDE,						// deg
            LONGITUDE,						// deg
            MOTION_STATUS,                  // MotionStatus element
            CYCLE_GPS,                      // length of consumer call() method in ms....int
            CYCLE_MAP,
            CYCLE_SPAT,
            CYCLE_XGV,
            CYCLE_EAD,
            XGV_STATUS,                     // XgvStatus object
            STATUS_MESSAGE                  // StringBuffer element containing status messages
    };

    public DataElementHolder()   {
    }

    public DataElement get(DataElementKey key)   {
        return dataElements.get(key);
    }

    public void put(DataElementKey key, DataElement element)   {
        dataElements.put(key, element);
    }

    public void appendStatusMessage(String statusMessage)   {
        StringBufferDataElement currentValue = (StringBufferDataElement) dataElements.get(DataElementKey.STATUS_MESSAGE);
        if (currentValue == null)   {
            currentValue = new StringBufferDataElement(new StringBuffer());
        }
        currentValue.append(" " + statusMessage);
    }

    public int size() {
        return dataElements.size();
    }

    public void clear() {
    	dataElements.clear();
    }

    public double getDoubleElement(DataElementKey key)   {
        double value = 0.0;
        DataElement dataElement = dataElements.get( key );
        if ( dataElement != null)   {
            value = ( (DoubleDataElement) dataElement).value();
        }

        // not sure, but i think we wanna provide something other than 0 if no speed element
        if (dataElement == null && key == SPEED)   {
            value = -1.0;
        }
        return value;
    }

    /**
     * Ensure we get a value
     *
     * Primarily used when getting cycle times for operations from the holder.  In some situations, the element
     * may not exist, so we want to ensure we get a value in this situation i.e. 0
     *
     * @param key
     * @return
     */
    public int getIntElement(DataElementKey key)   {
        int value = 0;
        DataElement dataElement = dataElements.get( key );
        if ( dataElement != null)   {
            value = ( (IntDataElement) dataElement).value();
        }

        return value;
    }


    public void setLatitude(double latitude)   {
        dataElements.put(LATITUDE, new DoubleDataElement(latitude) );
    }

    public double getLatitude()   {
        return getDoubleElement( LATITUDE );
    }

    public void setLongitude(double longitude)   {
        dataElements.put(LONGITUDE, new DoubleDataElement(longitude) );
    }

    public double getLongitude()   {
        return getDoubleElement( LONGITUDE );
    }

    public void setSpeed(double velocity)   {
        dataElements.put(SPEED, new DoubleDataElement(velocity) );
    }

    public double getSpeed()   {
        return getDoubleElement( SPEED );
    }

    /**
     * Get the XgvStatus object if it exists, null otherwise
     *
     * @return XgvStatus or null
     */
    public XgvStatus getXgvStatus()   {
        XgvStatus xgvStatus = null;

        DataElement element = dataElements.get(XGV_STATUS);

        if (element != null && element instanceof XgvStatusDataElement)   {
            xgvStatus = ((XgvStatusDataElement) element).value();

        }

        return xgvStatus;

    }


    public void putAll(DataElementHolder newData)   {
        dataElements.putAll(newData.dataElements);
    }


    /**
     * Ensure we have a fully loaded holder that contains ALL keys
     *
     * Currently, we are ignoring MAP_MESSAGE
     *
     * @return  boolean
     */
    public boolean validate()   {
        boolean result = true;

        for (DataElementKey key : validationKeys)   {
                DataElement element = dataElements.get(key);
                if (element == null)  {
                    String statusMessage = "validateHolder missing " + key.toString() + " element.";
                    logger.warn(ILogger.TAG_EXECUTOR, statusMessage);
                    appendStatusMessage(statusMessage);
                    return false;
                }
        }

        return result;
    }

    /**
     * Removes the element from the holder and returns its value
     *
     * @param key DataElementKey
     * @return DataElement
     */
    public DataElement remove(DataElementKey key)   {
        DataElement element = dataElements.remove(key);
        return element;
    }

    /**
     * Provide a header string to identify the primary logged data items
     * Provides the holder elements in DataElementKey enum order
     *
     * @return String   Tab separated data element keys
     */
    public static String getLogHeader()    {
        StringBuffer sb = new StringBuffer();

        DataElementKey[] keys = DataElementKey.values();

        boolean firstValue = true;

        for (DataElementKey key : keys)   {
            if (!firstValue) {
                sb.append("\t");
            }

            sb.append(key);
            firstValue = false;
        }

        return sb.toString();
    }

    /**
     * Provide a single line of consumer acquired data to the log
     *
     * Provides the holder element VALUE in DataElementKey enum order
     *
     * @return String   Tab separated data element values
     */
    public String getLogString()   {
        StringBuffer sb = new StringBuffer();

        boolean firstValue = true;
        DataElementKey[] keys = DataElementKey.values();

        for (DataElementKey key : keys)   {

            DataElement value = dataElements.get(key);

            if (!firstValue) {
                sb.append("\t");
            }

            String strValue = "";
            if (value instanceof DoubleDataElement)   {
                strValue = Double.toString(((DoubleDataElement) value).value());
            }
            else if (value instanceof PhaseDataElement)   {
                strValue = ((PhaseDataElement) value).value().toString();
            }
            else if (value instanceof XgvStatusDataElement)   {
                strValue = ((XgvStatusDataElement) value).value().toString();
            }
            else if (value instanceof MotionStatusDataElement)   {
                strValue = ((MotionStatusDataElement) value).value().toString();
            }
            else if (value instanceof IntDataElement)   {
                strValue = Integer.toString(getIntElement(key));
            }
            else if (value instanceof StringBufferDataElement)   {
                strValue = ((StringBufferDataElement) value).value();
            }

            sb.append(strValue);

            firstValue = false;
        }

        return sb.toString();
    }


    @Override
    public String toString()   {
        StringBuffer sb = new StringBuffer();

        sb.append("\n     DataElementHolder [");

        for (Map.Entry<DataElementKey, DataElement> entry : dataElements.entrySet()) {
            DataElementKey key = entry.getKey();
            DataElement value = entry.getValue();

            String strValue = "";
            if (value instanceof DoubleDataElement)   {
                strValue = Double.toString(((DoubleDataElement) value).value());
            }
            else if (value instanceof PhaseDataElement)   {
                strValue = ((PhaseDataElement) value).value().toString();
            }
            else if (value instanceof XgvStatusDataElement)   {
                strValue = ((XgvStatusDataElement) value).value().toString();
            }
            else if (value instanceof MotionStatusDataElement)   {
                strValue = ((MotionStatusDataElement) value).value().toString();
            }

            sb.append("\n        " + key + ": " + strValue);
        }

        sb.append(" ]");

        return sb.toString();
    }
}


