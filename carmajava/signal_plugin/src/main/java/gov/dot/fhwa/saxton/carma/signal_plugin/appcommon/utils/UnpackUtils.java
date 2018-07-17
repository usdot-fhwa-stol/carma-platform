package gov.dot.fhwa.saxton.glidepath.appcommon.utils;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Utilities to unpack bytes to help deal with signed and unsigned data
 *
 */
public class UnpackUtils {

    private UnpackUtils() {
    }

    private static class UnpackUtilsHolder  {
        private static final UnpackUtils _instance = new UnpackUtils();
    }

    public static UnpackUtils getInstance()
    {
        return UnpackUtilsHolder._instance;
    }


    public int unpack8(byte theByte)   {
        return (int) theByte;
    }


    public int unpackU8(byte theByte)   {
        return (int) theByte & 0xFF;
    }


    public int unpack16(byte[] bytes)   {
        if (bytes.length != 2)   {
            return 0;
        }

        short shortValue = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getShort();
        return (int) shortValue;
    }



    public int unpackU16(byte[] bytes)   {
        if (bytes.length != 2)   {
            return 0;
        }

        short shortValue = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getShort();
        int intValue = (int) shortValue & 0xFFFF;

        return intValue;
    }


    public int unpack24(byte[] bytes)   {
        if (bytes.length != 3)   {
            return 0;
        }

        // Shift each byte to its proper position and OR it into the integer.
        int value = ((int)bytes[2]) << 16;
        value |= ((int)bytes[1]) << 8;
        value |= bytes[0];

        return value;
    }



    public int unpack32(byte[] bytes)   {

        if (bytes.length != 4)   {
            return 0;
        }

        int intValue = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getInt();

        return intValue;
    }



    public long unpackU32(byte[] bytes)   {

        if (bytes.length != 4)   {
            return 0;
        }

        int intValue = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN).getInt();

        return (long) intValue & 0xFFFFFFFFL;
    }

    public long unpackU32BigEndian(byte[] bytes)   {

        if (bytes.length != 4)   {
            return 0;
        }

        int intValue = ByteBuffer.wrap(bytes).order(ByteOrder.BIG_ENDIAN).getInt();

        return (long) intValue & 0xFFFFFFFFL;
    }


}