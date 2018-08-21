package gov.dot.fhwa.saxton.carma.signal_plugin.xgv;

/**
 * Converts XGV integer values into equivalent Java types
 */
public class XgvIntegerConverter {

    /**
     * Converts a scaled int stored as an unsigned integer into a real value stored as a double.
     * @param val The raw scaled int value
     * @param max The maximum value for the scaled int
     * @param min The minimum value of the scaled int
     * @param bits The number of bits in the scaled int
     * @return The real value encoded in the scaled int
     */
    public static double unsignedScaledToReal(int val, double max, double min, int bits) {
        return val * ((max - min)/(Math.pow(2.0, bits ) - 1)) + min;
    }

    /**
     * Converts a scaled int stored as a signed integer into a real value stored as a double.
     * @param val The raw scaled int value
     * @param max The maximum value for the scaled int
     * @param min The minimum value of the scaled int
     * @param bits The number of bits in the scaled int
     * @return The real value encoded in the scaled int
     */
    public static double signedScaledToReal(int val, double max, double min, int bits) {
        return (val / 2.0) * ((max - min)/(Math.pow(2.0, bits - 1) - 1)) + (max + min)/2.0;
    }

    /**
     * Converts a real value represented as a double to a an unsigned scaled integer.
     * @param val The raw real value
     * @param max The maximum value for the scaled int to be generated
     * @param min The minimum value of the scaled int to be generated
     * @param bits The number of bits in the scaled int to be generated
     * @return The unsigned scaled int
     */
    public static int realToUnsignedScaled(double val, double max, double min, int bits) {
        return (int) ((val - min) * ((Math.pow(2.0, bits) - 1)/(max - min)));
    }

    /**
     * Converts a real value represented as a double to a signed scaled integer.
     * @param val The raw real value
     * @param max The maximum value for the scaled int to be generated
     * @param min The minimum value of the scaled int to be generated
     * @param bits The number of bits in the scaled int to be generated
     * @return The signed scaled int
     */
    public static int realToSignedScaled(double val, double max, double min, int bits) {
        return (int) ((val - ((max + min)/2.0)) * (2 * (Math.pow(2.0, bits - 1)/(max - min))));
    }
}
