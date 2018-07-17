package gov.dot.fhwa.saxton.glidepath.appcommon.utils;

/**
 * Facilitates the parsing of smaller-than-byte values out of a byte array.
 *
 */
public class BitStreamUnpacker {
    private int size;
    private int currentBit = 0;
    private byte[] bytes;
    private static int leastSignificantBitMask = 0x00000001;
    private boolean littleEndian = false;

    public BitStreamUnpacker(byte[] bytes) {
        this.bytes = bytes;
        this.size = bytes.length * 8;
    }

    public BitStreamUnpacker(byte[] bytes, boolean littleEndian) {
        this(bytes);
        this.littleEndian = littleEndian;
    }

    public int readBit() {
        // Bounds check on byte array
        if (currentBit >= size) {
            throw new IndexOutOfBoundsException("Attempted read beyond end of bitstream.");
        }

        // Find the byte in the byte array we want and find the bit in that byte we want
        byte container = bytes[currentBit / 8];
        int bitIndex = currentBit % 8;

        // Clear the bits on the left, put the bit we want all the way in the LSB, then bitmask out all the other bits
        int value = ((container << (7 - bitIndex)) >>> 7) & leastSignificantBitMask;

        this.currentBit++;
        return value;
    }

    public boolean readBool() {
        return readBit() == 1;
    }

    public int readBits(int numBits) {
        if (numBits > 32) {
            // TODO: Find or create a more descriptive exception
            throw new IndexOutOfBoundsException("Attempted to read larger than 32 bits of data into an int return value");
        }

        int out = 0;
        for (int i = 0; i < numBits; i++) {
            out += (readBit() << i);
        }

        return out;
    }

    public int readByte() {
        return readBits(8);
    }

    public byte[] readByteArray(int numBytes) {
        byte[] out = new byte[numBytes];
        for (int i = 0; i < numBytes; i++) {
            out[i] = (byte) readByte();
        }

        return out;
    }

    public int readShort() {
        int out = readBits(16);

        if (!littleEndian) {
            out = swapEndianShort(out);
        }

        return out;
    }

    public int readInt() {
        int out = readBits(32);

        if (!littleEndian) {
            out = swapEndianInt(out);
        }

        return out;
    }

    private int swapEndianInt(int i) {
        return (((i & 0xff) << 24) + ((i & 0xff00) << 8) + ((i & 0xff0000) >> 8) + (i >>> 24));
    }

    private int swapEndianShort(int i) {
        return (((i & 0xff) << 8) + ((i & 0xff00) >>> 8));
    }

    public boolean isLittleEndian() {
        return littleEndian;
    }

    public void setBigEndian() {
        littleEndian = false;
    }

    public void setLittleEndian() {
        littleEndian = true;
    }
}
