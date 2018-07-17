package gov.dot.fhwa.saxton.carma.signal_plugin.appcommon.utils;

import java.util.ArrayList;

/**
 * Manages creation of a packet by emulating a stream but allowing the writing of bit-level data as opposed to the
 * Java normal byte-level access. Operates in a big-endian mode since Java is big-endian.
 */
public class BitStreamPacker {
    // Already stored bytes
    ArrayList<Byte> bytes = new ArrayList<Byte>();

    // Current byte being modified
    byte cur;

    // Index into the current byte being worked on
    int bitIndex = 0;

    private boolean littleEndian = false;

    public BitStreamPacker() {

    }

    public BitStreamPacker(boolean littleEndian) {
        this.littleEndian = littleEndian;
    }

    public void writeBit(int data) {
        // Fill the bit at bitIndex with our value
        cur |= (data << bitIndex);

        // Increment the bitIndex
        bitIndex = ++bitIndex % 8;

        // We filled the byte
        if (bitIndex == 0) {
            bytes.add(cur);
            cur = 0;
        }
    }

    public void writeBits(int data, int size) {
        // Iterate down the byte, writing the values we find

        int index = 1;
        for (int i = 0; i < size; i++) {
            writeBit((index & data) >> i);
            index <<= 1;
        }
    }

    public void writeByte(int data) {
        writeBits(data, 8);
    }

    public void writeShort(int data) {
        if (!littleEndian) {
            data = swapEndianShort(data);
        }

        writeBits(data, 16);
    }

    public void writeInt(int data) {
        if (!littleEndian) {
            data = swapEndianInt(data);
        }

        writeBits(data, 32);
    }

    public void writeBool(boolean data) {
        writeBit(data ? 1 : 0);
    }

    public void writeByteArray(byte[] data) {
        for (int i = 0; i < data.length; i++) {
            writeByte(data[i]);
        }
    }

    public byte[] getBytes() {
        byte[] data;

        // Adjustment for size of list if we have a current working byte to write
        if (bitIndex == 0) {
            data = new byte[bytes.size()];
        } else {
            data = new byte[bytes.size() + 1];
        }

        // Copy all the full bytes
        for (int i = 0; i < bytes.size(); i++) {
            data[i] = bytes.get(i).byteValue();
        }

        // Copy the working byte
        if (bitIndex != 0) {
            data[bytes.size()] = (byte) (cur << (8 - bitIndex));
        }

        return data;
    }

    private int swapEndianInt(int i) {
        return (((i & 0xff) << 24) + ((i & 0xff00) << 8) + ((i & 0xff0000) >> 8) + (i >>> 24));
    }

    private int swapEndianShort(int i) {
        return (((i & 0xff) << 8) + ((i & 0xff00) >>> 8));
    }

    public void setBigEndian() {
        littleEndian = false;
    }

    public void setLittleEndian() {
        littleEndian = true;
    }
}
