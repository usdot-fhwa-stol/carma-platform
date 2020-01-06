/*
 * Copyright (C) 2018-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.message.helper;

/**
 * This class has three functionalities. It can convert a string type into
 * a byte array and set the byte array as a field based on input field parameter.
 * It will handle both a field with required length or a field with dynamic
 * length. It can also convert a long type timestamp into a byte array and
 * set the byte array as a filed based on the input field parameter. This class
 * will be used mainly by different Mobility helper class. Its third functionality
 * is to read all data between '[' and ']' from a byte[] and return as a String. 
 */
public class StringConverterHelper {
    
    public static final String DYNAMIC_STRING_DEFAULT = "[]";
    public static final int TIMESTAMP_LENGTH = Long.toString(Long.MAX_VALUE).length();
    
    public static byte[] setDynamicLengthString(String inputString, int maxLength) {
        char[] tmp;
        if(inputString.length() <= maxLength && inputString.length() != 0) {
            tmp = inputString.toCharArray();   
        } else {
            tmp = DYNAMIC_STRING_DEFAULT.toCharArray();
        }
        byte[] field = new byte[tmp.length];
        for(int i = 0; i < tmp.length; i++) {
            field[i] = (byte) tmp[i];
        }
        return field;
    }
    
    public static byte[] setFixedLengthString(String inputString, int length, String defaultString) {
        char[] tmp;
        if(inputString.length() == length) {
            tmp = inputString.toCharArray();
        } else {
            tmp = defaultString.toCharArray();
        }
        byte[] field = new byte[length];
        for(int i = 0; i < tmp.length; i++) {
            field[i] = (byte) tmp[i];
        }
        return field;
    }
    
    public static byte[] setTimestamp(long time) {
        StringBuffer number = new StringBuffer(Long.toString(time));
        int numberOfZeroNeeded = TIMESTAMP_LENGTH - number.length();
        for(int i = 0; i < numberOfZeroNeeded; i++) {
            number.insert(0, '0');
        }
        char[] tmp = number.toString().toCharArray();
        byte[] field = new byte[tmp.length];
        for(int i = 0; i < tmp.length; i++) {
            field[i] = (byte) tmp[i];
        }
        return field;
    }
    
    public static String readDynamicLengthString(byte[] input) {
        StringBuffer buffer = new StringBuffer();
        for(byte ch : input) {
            if(ch == 0) {
                break;
            } else {
                buffer.append((char) ch);
            }
        }
        String bufferString = buffer.toString();
        // If this is a default dynamic string return an empty string
        if (bufferString.equals(DYNAMIC_STRING_DEFAULT)) {
            return "";
        }
        return bufferString;
    }
}
