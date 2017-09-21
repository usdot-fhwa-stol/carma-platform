#pragma once
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <vector>
#include <bitset>
#include <stdexcept>
#include <boost/dynamic_bitset.hpp>

// This class assumes that the CAN data is arranged as shown below

//         __________Bits__________
// Byte 0 | 63 62 61 60 59 58 57 56
// Byte 1 | 55 54 53 52 51 50 49 48
// Byte 2 | 47 46 45 44 43 42 41 40
// Byte 3 | 39 38 37 36 35 34 33 32
// Byte 4 | 31 30 29 28 27 26 25 24
// Byte 5 | 23 22 21 20 19 18 17 16
// Byte 6 | 15 14 13 12 11 10 09 08
// Byte 7 | 07 06 05 04 03 02 01 00

// and are ordered in an array as shown below
//         data: [ B0 | B1 | B2 | B3 | B4 | B5 | B6 | B7 ]
// bitset index: [63,62,.............................,2,1,0]

namespace cav
{
namespace can
{

namespace FieldDataTypes
{
enum FieldDataType
{
    UNSIGNED = 0,
    SIGNED = 1,
};
}

/**
 * @brief The DataField class represents a single data field in a CAN message.
 *        It provides functions to access the data using the configured encoding.
 */
class DataField
{

private: // private data members
    FieldDataTypes::FieldDataType data_type_;
    float scale_factor_;
    float offset_;
    unsigned char data_starting_bit_;
    unsigned char data_bit_length_;
    bool little_endian_;


public:
    typedef std::bitset<64> bitset64;


    /**
     * @brief Construct for data field
     * @param type Field Type, if SIGNED then msb is considered sign bit
     * @param start where in the dataset is the field
     * @param length number of bits composing field
     * @param littleEndian if littleEndian set to true
     * @param scaleFactor scaling factor
     * @param offset offset of from 0
     */
    DataField(FieldDataTypes::FieldDataType type, unsigned char start, unsigned char length, bool littleEndian, float scaleFactor, float offset)
            : data_type_(type), data_starting_bit_(start), data_bit_length_(length), little_endian_(littleEndian), scale_factor_(scaleFactor), offset_(offset)
    {

    }



    /**
     * @brief Extract the field from the passed data
     * @param data
     * @return
     */
    float getValue(const std::bitset<64>& data) const
    {
        boost::dynamic_bitset<> subset;
        size_t start = data_starting_bit_ - (data_bit_length_ - 1); // start closer to end of message a.k.a lesser significant byte a.k.a towards byte 8
        size_t end = data_starting_bit_; // end closer to top of message a.k.a. more significant byte a.k.a. towards byte 0

        for (size_t i = start; i <= end; i++)
        {
            subset.push_back(data[i]);
        }

        int64_t unscaled_value;

        if (data_type_ == FieldDataTypes::SIGNED)
        {
            if ( subset.test(subset.size() - 1) ) // negative number for twos complement
            {
                // invert the digits and add 1
                subset = ~subset;

                int64_t temp = subset.to_ulong();
                temp = temp + 1; // add 1

                unscaled_value = -temp;
            }
            else
            {
                unscaled_value = subset.to_ulong();
            }
        }
        else if (data_type_ == FieldDataTypes::UNSIGNED)
        {
            unscaled_value = subset.to_ulong();
        }
        else
        {
            throw std::invalid_argument("FieldDataType must be UNSIGNED|SIGNED");
        }

        return (float)(unscaled_value * scale_factor_ + offset_);
    }


    /**
     * @brief Sets the value of the field within data
     * @param data
     * @param value
     */
    void setValue(std::bitset<64>& data, const float& value) const
    {
        //std::cout << "setValue(data, " << value << ")" << std::endl;
        float scaled = (value - offset_)/scale_factor_;
        //std::cout << "scaled = " << scaled << std::endl;
        int64_t sll = (int64_t)scaled;
        //std::cout << "sll = " << sll << std::endl;

        uint64_t to_pack;

        if (data_type_ == FieldDataTypes::SIGNED)
        {
            to_pack = sll;
        }
        else if (data_type_ == FieldDataTypes::UNSIGNED)
        {
            if (sll < 0)  // make sure it's not a negative number
                sll = 0;

            to_pack = sll;
        }

        std::bitset<64> packed_data(to_pack);

        int start = data_starting_bit_ - (data_bit_length_ - 1);
        int end   = data_starting_bit_;
        unsigned int j = 0;
        for (unsigned int i = start; i <= end; i++)
        {
            data[i] = packed_data[j];
            j++;
        }


    }

    /**
     * @brief Extracts unsigned int from data
     * @param data
     * @return
     */
    unsigned int getUnsignedIntValue(const std::bitset<64>& data) const
    {
        boost::dynamic_bitset<> subset;
        size_t start = data_starting_bit_ - (data_bit_length_ - 1); // start closer to end of message a.k.a lesser significant byte a.k.a towards byte 8
        size_t end = data_starting_bit_; // end closer to top of message a.k.a. more significant byte a.k.a. towards byte 0

        for (size_t i = start; i <= end; i++)
        {
            subset.push_back(data[i]);
        }
        unsigned long temp = subset.to_ulong();
        return (unsigned int)(temp * scale_factor_ + offset_);
    }

};



}
}