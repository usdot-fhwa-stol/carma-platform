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

#include <map>
#include <bitset>
#include <iostream>
#include <string>
#include <sstream>

#include "data_field.h"

namespace cav
{
namespace can
{


/**
 * @brief This class helps parse CAN Messages by using datafields to parse the data
 * @tparam T key type for field query/set
 */
template<typename T = std::string>
class CANMessageDefinition
{
public:
    CANMessageDefinition()
    {

    }

    virtual ~CANMessageDefinition() {}


    /**
     * @brief Sets the binary representtion of the internal data
     * @param data
     */
    void setData(unsigned long long data)
    {
        unsigned long l,h;
        l = data & 0x00000000FFFFFFFF;
        h = (data >> 32) & 0x00000000FFFFFFFF;
        std::bitset<32> low(l);
        std::bitset<32> high(h);

        for (unsigned int i = 0; i < 32; i++)
            data_[i] = low[i];
        for (unsigned int i = 0; i < 32; i++)
            data_[i+32] = high[i];
    }


    /**
     * @brief Sets the binary representtion of the internal data
     * @param data
     */
    void setData(std::bitset<64>& data)
    {
        data_ = data;
    }

    /**
     * @brief Sets the binary representtion of the internal data
     * @param data array containing data to set the internal data to
     * @param data_length
     */
    void setData(unsigned char const * const data, const unsigned char data_length)
    {
        if (data_length > 8)
            throw std::runtime_error("CanMessage::setData() data_lenth exceeds 8 bytes");

        if (data_length < 8)
            std::cerr << "CAN message Data length is: " << data_length << std::endl;


        for (unsigned int i = 0; i < data_length; i++)
            setByte(data_, i, data[i]);
    }


    /**
     * @brief Converts the internal data representation to a byte array
     * @param data output is stored in this array
     * @param data_length
     */
    void toBytes(unsigned char* data, unsigned char data_length)
    {
        for (unsigned int i = 0; i < data_length; i++)
        {
            data[i] = getByte(data_, i);
        }
    }


    /**
     * @brief Clears the internal dta
     */
    void clearData()
    {
        data_.reset();
    }

    /**
     * @brief Adds a data definition
     * @param name
     * @param fieldDefinition
     */
    void addFieldDefinition(const T& name, DataField& fieldDefinition)
    {
        fields_.insert(std::make_pair(name,fieldDefinition));
    }


    /**
     * @brief Gets the value associated with a definition
     * @param fieldName
     * @return
     */
    float getValue(const T& fieldName) const
    {
        const DataField& field = getField(fieldName);

        return field.getValue(data_);
    }


    /**
     * @brief Gets the unsigned value associated with a value
     * @param fieldName
     * @return
     */
    unsigned int getUnsignedIntValue(const T& fieldName) const
    {
        const DataField& field = getField(fieldName);
        return field.getUnsignedIntValue(data_);
    }


    /**
     * @brief Sets the value of a field
     * @param fieldName
     * @param value
     */
    void setValue(T fieldName, float value)
    {
        const DataField& field = getField(fieldName);

        field.setValue(data_, value);
    }

    /**
     * @brief Converts the data to human readable bytes
     * @return
     */
    std::string dataToString() const
    {
        return data_.to_string();
    }
private: // private functions

    /**
     * @brief Extracts a byte from the bitset. Allows getting unaligned bytes
     * @param bits bitset to extract byte from
     * @param byte_address the index of the byte to start extracting from
     * @return
     */
    unsigned char getByte(std::bitset<64>& bits, int byte_address)
    {
        unsigned char b = 0;
        unsigned char bit_bottom   = (7-byte_address)*8;

        for (unsigned int i = 0; i < 8; i++)
        {
            if (bits[bit_bottom + i])
                b |= 1 << i;
        }

        return b;
    }

    /**
     * @brief Sets a byte worth of data starting a byte_address. byte_address can be any offset
     * @param bits
     * @param byte_address
     * @param byte
     */
    void setByte(std::bitset<64>& bits, int byte_address, unsigned char byte)
    {
        unsigned char bit_bottom = (7-byte_address)*8;

        for (unsigned int i = 0; i < 8; i++)
        {
            if (byte & (1 << i))
                bits[bit_bottom + i] = 1;
            else
                bits[bit_bottom + i] = 0;
        }
    }


    /**
     * @brief Gets the datafield associated with a key
     * @param name
     * @return
     */
    const DataField& getField(const T& name) const
    {
        if (fields_.find(name) == fields_.end())
        {
            std::stringstream ss;
            ss << "Requested field does not exist: " << name;
            throw std::runtime_error("Requested field does not exist: " + name);
        }

        return fields_.at(name);
    }

private: // private data members

    std::map<T, DataField> fields_;
    std::bitset<64> data_;
};
}
}