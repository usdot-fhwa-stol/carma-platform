#pragma once

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

template<typename T = std::string>
class CANMessageDefinition
{
public:
    CANMessageDefinition()
    {

    }

    virtual ~CANMessageDefinition() {}

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
    void setData(std::bitset<64>& data)
    {
        data_ = data;
    }

    void setData(unsigned char const * const data, const unsigned char data_length)
    {
        if (data_length > 8)
            throw std::runtime_error("CanMessage::setData() data_lenth exceeds 8 bytes");

        if (data_length < 8)
            std::cerr << "CAN message Data length is: " << data_length << std::endl;


        for (unsigned int i = 0; i < data_length; i++)
            setByte(data_, i, data[i]);
    }

    void toBytes(unsigned char* data, unsigned char data_length)
    {
        for (unsigned int i = 0; i < data_length; i++)
        {
            data[i] = getByte(data_, i);
        }
    }

    void clearData()
    {
        data_.reset();
    }

    void addFieldDefinition(const T& name, DataField& fieldDefinition)
    {
        fields_.insert(std::make_pair(name,fieldDefinition));
    }

    float getValue(const T& fieldName) const
    {
        const DataField& field = getField(fieldName);

        return field.getValue(data_);
    }

    unsigned int getUnsignedIntValue(const T& fieldName) const
    {
        const DataField& field = getField(fieldName);
        return field.getUnsignedIntValue(data_);
    }

    void setValue(T fieldName, float value)
    {
        const DataField& field = getField(fieldName);

        field.setValue(data_, value);
    }

    std::string dataToString() const
    {
        return data_.to_string();
    }
private: // private functions

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