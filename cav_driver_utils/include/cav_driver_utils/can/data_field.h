#pragma once

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

///
/// \brief The DataField class represents a single data field in a CAN message.
///        It provides functions to access the data using the configured encoding.
///
class DataField
{

private: // private data members
    FieldDataTypes::FieldDataType data_type_;
    float scale_factor_;
    float offset_;
    unsigned char data_starting_bit_;
    unsigned char data_bit_length_;
    bool little_endian_;
    bool packed_signed;
    float min_value_;
    float max_value_;

//    unsigned long long data_;
    std::bitset<64> data_;

public:
    typedef std::bitset<64> bitset64;
    DataField(FieldDataTypes::FieldDataType type, unsigned char start, unsigned char length, bool littleEndian, float scaleFactor, float offset)
            : data_type_(type), data_starting_bit_(start), data_bit_length_(length), little_endian_(littleEndian), scale_factor_(scaleFactor), offset_(offset)
    {

    }

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

private: // private functions

    bool isValid(int start, int length)
    {
        if (start + length < 64)
            return true;
        else
            return false;
    }

    unsigned long long fromVector(std::vector<unsigned char> data, bool littleEndian)
    {
        if (data.size() != 8)
        {
            throw std::runtime_error("fromVector(): data must be 8 bytes long");
        }

        unsigned long long ret_val;

        if (littleEndian) // (lest significant byte is at index 0)
        {
            for (unsigned int i = 0; i < data.size(); i++)
            {
                ret_val = ret_val | data[i] << 8*(7-i);
            }
        }
        else // big endian (most significant byte is at index 0
        {
            for (unsigned int i = 0; i < data.size(); i++)
            {
                ret_val = ret_val | data[i] << 8*i;
            }
        }
    }

    std::vector<unsigned char> fromULL(unsigned long long data, bool littleEndian)
    {
        std::vector<unsigned char> ret_val;
        ret_val.resize(8, 0);

        if (littleEndian) // (least significant byte is at index 0)
        {
            for (unsigned int i = 0; i < 8; i++)
            {
                ret_val[i] = (data >> (7-i)*8) & 0xFF;
            }
        }
        else // big endian (most significant byte is at index 0
        {
            for (unsigned int i = 0; i < 8; i++)
            {
                ret_val[i] = (data >> i*8) & 0xFF;
            }
        }
    }

};



}
}