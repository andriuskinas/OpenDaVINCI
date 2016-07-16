/**
 * OpenDaVINCI - Portable middleware for distributed components.
 * Copyright (C) 2015 Christian Berger
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <cstring>
#include <iostream>
#include <vector>

#include "opendavinci/odcore/opendavinci.h"
#include "opendavinci/odcore/base/ProtoDeserializer.h"
#include "opendavinci/odcore/base/ProtoDeserializerVisitor.h"
#include "opendavinci/odcore/base/Visitable.h"

namespace odcore {
    namespace base {

class Serializable;

        ProtoDeserializer::ProtoDeserializer() :
            m_size(0),
            m_buffer(NULL) {}

        ProtoDeserializer::ProtoDeserializer(istream &i) :
            m_size(0),
            m_buffer(&i) {
            deserializeDataFrom(i);
        }

        ProtoDeserializer::~ProtoDeserializer() {}

        void ProtoDeserializer::deserializeDataFrom(istream &in) {
            // Reset internal states as this deserializer could be reused.
            m_size = 0;
            m_buffer = &in;

            // Disable copying the data from istream into the decoding buffer
            // to avoid fully consuming the bytes that cannot be fully processed
            // by the class using this decoder.
//            while (in.good()) {
//                char c = in.get();
//                m_buffer.put(c);
//            }
        }

        void ProtoDeserializer::deserializeDataFromWithHeader(istream &in) {
            // Reset internal states as this deserializer could be reused.
            m_size = 0;
            m_buffer = &in;

            // Read magic number.
            uint16_t magicNumber = 0;
            in.read((char*)(&magicNumber), sizeof(magicNumber));
            magicNumber = ntohs(magicNumber);
            if (magicNumber == 0x0DA4) {
                // Read message size and put remaining bytes into m_buffer.
                uint64_t value = 0;
                decodeVarInt(in, value);
                m_size = static_cast<uint32_t>(value);

                // Disable copying the data from istream into the decoding buffer
                // to avoid fully consuming the bytes that cannot be fully processed
                // by the class using this decoder.
//                // Read up to m_size bytes as long as the input stream is fine.
//                uint32_t size = m_size;
//                while (in.good() && (size > 0)) {
//                    char c = in.get();
//                    m_buffer.put(c);
//                    size--;
//                }
            }
            else {
                // Stream is good but still no matching magic number?
                clog << "[core::base::ProtoDeserializer]: Stream corrupt: magic number not found." << endl;
            }
        }

        uint32_t ProtoDeserializer::readAndValidateKey(const uint32_t &id, const ProtoSerializerVisitor::PROTOBUF_TYPE &expectedType) {
            uint64_t key = 0;
            const uint32_t size = decodeVarInt(*m_buffer, key);
            const uint32_t fieldId = static_cast<uint32_t>(key >> 3);
            const ProtoSerializerVisitor::PROTOBUF_TYPE protoType = static_cast<ProtoSerializerVisitor::PROTOBUF_TYPE>(key & 0x7);

            if (fieldId != id) {
                clog << "[core::base::ProtoDeserializer]: ERROR! Field ids do not match: Found " << fieldId << ", expected: " << id << endl;
            }
            if (protoType != expectedType) {
                clog << "[core::base::ProtoDeserializer]: ERROR! Expected type does not match: Found " << protoType << ", expected " << expectedType << endl;
            }

            return size;
        }

        int8_t ProtoDeserializer::decodeZigZag8(uint8_t value) {
            return static_cast<int64_t>((value >> 1) ^ -(value & 1));
        }

        int16_t ProtoDeserializer::decodeZigZag16(uint16_t value) {
            return static_cast<int64_t>((value >> 1) ^ -(value & 1));
        }

        int32_t ProtoDeserializer::decodeZigZag32(uint32_t value) {
            return static_cast<int64_t>((value >> 1) ^ -(value & 1));
        }

        int64_t ProtoDeserializer::decodeZigZag64(uint64_t value) {
            return static_cast<int64_t>((value >> 1) ^ -(value & 1));
        }

        uint8_t ProtoDeserializer::decodeVarInt(istream &in, uint64_t &value) {
            value = 0;
            uint8_t size = 0;
            while (in.good()) {
                char c = in.get();
                value |= static_cast<unsigned int>( (c & 0x7f) << (0x7 * size++) );
                if ( !(c & 0x80) ) break;
            }

            // Protobuf's VarInt is based on little endian.
            value = le64toh(value);

            return size;
        }

        ///////////////////////////////////////////////////////////////////////

        uint32_t ProtoDeserializer::readValue(istream &i, Serializable &v) {
            uint32_t bytesRead = 0;

            // Read length.
            uint64_t length = 0;
            bytesRead += decodeVarInt(i, length);

            // Create contiguous buffer.
            vector<char> buffer(length);

            // Read data from stream into buffer.
            i.read(&buffer[0], length);
            bytesRead += length;

            // Create string from buffer.
            const string s(&buffer[0], &buffer[length]);

            // Deserialize v from string using a stringstream.
            stringstream sstr(s);

            // Check whether v is from type Visitable to use ProtoSerializerVisitor.
            try {
                const Visitable &v2 = dynamic_cast<const Visitable&>(v);

                // Cast succeeded, visited nested class using a ProtoSerializerVisitor.
                ProtoDeserializerVisitor nestedVisitor;

                // Set buffer to deserialize data from.
                nestedVisitor.deserializeDataFrom(sstr);

                // Visit v and set values.
                const_cast<Visitable&>(v2).accept(nestedVisitor);
            }
            catch(...) {
                // Deserialize v using the default way as it is not of type Visitable.
                sstr >> v;
            }

            return bytesRead;
        }

        uint32_t ProtoDeserializer::readValue(istream &i, bool &v) {
            uint64_t _v = 0;
            uint32_t bytesRead = decodeVarInt(i, _v);
            v = static_cast<bool>(_v);
            return bytesRead;
        }

        uint32_t ProtoDeserializer::readValue(istream &i, char &v) {
            uint64_t _v = 0;
            uint32_t bytesRead = decodeVarInt(i, _v);
            v = static_cast<char>(_v);
            return bytesRead;
        }

        uint32_t ProtoDeserializer::readValue(istream &i, unsigned char &v) {
            uint64_t _v = 0;
            uint32_t bytesRead = decodeVarInt(i, _v);
            v = static_cast<unsigned char>(_v);
            return bytesRead;
        }

        uint32_t ProtoDeserializer::readValue(istream &i, int8_t &v) {
            uint64_t _v = 0;
            uint32_t bytesRead = decodeVarInt(i, _v);
            v = static_cast<int8_t>(decodeZigZag8(_v));
            return bytesRead;
        }

        uint32_t ProtoDeserializer::readValue(istream &i, int16_t &v) {
            uint64_t _v = 0;
            uint32_t bytesRead = decodeVarInt(i, _v);
            v = static_cast<int16_t>(decodeZigZag16(_v));
            return bytesRead;
        }

        uint32_t ProtoDeserializer::readValue(istream &i, uint16_t &v) {
            uint64_t _v = 0;
            uint32_t bytesRead = decodeVarInt(i, _v);
            v = static_cast<uint16_t>(_v);
            return bytesRead;
        }

        uint32_t ProtoDeserializer::readValue(istream &i, int32_t &v) {
            uint64_t _v = 0;
            uint32_t bytesRead = decodeVarInt(i, _v);
            v = static_cast<int32_t>(decodeZigZag32(_v));
            return bytesRead;
        }

        uint32_t ProtoDeserializer::readValue(istream &i, uint32_t &v) {
            uint64_t _v = 0;
            uint32_t bytesRead = decodeVarInt(i, _v);
            v = static_cast<uint32_t>(_v);
            return bytesRead;
        }

        uint32_t ProtoDeserializer::readValue(istream &i, int64_t &v) {
            uint64_t _v = 0;
            uint32_t bytesRead = decodeVarInt(i, _v);
            v = static_cast<int64_t>(decodeZigZag64(_v));
            return bytesRead;
        }

        uint32_t ProtoDeserializer::readValue(istream &i, uint64_t &v) {
            uint64_t _v = 0;
            uint32_t bytesRead = decodeVarInt(i, _v);
            v = _v;
            return bytesRead;
        }

        uint32_t ProtoDeserializer::readValue(istream &i, float &v) {
            // 4 bytes values need to obey little endian encoding.
            uint32_t _v = 0;
            i.read(reinterpret_cast<char*>(&_v), sizeof(uint32_t));
            _v = le32toh(_v);
            v = *(reinterpret_cast<float*>(&_v));
            return sizeof(const uint32_t);
        }

        uint32_t ProtoDeserializer::readValue(istream &i, double &v) {
            // 8 bytes values need to obey little endian encoding.
            uint64_t _v = 0;
            i.read(reinterpret_cast<char*>(&_v), sizeof(uint64_t));
            _v = le64toh(_v);
            v = *(reinterpret_cast<double*>(&_v));
            return sizeof(const uint64_t);
        }

        uint32_t ProtoDeserializer::readValue(istream &i, string &v) {
            uint32_t bytesRead = 0;

            // Read length.
            uint64_t length = 0;
            bytesRead += decodeVarInt(i, length);

            // Create contiguous buffer.
            vector<char> buffer(length);

            // Read data from stream.
            i.read(&buffer[0], length);
            bytesRead += length;

            // Create string from buffer.
            v = string(&buffer[0], &buffer[length]);

            return bytesRead;
        }

        uint32_t ProtoDeserializer::readValue(istream &i, void *data, const uint32_t &size) {
            uint32_t bytesRead = 0;

            // Read length.
            uint64_t length = 0;
            bytesRead += decodeVarInt(i, length);

            // Read data.
            char *_data = new char[length];
            i.read(_data, length);
            bytesRead += length;

            // Move data.
            memset(data, 0, size);
            memcpy(data, _data, (size < length ? size : length));
            OPENDAVINCI_CORE_DELETE_ARRAY(_data);

            return bytesRead;
        }

        ///////////////////////////////////////////////////////////////////////

        void ProtoDeserializer::read(const uint32_t &id, Serializable &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::LENGTH_DELIMITED);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, bool &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::VARINT);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, char &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::VARINT);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, unsigned char &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::VARINT);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, int8_t &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::VARINT);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, int16_t &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::VARINT);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, uint16_t &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::VARINT);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, int32_t &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::VARINT);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, uint32_t &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::VARINT);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, int64_t &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::VARINT);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, uint64_t &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::VARINT);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, float &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::FOUR_BYTES);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, double &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::EIGHT_BYTES);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, string &v) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::LENGTH_DELIMITED);
            m_size -= readValue(*m_buffer, v);
        }

        void ProtoDeserializer::read(const uint32_t &id, void *data, const uint32_t &size) {
            m_size -= readAndValidateKey(id, ProtoSerializerVisitor::LENGTH_DELIMITED);
            m_size -= readValue(*m_buffer, data, size);
        }

        ///////////////////////////////////////////////////////////////////////

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, Serializable &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, bool &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, char &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, unsigned char &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, int8_t &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, int16_t &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, uint16_t &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, int32_t &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, uint32_t &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, int64_t &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, uint64_t &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, float &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, double &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, string &v) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , v);
        }

        void ProtoDeserializer::read(const uint32_t &fourByteID, const uint8_t &oneByteID, const string &/*longName*/, const string &/*shortName*/, void *data, const uint32_t &size) {
            read( (oneByteID > 0 ? oneByteID : fourByteID) , data, size);
        }

    }
} // odcore::base

