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

#ifndef OPENDAVINCI_CORE_BASE_LCMSERIALIZERVISITOR_H_
#define OPENDAVINCI_CORE_BASE_LCMSERIALIZERVISITOR_H_

#include <sstream>
#include <string>

#include "opendavinci/odcore/opendavinci.h"
#include "opendavinci/odcore/serialization/Serializer.h"
#include "opendavinci/odcore/base/Visitor.h"

namespace odcore {
    namespace base {

class Serializable;

        using namespace std;

        /**
         * This class provides a serialization visitor to encode data
         * in LCM format (cf. https://github.com/lcm-proj/lcm).
         */
        class LCMSerializerVisitor : public Serializer, public Visitor {
            private:
                /**
                 * "Forbidden" copy constructor. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the copy constructor.
                 */
                LCMSerializerVisitor(const LCMSerializerVisitor &);

                /**
                 * "Forbidden" assignment operator. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the assignment operator.
                 */
                LCMSerializerVisitor& operator=(const LCMSerializerVisitor &);

            public:
                LCMSerializerVisitor();

                virtual ~LCMSerializerVisitor();

                virtual void getSerializedData(ostream &o);

                /**
                 * This method sets the channel name to be used for the serialized data.
                 *
                 * @param channelName Name of the LCM channel for which this data shall be serialized.
                 */
                void setChannelName(const string &channelName);

                /**
                 * This method returns the calculated hash value.
                 *
                 * @return Calculated hash.
                 */
                int64_t getHash() const;

            private:
                virtual uint32_t writeValue(ostream &o, const Serializable &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const bool &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const char &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const unsigned char &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const int8_t &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const int16_t &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const uint16_t &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const int32_t &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const uint32_t &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const int64_t &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const uint64_t &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const float &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const double &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const string &v) {(void)o; (void)v; return 0;}
                virtual uint32_t writeValue(ostream &o, const void *data, const uint32_t &size) {(void)o; (void)data; (void)size; return 0;}

            private:
                virtual void write(const uint32_t &id, const Serializable &s);
                virtual void write(const uint32_t &id, const bool &b);
                virtual void write(const uint32_t &id, const char &c);
                virtual void write(const uint32_t &id, const unsigned char &uc);
                virtual void write(const uint32_t &id, const int8_t &i);
                virtual void write(const uint32_t &id, const int16_t &i);
                virtual void write(const uint32_t &id, const uint16_t &ui);
                virtual void write(const uint32_t &id, const int32_t &i);
                virtual void write(const uint32_t &id, const uint32_t &ui);
                virtual void write(const uint32_t &id, const int64_t &i);
                virtual void write(const uint32_t &id, const uint64_t &ui);
                virtual void write(const uint32_t &id, const float &f);
                virtual void write(const uint32_t &id, const double &d);
                virtual void write(const uint32_t &id, const string &s);
                virtual void write(const uint32_t &id, const void *data, const uint32_t &size);

            private:
                virtual void beginVisit();
                virtual void endVisit();

                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const Serializable &s);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const bool &b);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const char &c);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const unsigned char &uc);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const int8_t &i);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const int16_t &i);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const uint16_t &ui);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const int32_t &i);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const uint32_t &ui);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const int64_t &i);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const uint64_t &ui);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const float &f);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const double &d);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const string &s);
                virtual void write(const uint32_t &id, const string &longName, const string &shortName, const void *data, const uint32_t &size);

            public:
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, Serializable &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, bool &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, char &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, unsigned char &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, int8_t &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, int16_t &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, uint16_t &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, int32_t &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, uint32_t &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, int64_t &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, uint64_t &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, float &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, double &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, string &v);
                virtual void visit(const uint32_t &id, const string &longName, const string &shortName, void *data, const uint32_t &size);

            public:
                /**
                 * This method is adapted from LCM to calculate the corresponding hash value.
                 */
                static int64_t calculateHash(int64_t v, const char &c);

                /**
                 * This method is adapted from LCM to calculate the corresponding hash value.
                 */
                static int64_t calculateHash(int64_t v, const string &s);

            private:
                string m_channelName;
                int64_t m_hash;
                stringstream m_buffer;
        };

    }
} // odcore::base

#endif /*OPENDAVINCI_CORE_BASE_LCMSERIALIZERVISITOR_H_*/
