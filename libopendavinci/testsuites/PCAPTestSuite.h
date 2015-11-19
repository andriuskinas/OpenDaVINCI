/**
 * OpenDaVINCI - Portable middleware for distributed components.
 * Copyright (C) 2008 - 2015 Christian Berger, Bernhard Rumpe
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

#ifndef CORE_PCAPTESTSUITE_H_
#define CORE_PCAPTESTSUITE_H_

#include "cxxtest/TestSuite.h"

#include <sstream>

#include "core/data/Container.h"
#include "core/io/conference/ContainerListener.h"
#include "core/io/protocol/PCAPProtocol.h"

#include "GeneratedHeaders_CoreData.h"

using namespace std;
using namespace core::base;
using namespace core::data;
using namespace coredata;
using namespace core::io::protocol;


class PCAPTest : public CxxTest::TestSuite, public core::io::conference::ContainerListener {
    public:
        bool passed;

        virtual void nextContainer(core::data::Container &c) {
            static int nextID = 1000;

            cout << "Received container " << c.getDataType() << endl;

            if (c.getDataType() == 1000) {
                passed &= (c.getDataType() == nextID++);
            }
            if (c.getDataType() == 1001) {
                passed &= (c.getDataType() == nextID++);
            }
            if (c.getDataType() == 1002) {
                // Here, we have a valid packet.
                passed &= (c.getDataType() == nextID);
                nextID = 1001;

                {
                    pcap::Packet packet = c.getData<pcap::Packet>();
                    pcap::PacketHeader packetHeader = packet.getHeader();
                    const string payload = packet.getPayload();

                    cout << packetHeader.toString() << endl;
                    cout << "Payload: '" << payload << "'" << endl;
                }
            }
        }

        void testPCAPDecoding() {
            passed = true;
            stringstream rawDataStream;

            // Serialized raw data (pcap excerpt).
            string data = "-44 -61 -78 -95 2 0 4 0 0 0 0 0 0 0 0 0 -1 -1 0 0 1 0 0 0 -96 -98 -59 83 61 73 7 0 -32 4 0 0 -32 4 0 0 -1 -1 -1 -1 -1 -1 0 0 0 0 0 0 8 0 69 0 4 -46 0 1 0 0 -128 17 -83 -97 -64 -88 3 43 -64 -88 3 -1 1 -69 9 64 4 -66 0 0 -1 -35 -19 15 98 9 51 64 9 43 102 17 60 116 18 43 -92 6 -39 55 7 77 20 1 32 -5 0 -121 -78 1 -70 0 0 15 0 0 16 -65 0 -42 -89 12 72 -51 12 59 -58 10 35 0 0 15 -55 6 -25 -34 6 -42 -127 7 -119 -90 6 -34 18 14 55 -88 14 39 -124 11 48 -16 11 50 -4 19 48 -121 20 27 -109 15 59 -46 15 53 87 21 48 76 22 38 44 16 54 46 7 -75 -1 -18 -1 15 26 26 27 81 27 39 0 0 16 0 0 15 -101 28 40 -97 28 45 -69 23 57 -18 24 51 43 29 31 -66 32 64 15 26 50 54 27 46 12 49 61 109 51 87 16 37 48 -115 38 65 -98 60 83 60 77 90 54 41 76 -111 43 76 72 92 64 0 0 15 -12 46 70 -94 50 67 0 0 16 0 0 15 69 52 81 32 56 88 0 0 16 0 0 15 31 66 32 0 0 15 -1 -18 8 16 32 26 27 82 27 38 0 0 16 0 0 15 -103 28 39 23 29 45 -79 23 56 -28 24 50 121 29 36 -65 32 61 8 26 47 37 27 46 -6 48 64 50 51 94 15 37 48 -116 38 65 95 60 82 56 77 84 51 41 76 -111 43 76 76 92 48 0 0 15 -63 46 72 -97 50 62 0 0 16 0 0 15 35 52 84 -20 55 85 0 0 16 0 0 15 -4 65 27 0 0 15 -1 -18 17 16 29 26 27 90 27 39 0 0 16 0 0 15 -101 28 40 20 29 40 -94 23 57 -44 24 51 -102 29 37 -65 32 54 -12 25 51 14 27 47 -20 48 59 21 51 93 16 37 44 -114 38 67 82 55 83 -111 58 41 52 41 81 -113 43 76 -11 78 57 0 0 15 -39 46 64 -101 50 72 0 0 16 0 0 15 -3 51 88 -51 55 84 0 0 16 0 0 15 107 66 32 0 0 15 -1 -35 26 16 0 0 24 56 9 42 106 17 59 114 18 42 -87 6 -42 36 7 108 0 0 21 -12 0 119 -79 1 -87 0 0 15 0 0 16 -63 0 -41 -95 12 56 0 0 15 0 0 21 -35 10 75 -56 6 -36 -50 6 -31 123 7 -90 -88 6 -31 19 14 56 -89 14 39 -124 11 47 -17 11 50 1 20 45 -115 20 27 -106 15 60 -48 15 53 99 21 48 84 22 37 36 16 53 -1 6 -44 -1 -18 44 16 34 26 27 89 27 38 0 0 16 0 0 15 -91 28 39 86 29 40 -117 23 58 -69 24 53 115 29 34 113 32 47 -26 25 52 -23 26 42 -50 48 59 14 51 98 7 37 45 -113 38 62 18 55 89 -71 76 89 47 41 89 -121 43 76 -101 59 49 0 0 15 -64 46 64 -117 50 67 0 0 16 0 0 15 -4 51 91 -118 55 87 0 0 16 0 0 15 -23 65 31 0 0 15 -1 -18 53 16 39 26 27 81 27 37 0 0 16 0 0 15 -81 28 38 94 29 35 126 23 59 -84 24 56 82 29 34 -114 32 53 -39 25 53 -38 26 41 -59 48 63 -25 50 98 9 37 44 -120 38 62 -105 54 87 -57 76 95 59 41 79 -121 43 76 -108 59 49 -5 75 43 -82 46 59 -116 50 72 0 0 16 0 0 15 -38 51 88 112 55 87 0 0 16 0 0 15 0 0 16 0 0 15 -1 -18 62 16 54 26 27 1 26 39 0 0 16 0 0 15 -88 28 38 -65 29 33 117 23 59 -96 24 50 70 29 30 -104 32 53 -48 25 58 -64 26 42 -90 48 63 -75 50 112 2 37 41 -119 38 62 86 54 104 -52 76 101 51 41 81 -127 43 75 84 79 70 -85 75 37 -98 46 55 -118 50 72 0 0 16 0 0 15 -55 51 88 66 55 86 0 0 16 0 0 15 0 0 16 0 0 15 -1 -35 71 16 99 9 50 69 9 42 106 17 59 121 18 43 -56 6 -36 -92 6 -33 18 1 29 7 1 -76 -71 1 -100 0 0 15 0 0 16 -63 0 -42 -79 12 59 -62 12 54 -62 10 35 -39 10 59 -84 7 96 -47 6 -29 -79 7 80 -88 6 -31 22 14 56 -86 14 40 -119 11 47 -15 11 50 5 20 45 -104 20 27 -101 15 59 -43 15 53 101 21 48 85 22 39 48 16 53 0 0 15 -1 -18 89 16 39 26 26 107 27 40 0 0 16 0 0 15 -83 28 36 -62 29 34 92 23 60 126 24 51 46 29 37 -60 32 78 -70 25 55 -75 26 45 -127 48 65 118 50 89 4 37 37 -123 38 62 54 54 101 -25 76 78 47 41 72 120 43 66 -103 79 64 0 0 15 -102 46 59 24 50 81 0 0 16 0 0 15 -6 51 62 21 55 71 0 0 16 0 0 15 41 64 127 0 0 15 -1 -18 98 16 49 26 26 106 27 40 0 0 16 0 0 15 -78 28 34 -54 29 34 85 23 60 118 24 52 38 29 34 -49 32 68 -78 25 55 -92 26 43 107 48 58 70 50 111 16 37 37 -114 38 62 8 54 86 -59 64 92 43 41 69 114 43 64 -106 79 70 0 0 15 -107 46 59 71 50 76 0 0 16 0 0 15 -90 52 66 -9 54 79 0 0 16 0 0 15 -7 63 102 0 0 15 -1 -18 107 16 52 26 27 106 27 41 0 0 16 0 0 15 -71 28 34 -60 29 34 69 23 59 109 24 53 122 29 34 -51 32 61 -94 25 56 -103 26 46 80 48 58 56 50 113 6 37 40 -127 38 61 -26 53 80 -28 63 94 37 41 76 118 43 62 -92 90 71 0 0 15 -105 46 57 -124 50 69 0 0 16 0 0 15 -42 52 94 -32 54 76 0 0 16 0 0 15 0 0 21 0 0 15 -81 32 68 101 103 67 -96 -98 -59 83 -4 74 7 0 -32 4 0 0 -32 4 0 0 -1 -1 -1 -1 -1 -1 0 0 0 0 0 0 8 0 69 0 4 -46 0 1 0 0 -128 17 -83 -97 -64 -88 3 43 -64 -88 3 -1 1 -69 9 64 4 -66 0 0 -1 -35 125 16 100 9 49 0 0 15 110 17 59 123 18 41 1 7 -55 -89 6 -35 0 0 19 2 1 -84 -73 1 -111 0 0 15 0 0 16 -64 0 -42 0 0 28 -41 12 47 -66 10 34 -40 10 59 -95 7 79 -37 6 -35 -90 7 66 -64 6 -27 22 14 58 -79 14 41 -120 11 51 -10 11 51 12 20 47 -102 20 27 -106 15 58 -30 15 53 109 21 48 99 22 39 46 16 54 32 17 71 -1 -18 -122 16 61 26 27 125 27 49 0 0 16 0 0 15 -72 28 34 -51 29 34 41 23 61 82 24 53 -40 28 33 -53 32 43 -118 25 58 -120 26 49 41 48 58 38 50 100 18 37 39 -125 38 62 -55 53 91 -86 62 103 22 41 72 114 43 72 -76 90 87 0 0 15 -102 46 63 103 50 74 0 0 16 0 0 15 -86 52 107 114 54 61 0 0 16 0 0 15 0 0 19 0 0 15 -1 -18 -113 16 56 26 27 122 27 40 0 0 16 0 0 15 -76 28 35 -60 29 34 32 23 62 64 24 54 10 29 32 -6 28 36 119 25 57 121 26 47 27 48 57 13 50 90 21 37 37 -123 38 62 -86 53 90 61 62 94 24 41 75 116 43 66 -61 90 87 0 0 15 -101 46 57 86 50 76 0 0 16 0 0 15 -108 52 117 93 54 61 0 0 16 0 0 15 65 57 50 0 0 15 -1 -18 -104 16 56 26 27 111 27 36 0 0 16 0 0 15 -69 28 35 -59 29 34 19 23 60 53 24 53 -75 28 37 -38 28 34 110 25 58 107 26 50 5 48 62 4 50 78 18 37 40 -121 38 61 82 53 87 -38 61 89 24 41 80 107 43 73 -39 90 87 0 0 15 -100 46 55 100 50 69 0 0 17 0 0 15 122 52 111 15 54 54 -41 63 62 0 0 15 50 57 -98 0 0 15 -1 -35 -86 16 115 9 50 70 9 40 106 17 57 122 18 42 53 7 61 -88 6 -35 27 1 30 -90 0 -48 -79 1 111 0 0 15 0 0 16 -58 0 -42 -82 12 75 0 0 15 0 0 21 -37 10 57 -99 7 85 -37 6 -28 -88 7 66 -53 6 -26 20 14 56 -86 14 40 -110 11 51 -2 11 52 19 20 48 -103 20 27 -111 15 66 -37 15 51 108 21 48 102 22 39 53 16 53 0 0 15 -1 -18 -77 16 60 26 27 113 27 34 0 0 16 0 0 15 -72 28 36 -65 29 33 -5 22 59 28 24 53 -114 28 39 -83 32 38 75 25 61 71 26 51 -30 47 57 -56 49 82 17 37 40 -125 38 65 -31 52 91 56 61 84 19 41 72 102 43 75 10 91 95 0 0 15 -89 46 68 93 50 74 0 0 17 0 0 15 69 52 111 -128 53 56 -8 63 58 0 0 15 -36 56 -122 0 0 15 -1 -18 -68 16 62 26 27 110 27 34 0 0 16 0 0 15 -78 28 35 -66 29 32 -24 22 59 14 24 53 -39 28 31 -55 32 38 59 25 54 63 26 49 -45 47 57 -89 49 82 11 37 44 120 38 65 -55 52 102 -18 60 91 18 41 70 112 43 73 116 57 50 0 0 15 -90 46 70 89 50 71 0 0 17 0 0 15 47 52 108 -99 53 57 0 0 16 0 0 15 -66 56 -119 0 0 15 -1 -18 -59 16 69 26 27 126 27 34 0 0 16 0 0 15 -77 28 39 -58 29 32 -30 22 58 -3 23 53 -24 28 31 5 29 34 50 25 56 51 26 49 -56 47 57 -120 49 82 11 37 43 124 38 59 -79 52 101 -86 60 101 8 41 72 112 43 75 114 57 56 0 0 15 -97 46 74 85 50 69 0 0 17 0 0 15 23 52 113 -30 53 71 0 0 16 0 0 15 -94 56 -119 0 0 15 -1 -35 -41 16 102 9 61 63 9 43 0 0 30 -127 18 41 63 7 59 -87 6 -37 28 1 29 -83 0 -56 -75 1 74 31 10 21 0 0 16 -60 0 -43 0 0 27 -50 12 65 -54 10 35 0 0 15 -86 7 72 -38 6 -30 0 0 29 80 7 105 32 14 56 -83 14 40 -116 11 59 -9 11 51 22 20 47 -92 20 27 -91 15 59 -38 15 53 112 21 48 105 22 39 53 16 54 40 17 68 -1 -18 -32 16 65 26 27 111 27 34 -73 55 27 0 0 15 -77 28 40 -57 29 32 -50 22 60 -29 23 53 -101 28 33 -109 29 37 12 25 57 21 26 51 -79 47 59 87 49 107 17 37 43 -124 38 65 -116 52 109 32 60 103 11 41 75 101 43 70 58 91 104 0 0 15 -95 46 68 69 50 71 0 0 16 0 0 15 -38 51 109 109 53 59 0 0 16 0 0 15 89 56 -127 0 0 15 -1 -18 -23 16 67 26 28 108 27 34 -88 55 24 0 0 15 -70 28 40 -53 29 33 -56 22 61 -41 23 53 -16 28 33 -82 29 28 11 25 58 -1 25 53 -117 47 59 85 49 107 25 37 43 -128 38 61 113 52 90 -36 59 112 9 41 80 94 43 68 67 91 104 0 0 15 -97 46 72 59 50 71 0 0 16 0 0 15 -56 51 109 71 53 62 0 0 16 0 0 15 62 56 -121 0 0 15 -1 -18 -5 16 65 26 27 110 27 34 0 0 16 0 0 15 -70 28 42 -53 29 34 -72 22 59 -55 23 53 -65 28 34 -88 29 36 -13 24 55 -15 25 50 -115 47 59 69 49 97 22 37 47 120 38 65 52 52 108 -82 59 108 10 41 84 100 43 71 86 91 88 0 0 15 -101 46 70 52 50 71 0 0 16 0 0 15 -82 51 106 113 53 73 0 0 16 0 0 15 34 56 -125 0 0 15 -17 73 118 50 53 99 -96 -98 -59 83 86 76 7 0 -32 4 0 0 -32 4 0 0 -1 -1 -1 -1 -1 -1 0 0 0 0 0 0 8 0 69 0 4 -46 0 1 0 0 -128 17 -83 -97 -64 -88 3 43 -64 -88 3 -1 1 -69 9 64 4 -66 0 0 -1 -35 4 17 -24 8 70 -30 8 47 -123 6 -128 -74 7 72 0 0 28 -87 6 -37 0 0 19 -1 0 -79 0 0 28 31 10 24 0 0 16 -55 0 -43 -76 12 73 -28 12 48 -56 10 34 -29 10 71 -99 7 72 -35 6 -40 -17 10 65 124 7 77 23 14 54 0 0 15 0 0 27 2 12 57 27 20 50 -85 20 27 -93 15 59 -30 15 52 125 21 49 115 22 38 57 16 53 52 17 59 -1 -18 13 17 74 26 27 115 27 34 0 0 16 0 0 15 -69 28 42 -47 29 34 -91 22 59 -79 23 52 -125 29 36 -123 29 39 -40 24 56 -42 25 49 -119 47 56 29 49 97 20 37 45 124 38 55 -41 51 106 48 59 97 7 41 84 96 43 73 117 91 96 88 59 35 -105 46 70 3 50 71 0 0 16 0 0 15 122 51 105 69 53 78 0 0 16 0 0 15 -28 55 -123 0 0 15 -1 -18 22 17 75 26 27 123 27 34 0 0 16 0 0 15 -68 28 41 -57 29 31 -105 22 59 -94 23 53 112 29 36 -94 29 38 -61 24 54 -55 25 51 117 47 56 18 49 104 28 37 45 126 38 58 -66 51 114 21 55 77 18 41 80 92 43 71 -91 91 97 86 59 28 -107 46 67 -71 49 72 0 0 16 0 0 15 98 51 107 40 53 83 0 0 16 0 0 15 -62 55 -126 0 0 15 -1 -18 40 17 67 26 27 118 27 34 0 0 16 0 0 15 -66 28 42 -54 29 30 -117 22 59 -103 23 52 3 30 30 -113 29 38 -73 24 56 -76 25 51 88 47 63 -10 48 106 20 37 44 116 38 62 -70 51 101 5 55 80 8 41 79 89 43 68 -85 91 88 0 0 15 -113 46 72 -98 49 94 0 0 17 0 0 15 75 51 107 17 53 83 0 0 16 0 0 15 -98 55 -124 -88 63 46 -1 -35 49 17 0 0 28 58 9 44 31 7 -75 0 0 15 -94 9 78 -66 6 -33 30 1 29 -88 0 -53 0 0 26 57 10 27 0 0 16 -58 0 -42 -67 12 54 -33 12 49 0 0 21 0 0 15 0 0 28 47 7 -127 16 11 51 108 11 66 24 14 54 -79 14 38 -38 6 126 4 12 56 28 20 44 -87 20 27 -94 15 59 -26 15 51 -128 21 48 107 22 37 58 16 56 0 0 15 -1 -18 58 17 78 26 27 118 27 34 0 0 16 0 0 15 -56 28 39 -42 29 31 -127 22 59 -126 23 52 40 31 34 5 29 40 -105 24 58 -113 25 49 85 47 60 -71 48 75 29 37 48 125 38 68 111 51 100 122 54 75 9 41 75 78 43 73 9 65 93 0 0 15 -121 46 67 -23 49 75 -73 -33 -95 0 0 15 26 51 109 -39 52 83 0 0 16 0 0 15 105 55 -122 -72 63 54 -1 -18 67 17 76 26 27 125 27 34 0 0 16 0 0 15 -58 28 39 -44 29 31 118 22 58 121 23 53 31 31 37 77 29 38 -114 24 58 -125 25 50 68 47 60 -83 48 79 22 37 48 125 38 65 68 51 94 43 54 78 9 41 80 84 43 73 38 64 123 0 0 15 -123 46 67 -41 49 70 0 0 17 0 0 15 0 51 106 -65 52 82 0 0 16 0 0 15 68 55 -125 0 0 15 -1 -18 85 17 74 26 26 123 27 34 0 0 16 0 0 15 -57 28 38 -42 29 30 107 22 58 110 23 52 17 31 43 91 29 37 121 24 58 116 25 53 70 47 58 -89 48 79 19 37 49 120 38 65 20 51 86 -9 53 100 11 41 72 79 43 77 79 63 112 0 0 15 123 46 67 -42 49 73 0 0 17 0 0 15 -24 50 106 -99 52 87 0 0 16 0 0 15 43 55 -126 0 0 15 -1 -35 94 17 111 9 68 73 9 47 29 7 -123 -121 18 39 -67 9 62 -70 6 -35 0 0 19 5 1 -77 0 0 23 72 10 30 0 0 16 -59 0 -42 -74 12 53 0 0 15 -51 10 35 -31 10 70 85 13 55 -48 13 51 20 11 55 -127 11 64 107 7 75 -77 14 38 -107 6 -42 80 7 62 9 20 57 -66 20 27 -77 15 59 -24 15 52 -125 21 49 123 22 38 62 16 55 49 17 69 -1 -18 103 17 76 26 26 125 27 34 0 0 16 0 0 15 -54 28 40 -48 29 29 95 22 56 104 23 52 12 31 47 -34 28 39 100 24 58 84 25 53 41 47 60 110 48 95 22 37 52 126 38 68 -14 50 80 -21 53 85 19 41 77 74 43 75 89 62 121 108 102 76 83 46 82 -84 49 70 0 0 17 0 0 15 -71 50 107 104 52 87 0 0 16 0 0 15 -20 54 -127 -61 56 33 -1 -18 121 17 76 26 25 125 27 35 0 0 16 0 0 15 -42 28 40 -45 29 31 85 22 53 84 23 50 22 31 45 61 29 36 84 24 57 74 25 53 35 47 56 106 48 95 28 37 49 127 38 69 -9 50 96 -39 53 88 9 41 69 82 43 75 -11 61 123 126 102 127 74 46 77 -100 49 79 0 0 17 0 0 15 -91 50 110 83 52 84 0 0 16 -6 63 94 -45 54 -124 -80 56 62 -1 -18 -126 17 73 26 26 122 27 35 0 0 16 0 0 15 -47 28 40 -31 29 34 94 22 50 92 23 47 24 31 39 113 29 37 76 24 57 63 25 51 29 47 56 124 48 95 32 37 49 120 38 75 -48 50 -128 -59 53 82 16 41 64 65 43 66 -107 61 -127 -41 102 -25 77 46 77 125 49 74 0 0 16 0 0 15 -116 50 107 51 52 84 0 0 16 23 64 -109 -76 54 -122 -64 56 68 -81 32 68 101 103 67 -96 -98 -59 83 -10 77 7 0 -32 4 0 0 -32 4 0 0 -1 -1 -1 -1 -1 -1 0 0 0 0 0 0 8 0 69 0 4 -46 0 1 0 0 -128 17 -83 -97 -64 -88 3 43 -64 -88 3 -1 1 -69 9 64 4 -66 0 0 -1 -35 -117 17 0 0 29 68 9 51 27 7 -126 -124 18 41 -108 9 63 4 7 65 0 0 18 -83 0 -61 -7 6 31 67 10 31 11 1 21 -56 0 -43 0 0 28 -40 12 59 -50 10 35 -15 10 60 87 13 47 -50 13 56 22 11 53 125 11 72 -103 6 -33 -66 14 40 -97 6 -42 74 7 94 -19 19 70 -81 20 32 -85 15 58 -22 15 51 -120 21 48 124 22 37 68 16 53 57 17 58 -1 -18 -108 17 80 26 27 -127 27 36 0 0 16 0 0 15 -43 28 47 -27 29 47 92 22 53 82 23 46 88 31 30 35 29 33 50 24 54 35 25 51 14 47 58 95 48 99 28 37 51 -124 38 67 -84 50 85 98 53 98 20 41 75 70 43 75 -2 60 113 0 0 15 52 46 79 95 49 72 0 0 16 0 0 15 92 50 106 2 52 83 0 0 16 0 0 15 122 54 -120 -128 56 71 -1 -18 -90 17 82 26 27 127 27 36 0 0 16 0 0 15 -35 28 49 -30 29 47 95 22 47 90 23 45 98 31 28 -43 28 38 41 24 55 19 25 51 15 47 60 89 48 85 35 37 47 -128 38 62 125 50 97 74 53 95 14 41 79 61 43 77 -74 60 104 64 57 37 37 46 73 56 49 71 0 0 16 0 0 15 71 50 84 -21 51 80 0 0 16 0 0 15 92 54 -126 96 56 70 -1 -18 -81 17 80 26 27 -128 27 37 0 0 16 0 0 15 -41 28 47 -26 29 44 -115 22 45 75 23 43 104 31 27 -12 28 34 33 24 53 11 25 51 -3 46 58 66 48 87 36 37 48 -124 38 62 -123 50 97 66 53 95 7 41 80 62 43 77 124 60 104 68 57 34 14 46 64 43 49 74 0 0 16 0 0 15 0 0 23 -48 51 88 0 0 16 0 0 15 65 54 -127 53 56 64 -1 -35 -72 17 36 9 60 62 9 59 -51 6 -47 -111 18 41 -110 9 66 -33 9 65 15 1 23 -89 0 -44 31 7 90 9 10 30 -2 0 21 -51 0 -44 -65 12 73 -24 12 54 0 0 21 0 0 15 91 13 49 -56 13 64 20 11 53 -124 11 68 115 6 -41 -68 7 43 -98 6 -49 81 7 120 -53 19 58 -127 20 34 -78 15 59 -16 15 51 -107 21 48 -117 22 38 74 16 55 61 17 57 -1 -18 -63 17 83 26 27 -127 27 36 0 0 16 0 0 15 -50 28 40 -30 29 45 8 23 60 -27 24 39 123 31 27 82 29 27 31 24 50 -3 24 47 -7 46 58 37 48 100 45 37 48 -124 38 55 83 50 104 -11 52 88 4 41 69 49 43 73 -5 59 123 0 0 15 -12 45 62 -5 48 75 0 0 16 0 0 15 70 45 39 -102 51 87 0 0 16 0 0 15 2 54 -122 -13 55 66 -1 -18 -45 17 82 26 28 126 27 37 0 0 16 0 0 15 -46 28 36 -27 29 37 46 23 47 -24 24 40 119 31 28 -28 28 27 28 24 50 -5 24 45 -28 46 57 31 48 98 35 37 50 -125 38 59 77 50 108 -80 52 80 3 41 72 48 43 78 -59 59 126 0 0 15 -10 45 70 -20 48 66 0 0 16 0 0 15 53 45 40 -120 51 85 0 0 16 0 0 15 -26 53 -120 -52 55 63 -1 -18 -36 17 86 26 28 -118 27 35 0 0 16 0 0 15 -51 28 36 -42 29 34 42 23 47 -5 24 42 111 31 29 -39 28 29 28 24 47 -15 24 42 -30 46 60 24 48 87 39 37 51 -123 38 61 41 50 96 -74 52 69 10 41 77 51 43 78 -117 59 -127 0 0 15 -27 45 64 -24 48 71 0 0 16 0 0 15 63 45 34 106 51 84 0 0 16 0 0 15 -48 53 -123 -81 55 66 -1 -35 -27 17 45 9 53 37 9 56 -43 6 -31 -37 6 -114 -113 9 63 -34 9 64 0 0 18 28 1 -73 -91 6 -26 69 10 27 18 1 21 -55 0 -31 0 0 28 -25 12 49 -48 10 35 -24 10 72 86 13 50 0 0 15 0 0 28 -128 11 72 121 6 -45 -68 7 44 -92 6 -47 100 7 121 0 0 28 92 20 37 -71 15 59 -9 15 52 -114 21 45 -116 22 41 69 16 53 60 17 57 -1 -18 -9 17 89 26 31 -128 27 35 0 0 16 0 0 15 -42 28 34 -29 29 29 -34 23 48 -15 24 40 -128 31 28 109 29 30 31 24 39 -19 24 41 -85 45 41 33 48 94 45 37 50 -122 38 62 27 50 83 96 52 81 9 41 79 23 43 78 29 59 127 0 0 15 -55 45 64 -69 48 70 0 0 16 0 0 15 116 45 29 58 51 81 0 0 16 0 0 15 -108 53 -124 113 55 66 -1 -18 0 18 80 26 27 -120 27 34 102 63 26 0 0 15 -44 28 35 -32 29 28 -32 23 49 -6 24 47 121 31 28 -119 29 34 -47 25 44 4 25 34 105 45 35 46 48 94 46 37 50 -126 38 61 15 50 90 56 52 86 3 41 79 10 43 69 -20 58 -123 0 0 15 -61 45 62 -67 48 72 123 45 29 0 0 15 -88 49 104 23 51 81 0 0 16 0 0 15 120 53 -124 87 55 68 -1 -18 9 18 77 26 27 -115 27 35 -71 63 -115 0 0 15 -40 28 35 -19 29 28 85 22 52 -6 24 47 126 31 28 -111 29 25 -48 25 48 -118 26 34 106 45 39 44 48 87 41 37 50 -123 38 55 -31 49 95 18 52 83 7 41 84 1 43 78 -65 58 -124 0 0 15 -64 45 66 -92 48 65 94 45 35 0 0 15 118 49 74 9 51 81 0 0 16 0 0 15 103 53 -125 51 55 62 -17 73 118 50 53 99 -96 -98 -59 83 66 79 7 0 -32 4 0 0 -32 4 0 0 -1 -1 -1 -1 -1 -1 0 0 0 0 0 0 8 0 69 0 4 -46 0 1 0 0 -128 17 -83 -97 -64 -88 3 43 -64 -88 3 -1 1 -69 9 64 4 -66 0 0 -1 -35 18 18 0 0 27 0 0 15 -122 17 59 11 7 -71 -117 9 62 -48 9 64 0 0 16 -77 0 -61 -87 6 -26 62 10 28 35 1 22 -58 0 -29 -72 12 75 -18 12 47 0 0 21 -30 10 59 101 13 49 -46 13 61 28 11 60 -119 11 70 -128 6 -34 -75 7 42 -89 6 -41 67 7 118 -91 19 65 65 20 34 -73 15 59 -12 15 51 -124 21 60 -119 22 36 0 0 29 66 17 57 -1 -18 36 18 76 26 26 -94 27 33 0 0 26 0 0 15 -44 28 38 -17 29 27 96 22 54 -6 24 48 -126 31 27 36 34 27 -29 25 49 -121 26 35 101 45 49 35 48 89 43 37 48 -115 38 57 -115 49 91 -60 51 75 -2 40 85 -16 42 79 119 58 108 7 64 59 -81 45 65 -116 48 72 113 45 47 0 0 15 76 45 71 -40 50 85 0 0 16 0 0 15 39 53 -126 -6 54 59 -1 -18 45 18 93 26 27 -71 27 32 -92 -119 -1 0 0 15 -39 28 38 -15 29 27 83 22 57 -2 24 46 -117 31 27 81 34 34 -23 25 47 -124 26 34 114 45 33 33 48 87 53 37 50 -121 38 54 -120 49 98 -83 51 74 -3 40 83 -24 42 81 65 55 92 109 63 62 -87 45 74 124 48 67 -115 45 43 0 0 15 50 45 -20 -44 45 39 0 0 16 0 0 15 13 53 -123 -46 54 61 -1 -18 54 18 87 26 26 92 28 29 0 0 21 0 0 15 -28 28 38 -11 29 27 57 22 54 -12 24 41 -103 31 27 72 34 35 -33 25 53 106 26 38 -119 45 33 16 48 84 44 37 51 -116 38 58 125 49 110 -123 51 77 -1 40 90 -33 42 79 -15 54 100 -26 62 69 -100 45 73 103 48 69 -124 45 35 0 0 15 0 0 29 -80 50 80 0 0 16 0 0 15 -11 52 -126 -77 54 61 -1 -35 72 18 118 9 51 66 9 52 -116 17 62 7 7 61 -113 9 70 -37 9 64 95 8 22 48 1 -72 -88 6 -22 38 7 27 0 0 16 -63 0 -27 -58 12 54 0 0 15 -49 10 36 0 0 15 92 13 51 0 0 15 31 11 57 -116 11 71 -123 6 -41 -125 7 42 -88 6 -41 -123 6 -44 -111 19 62 51 20 33 -75 15 58 -5 15 51 87 21 63 84 22 53 85 16 60 66 17 57 -1 -18 81 18 92 26 26 -63 28 35 0 0 19 0 0 15 -18 28 47 -104 30 26 90 22 51 98 23 37 -81 32 37 89 34 36 -32 25 53 -126 25 35 -115 46 86 -43 47 97 61 37 44 -113 38 58 85 49 88 71 51 74 -3 40 87 -61 42 75 -105 54 87 28 62 63 -116 45 67 91 48 69 0 0 17 0 0 15 8 49 99 119 50 79 0 0 16 0 0 15 -64 52 -125 126 54 58 -1 -18 90 18 88 26 25 -58 28 36 0 0 18 0 0 15 -13 28 45 27 31 32 0 0 27 100 23 38 -75 32 36 90 34 36 -31 25 54 100 25 42 -108 46 67 -52 47 93 66 37 46 -113 38 58 72 49 95 15 51 78 -13 40 90 -56 42 79 111 54 107 -55 61 67 123 45 71 83 48 74 0 0 16 0 0 15 -21 48 96 101 50 76 0 0 16 0 0 15 -86 52 -128 98 54 64 -1 -18 99 18 105 26 25 -49 28 39 0 0 17 0 0 15 35 29 36 22 31 34 -50 23 46 62 23 45 -66 32 32 89 34 38 -37 25 54 100 25 44 0 0 21 -52 47 90 75 37 48 -111 38 60 88 49 98 -31 50 85 -24 40 86 -64 42 77 101 54 116 103 61 73 126 45 71 72 48 71 111 45 23 0 0 15 -35 48 103 73 50 79 0 0 16 0 0 15 -113 52 122 61 54 60 -1 -35 117 18 0 0 25 64 9 49 -119 17 59 10 7 74 -108 9 73 -41 9 60 90 8 26 45 1 -102 -88 6 -24 52 7 28 35 1 24 -77 0 -23 -66 12 54 -31 12 53 -56 10 34 -24 10 77 96 13 49 -51 13 66 26 11 53 -116 11 71 55 7 -72 55 7 53 -39 6 -54 -123 6 -45 120 19 59 27 20 34 -60 15 59 -3 15 51 36 21 58 36 22 47 83 16 58 71 17 59 -1 -18 126 18 126 26 24 -64 28 43 23 56 24 0 0 15 3 30 48 16 31 36 -43 23 47 57 23 43 -63 32 36 80 34 36 -53 25 52 91 25 39 78 45 41 -84 47 94 82 37 50 -100 38 74 42 49 74 9 51 76 -26 40 81 -70 42 70 55 54 98 -55 60 69 102 45 69 35 48 67 105 45 83 0 0 15 -48 48 101 -46 45 43 0 0 16 0 0 15 86 52 -124 17 54 60 -1 -18 -121 18 -121 26 24 -77 28 49 12 56 24 0 0 15 6 30 44 32 31 36 -65 23 59 122 23 34 -63 32 38 91 34 39 -47 25 46 -81 25 33 -31 45 45 -105 47 87 84 37 47 -96 38 63 27 49 92 -63 50 70 -33 40 76 -90 42 68 5 54 100 -123 60 75 90 45 71 25 48 66 0 0 26 0 0 15 -111 48 95 0 0 15 0 0 16 0 0 15 63 52 -127 -19 53 63 -1 -18 -112 18 -108 26 23 -53 28 46 0 0 16 0 0 15 8 30 44 31 31 40 -66 23 50 98 23 35 -87 32 35 99 34 38 -65 25 44 107 26 35 82 46 65 113 47 87 94 37 49 -97 38 67 30 49 90 -93 50 82 -34 40 72 -90 42 64 3 54 97 55 60 75 84 45 71 1 48 71 0 0 22 0 0 15 118 48 86 49 45 32 0 0 16 0 0 15 36 52 -122 -52 53 65 -81 32 68 101 103 67 -96 -98 -59 83 97 81 7 0 -32 4 0 0 -32 4 0 0 -1 -1 -1 -1 -1 -1 0 0 0 0 0 0 8 0 69 0 4 -46 0 1 0 0 -128 17 -83 -97 -64 -88 3 43 -64 -88 3 -1 1 -69 9 64 4 -66 0 0 -1 -35 -94 18 109 9 54 78 9 43 -115 17 59 11 7 83 -107 9 80 -33 9 63 121 8 34 18 1 94 -87 6 -23 13 7 37 0 0 16 -64 0 -27 0 0 28 -24 12 58 0 0 21 0 0 15 100 13 52 0 0 15 27 11 53 -122 11 71 -70 6 -81 84 7 64 -72 7 -101 125 6 -36 97 19 60 -3 19 36 -65 15 59 3 16 51 9 21 52 8 22 42 89 16 59 77 17 53 -1 -18 -85 18 102 27 27 -55 28 43 0 0 16 0 0 15 17 30 51 44 31 46 -72 23 48 120 22 39 -51 32 38 99 34 40 -88 25 44 -126 26 38 42 46 67 94 47 82 97 37 50 -92 38 74 33 49 90 -98 50 80 -52 40 71 -101 42 64 -58 53 94 -55 59 70 60 45 82 -9 47 73 0 0 21 0 0 15 72 48 94 49 45 32 0 0 16 0 0 15 -12 51 -126 -105 53 59 -1 -18 -76 18 127 27 29 -59 28 43 0 0 16 0 0 15 14 30 47 49 31 44 -83 23 48 127 22 37 -51 32 40 107 34 40 -105 25 47 126 26 40 40 46 58 69 47 80 96 37 54 -89 38 76 28 49 90 -128 50 84 -59 40 71 -116 42 66 -107 53 104 -112 59 70 46 45 90 -19 47 80 0 0 19 0 0 15 56 48 94 56 45 28 0 0 16 0 0 15 -43 51 -123 127 53 59 -1 -18 -58 18 127 27 34 -64 28 43 0 0 16 0 0 15 7 30 44 57 31 43 -82 23 52 105 22 38 -53 32 38 118 34 41 -113 25 49 -128 26 47 34 46 60 56 47 69 94 37 57 -84 38 71 11 49 104 123 50 97 -59 40 71 -125 42 76 53 53 111 89 59 70 58 45 74 -37 47 79 0 0 18 0 0 15 52 48 98 83 45 25 0 0 16 0 0 15 -63 51 -121 91 53 62 -1 -35 -49 18 0 0 26 0 0 15 -96 17 58 -59 6 -22 -110 9 88 -28 9 68 105 8 37 0 0 15 -87 6 -35 4 7 110 29 1 22 -68 0 -26 -59 12 71 -27 12 47 -41 10 36 -16 10 67 103 13 50 -46 13 71 29 11 53 -117 11 71 5 7 57 0 0 15 0 0 29 125 6 -26 72 19 63 -25 19 38 -56 15 57 0 16 50 -6 20 61 -25 21 47 95 16 58 0 0 15 -1 -18 -40 18 -103 27 27 -42 28 40 0 0 16 123 63 42 30 30 46 48 31 46 -82 23 53 103 22 39 -60 32 38 -108 34 46 127 25 49 -126 26 40 18 46 62 17 45 42 91 37 59 -103 38 73 17 49 99 54 50 81 -85 40 73 121 42 71 8 53 111 2 59 62 50 45 72 -78 47 74 0 0 17 91 45 23 -6 47 95 127 49 86 0 0 16 0 0 15 -108 51 -122 44 53 64 -1 -18 -31 18 103 27 28 -42 28 40 0 0 16 121 63 117 23 30 45 43 31 46 -87 23 53 120 22 47 -62 32 39 -116 34 47 126 25 49 127 26 39 17 46 62 9 45 44 90 37 57 -107 38 65 18 49 90 18 50 76 -95 40 84 109 42 74 -26 52 88 -22 58 59 37 45 76 -96 47 79 0 0 16 73 45 27 -40 47 95 88 49 72 0 0 16 0 0 15 -128 51 -125 6 53 64 -1 -18 -13 18 28 27 37 -31 28 37 0 0 16 -111 63 82 31 30 49 42 31 39 -95 23 57 110 22 47 -76 32 41 -117 34 51 114 25 49 119 26 41 4 46 62 7 45 58 83 37 58 -110 38 65 13 49 90 -33 49 85 -105 40 82 113 42 76 -44 52 113 -60 58 55 14 45 72 -109 47 121 0 0 16 60 45 28 -38 47 93 0 0 15 0 0 16 0 0 15 99 51 -125 -18 52 64 -1 -35 -4 18 119 9 53 72 9 42 -114 17 59 23 7 60 -107 9 98 -42 9 76 110 8 42 -101 8 83 -66 6 -24 -42 6 -51 53 1 22 -56 0 -31 0 0 28 0 0 15 -51 10 34 -23 10 66 99 13 50 0 0 15 31 11 53 -115 11 68 15 7 60 0 0 15 0 0 27 -119 6 -27 48 19 66 -59 19 39 -52 15 57 8 16 51 -42 20 60 -64 21 53 94 16 56 80 17 69 -1 -18 5 19 93 27 30 -22 28 43 0 0 16 0 0 15 37 30 46 48 31 42 -106 23 51 104 22 51 -80 32 38 -113 34 51 70 25 62 92 26 45 10 46 64 22 45 34 66 37 61 -125 38 70 5 49 94 -17 49 85 -111 40 79 92 42 72 -121 52 101 36 55 59 20 45 74 -71 47 77 0 0 16 69 45 29 -65 47 101 53 45 42 0 0 16 0 0 15 53 51 -126 -72 52 61 -1 -18 14 19 -93 27 27 -25 28 45 0 0 16 0 0 15 36 30 47 47 31 43 -117 23 52 107 22 49 -75 32 41 -115 34 52 67 25 59 88 26 43 2 46 68 79 45 35 65 37 58 125 38 69 6 49 92 -34 49 73 -114 40 79 81 42 76 90 52 92 -6 54 74 7 45 74 -105 47 90 0 0 16 73 45 25 -84 47 99 4 49 59 0 0 16 0 0 15 28 51 124 -100 52 63 -1 -18 32 19 -94 27 27 -22 28 40 0 0 16 0 0 15 35 30 47 55 31 44 -121 23 51 111 22 40 -84 32 42 -117 34 51 68 25 53 79 26 48 -12 45 68 71 46 83 58 37 58 109 38 68 -14 48 85 -31 49 73 -124 40 69 81 42 76 52 52 94 -59 54 82 -5 44 72 -124 47 72 0 0 16 100 45 23 -108 47 103 -17 48 78 0 0 16 0 0 15 8 51 121 -122 52 63 -17 73 118 50 53 99 -96 -98 -59 83 -95 82 7 0 -32 4 0 0 -32 4 0 0 -1 -1 -1 -1 -1 -1 0 0 0 0 0 0 8 0 69 0 4 -46 0 1 0 0 -128 17 -83 -97 -64 -88 3 43 -64 -88 3 -1 1 -69 9 64 4 -66 0 0 -1 -35 41 19 116 9 51 69 9 43 -107 17 59 0 0 15 -107 9 89 -20 9 87 91 8 50 0 0 15 -5 6 -44 -43 6 -49 -81 9 21 -56 6 -82 -58 12 76 -26 12 57 -48 10 35 -29 10 58 101 13 49 -35 13 61 0 0 27 -110 11 68 24 7 58 -56 14 31 -97 11 48 -113 6 -26 28 19 64 -84 19 38 -53 15 57 12 16 52 -64 20 64 -93 21 53 93 16 57 88 17 58 -1 -18 50 19 -67 27 27 -10 28 45 0 0 16 0 0 15 49 30 47 54 31 44 110 23 51 99 24 59 -74 32 45 -114 34 48 85 25 47 47 26 46 120 45 59 -52 46 57 37 37 59 99 38 66 -38 48 94 -84 49 77 111 40 78 55 42 72 25 52 78 -106 54 76 -23 44 74 86 47 76 0 0 16 0 0 15 108 47 105 -74 48 82 0 0 16 0 0 15 -50 50 -128 78 52 57 -1 -18 68 19 -66 27 27 -12 28 42 -21 53 74 0 0 15 45 30 57 61 31 44 0 0 27 92 24 52 -72 32 41 -109 34 46 80 25 53 39 26 49 23 45 54 2 45 50 35 37 56 100 38 63 -48 48 89 -105 49 89 97 40 72 46 42 79 11 52 81 -105 54 76 -36 44 77 66 47 73 0 0 16 0 0 15 100 47 104 -93 48 86 0 0 16 0 0 15 -67 50 -126 55 52 60 -1 -18 77 19 -67 27 26 -7 28 42 0 54 -91 0 0 15 52 30 49 61 31 42 82 23 57 69 24 52 -74 32 40 -101 34 45 45";

            stringstream sstr(data);
            while (sstr.good()) {
                int i = 0;
                sstr >> i;
                char c = (char)i;
                rawDataStream << c; 
            }

            PCAPProtocol pcap;
            pcap.setContainerListener(this);
            pcap.nextString(rawDataStream.str());

            TS_ASSERT(passed);
        }

};

#endif /*CORE_PCAPTESTSUITE_H_*/
