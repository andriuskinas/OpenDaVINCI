/**
 * RPLidarGrabber - Sample application to encapsulate the interfacing with the rplidar.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <iostream>
#include <cstring>
#include <algorithm>

#include "RPLidarGrabber.h"
#include "rplidar.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif
#define DEG2RAD(x) ((x)*M_PI/180.)
namespace automotive {
    namespace miniature {

        using namespace odcore;
        using namespace odcore::data;
        using namespace odcore::wrapper;
        using namespace rp::standalone::rplidar;

        class RPLidarSerialInterface : public odcore::io::StringListener {
            void nextString(const std::string &s)
            {
                cout << "Received " << s.length() << " bytes containing '" << s << "'" << endl;
            }
        };

        RPLidarGrabber::RPLidarGrabber(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "RPLidarGrabber")
        {}

        RPLidarGrabber::~RPLidarGrabber() {
        }

        void RPLidarGrabber::setUp() {
            // This method will be call automatically _before_ running body().
            if (getFrequency() < 20) {
                cerr << endl << endl << "RPLidarGrabber: WARNING! Running proxy with a LOW frequency (consequence: data updates are too seldom and will influence your algorithms in a negative manner!) --> suggestions: --freq=20 or higher! Current frequency: " << getFrequency() << " Hz." << endl << endl << endl;
            }
        }

        void RPLidarGrabber::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

bool RPLidarGrabber::checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


                cout << "checking health" << endl;
    op_result = drv->getHealth(healthinfo);
    if (op_result==0) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

        int RPLidarGrabber::readOnce() {
            const char * opt_com_path = "/dev/ttyUSB0";
            _u32         opt_com_baudrate = 115200;
            
            // create the driver instance
            RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
            
            if (!drv) {
                cerr<< "insufficent memory, exit"<<endl;
                return -2;
            }

            // make connection...
            if (drv->connect(opt_com_path, opt_com_baudrate)!=0) {
                cerr<<"Error, cannot bind to the specified serial port "<<opt_com_path<<endl;
                return -2;
            }

            // check health...
            if (!checkRPLIDARHealth(drv)) {
                return -2;
            }


            // start scan...
            drv->startScan();

            // fetch result and print it out...
            {
                rplidar_response_measurement_node_t nodes[360*2];
                size_t   count = _countof(nodes);

                int result = drv->grabScanData(nodes, count);

                if (result==0) {
                    drv->ascendScanData(nodes, count);
                    for (int pos = 0; pos < (int)count ; ++pos) {
                        printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                            (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
                            (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                            nodes[pos].distance_q2/4.0f,
                            nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                    }
                }

            }

            // done!
            RPlidarDriver::DisposeDriver(drv);
            return 0;
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode RPLidarGrabber::body() {
            
            const char* SERIAL_PORT = "/dev/ttyUSB0";
            const uint32_t BAUD_RATE = 115200;
            
            // create the driver instance
            RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
            try {
                //std::shared_ptr<SerialPort> serial_port(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));

                
                cout << "created driver" << endl;
                if (!drv) {
                    cerr<< "insufficent memory, exit"<<endl;
                    exit(-2);
                }


                cout << "connecting" << endl;
                // make connection...
                if (drv->connect(SERIAL_PORT, BAUD_RATE)!=0) {
                    cerr<< "Error, cannot bind to the specified serial port "<<SERIAL_PORT<<endl;
                    return odcore::data::dmcp::ModuleExitCodeMessage::SERIOUS_ERROR;
                }
                cout << "connected" << endl;

                char cmd=0x52;
                std::string str=cmd+"";
                
                // This instance will handle any bytes that are received from our serial port.
                //RPLidarSerialInterface handler;
                //serial_port->setStringListener(&handler);

                // Start receiving bytes.
                //serial_port->start(); // WHEN THE PROGRAM REACHES THIS POINT, THE LIDAR STOPS.
                //serial_port->send(str);//0x52

//                const uint32_t ONE_SECOND = 1000 * 1000;
//                odcore::base::Thread::usleepFor(10 * ONE_SECOND);

                // Stop receiving bytes and unregister our handler.
                //serial_port->stop();
                //serial_port->setStringListener(NULL);
            }
            catch(string &exception) {
                cerr << "Serial port could not be created: " << exception << endl;
            }
//                cout << "while" << endl;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                
                cout << "RPLidarGrabber::body" << endl;
                
                rplidar_response_measurement_node_t nodes[360*2];
                size_t   count = _countof(nodes);

                int op_result = drv->grabScanData(nodes, count);

                if (op_result == 0) {
                    op_result = drv->ascendScanData(nodes, count);

//                    float angle_min = DEG2RAD(0.0f);
//                    float angle_max = DEG2RAD(359.0f);
                    if (op_result == RESULT_OK) {
                        if (true) {
                            const int angle_compensate_nodes_count = 360;
                            const int angle_compensate_multiple = 1;
                            int angle_compensate_offset = 0;
                            rplidar_response_measurement_node_t angle_compensate_nodes[angle_compensate_nodes_count];
                            memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_t));
                            size_t i = 0, j = 0;
                            for( ; i < count; i++ ) {
                                if (nodes[i].distance_q2 != 0) {
                                    float angle = (float)((nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                                    int angle_value = (int)(angle * angle_compensate_multiple);
                                    if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                                    for (j = 0; j < angle_compensate_multiple; j++) {
                                        angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
                                    }
                                }
                            }
          
                        cerr<<"SCAN OK"<<endl;
                        } 
//                        else {
//                            int start_node = 0, end_node = 0;
//                            int i = 0;
//                            // find the first valid node and last valid node
//                            while (nodes[i++].distance_q2 == 0);
//                            start_node = i-1;
//                            i = count -1;
//                            while (nodes[i--].distance_q2 == 0);
//                            end_node = i+1;

//                            angle_min = DEG2RAD((float)(nodes[start_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
//                            angle_max = DEG2RAD((float)(nodes[end_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

//                        cerr<<"SCAN OK (unreachable)"<<endl;
//                       }
                    } else {
                        cerr<<"Ascend Scan Data failed"<<endl;
                    }
//                        Container c(s);
//                        // Time stamp data before storing.
//                        c.setReceivedTimeStamp(TimeStamp());
//                        // Share data.
//                        getConference().send(c);
                    }else {
                        cerr<<"Grab scan data failed"<<endl;
                    }
            }
            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature

