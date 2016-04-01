/**
 * rplidar_grabber - Sample application to encapsulate the interfacing with the rplidar.
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

#ifndef RPLIDAR_EX_GRABBER_H_
#define RPLIDAR_EX_GRABBER_H_

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"
#include "rplidar.h"

namespace automotive {
    namespace miniature {

        /**
         * This class wraps the software/hardware interface board.
         */
        class RPLidarGrabber : public odcore::base::module::TimeTriggeredConferenceClientModule {
            private:
                /**
                 * "Forbidden" copy constructor. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the copy constructor.
                 *
                 * @param obj Reference to an object of this class.
                 */
                RPLidarGrabber(const RPLidarGrabber &/*obj*/);

                /**
                 * "Forbidden" assignment operator. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the assignment operator.
                 *
                 * @param obj Reference to an object of this class.
                 * @return Reference to this instance.
                 */
                RPLidarGrabber& operator=(const RPLidarGrabber &/*obj*/);

            public:
                /**
                 * Constructor.
                 *
                 * @param argc Number of command line arguments.
                 * @param argv Command line arguments.
                 */
                RPLidarGrabber(const int32_t &argc, char **argv);

                virtual ~RPLidarGrabber();

                odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();
int readOnce();
bool checkRPLIDARHealth(rp::standalone::rplidar::RPlidarDriver*);
            private:
                virtual void setUp();

                virtual void tearDown();

            private:
        };

    }
} // automotive::miniature

#endif /*RPLIDAR_EX_GRABBER_H_*/
