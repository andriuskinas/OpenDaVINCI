/**
 * boxparker - Sample application for realizing a box parking car.
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
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "BoxParker.h"

namespace automotive {
    namespace miniature {
        using namespace std;
        using namespace odcore::base;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace automotive;

        BoxParker::BoxParker(const int32_t &argc, char **argv) :   
            TimeTriggeredConferenceClientModule(argc, argv, "BoxParker"),
            m_foundGaps() {}

            BoxParker::~BoxParker() {}

            void BoxParker::setUp() {}
            void BoxParker::tearDown() {}

        vector<double> BoxParker::getFoundGaps() const {
            return m_foundGaps;
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode BoxParker::body() {
           
            int stageMoving = 0;
            

        while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data describing virtual sensor data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                // Create vehicle control data.
                VehicleControl vc;

                //Initalize the back and front sensors          
                //double BACK_SENSOR = sbd.getValueForKey_MapOfDistances(2);
                //double FRONT_SENSOR = sbd.getValueForKey_MapOfDistances(3); 

                //IR sensor
               // double RIGHT_SENSOR = sbd.getValueForKey_MapOfDistances(0);
                double DISTANCE_CAR = vd.getAbsTraveledPath(); 

                cout << "distance = " << DISTANCE_CAR << endl; 
       
        if ((DISTANCE_CAR >= 0) && (DISTANCE_CAR < 88)) { 
                    //Go forward.
                    cout << "MOVING FORWARD " << endl;
                    //vc.setSteeringWheelAngle(0);
                    vc.setSpeed(2);

                }
                if ((DISTANCE_CAR > 33) && (DISTANCE_CAR < 41.5)) {
                    // TURNING RIGHT UNTIL DISTANCE 41.5.
                    cout << "TURNING RIGHT UNTIL DISTANCE 41.5 " << endl;
                    vc.setSpeed(1);
vc.setSpeed(1);
                    vc.setSteeringWheelAngle(45);
                    stageMoving++;
                }

if (( DISTANCE_CAR>= 50) && (DISTANCE_CAR < 57))  {
                    // DRIVES FORWARD.
                    cout << "DRIVING" << endl;
                   // vc.setSpeed(1);
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(-100);
                    stageMoving++;
                }

if (( DISTANCE_CAR>= 58.5) && (DISTANCE_CAR < 66.5))  {
                    // TURNS INTO PARKING SLOT.
                    cout << "TURNING INTO PARKING LEFT" << endl;
                    vc.setSpeed(1);
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(80);
                    stageMoving++;
                }

if (( DISTANCE_CAR>= 72) && (DISTANCE_CAR < 80))  {
                    // DRIVES FORWARD.
                    cout << "DRIVING" << endl;
                   // vc.setSpeed(1);
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(100);
                    stageMoving++;
                }

 if ((DISTANCE_CAR >= 86.2) && (DISTANCE_CAR < 95.7))  {
                    // TURNS RIGHT.
                    cout << "TURNING RIGHT" << endl;
                    vc.setSpeed(-1);
vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(-30);
                    stageMoving++;
                }

if ((DISTANCE_CAR >= 95.7) && (DISTANCE_CAR < 97.7)) { 
                    //GO BACKWARD.
                    cout << "TURNING BACKWARDS" << endl;
                    //vc.setSteeringWheelAngle(0);
                    vc.setSpeed(-1);

                }

 if (DISTANCE_CAR >= 98) {
                    // End component.
                    cout << "PARKED" << endl;
                    vc.setSpeed(0);
                    stageMoving++;
                    //break;
                } 


               
                    
                // Create container for finally sending the data.
                Container c(vc);
                // Send container.
                getConference().send(c);
            }
            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    } // miniature
} // automotive
