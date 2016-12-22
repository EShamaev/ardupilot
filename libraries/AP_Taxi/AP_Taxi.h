/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * AP_Taxi.h
 *
 *      Author: Eugene
 */

#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Common/AP_Common.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AC_PID/AC_PID.h>

/// @class  AP_Taxi
/// @brief  Class managing ArduPlane taxi mode on the ground
class AP_Taxi
{
public:
    // constructor
    AP_Taxi(AP_Mission &_mission, AP_AHRS &_ahrs, AP_SpdHgtControl *_SpdHgt_Controller, AP_Navigation *_nav_controller):
            mission(_mission)
            ,ahrs(_ahrs)
            ,SpdHgt_Controller(_SpdHgt_Controller)
            ,nav_controller(_nav_controller)
    {
        AP_Param::setup_object_defaults(this, var_info);

        //(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt)
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Mission &mission;
    AP_AHRS &ahrs;
    AP_SpdHgtControl *SpdHgt_Controller;
    AP_Navigation *nav_controller;

    // Parameters
    AP_Float _min_thr;
    AP_Float _max_thr;
    AP_Float _g_speed;

    AC_PID _pid{1, 0.1, 0.01, 50, 2, 0.02};
};


