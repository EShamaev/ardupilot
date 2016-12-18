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
 * AP_Taxi.cpp
 *      Author: Eugene Shamaev
 */

#include "AP_Taxi.h"
#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_Taxi::var_info[] = {

    // @Param: MIN_THR
    // @DisplayName: Minimum Throttle in taxi mode
    // @Description: Overrides control of throttle to a new minimum value in taxi mode
    // @Range: 0 100
    // @Units: %
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("MIN_THR", 1, AP_Taxi, _min_thr, 0.0f),

    // @Param: MAX_THR
    // @DisplayName: Maximum Throttle in taxi mode
    // @Description: Overrides control of throttle to a new maximum value in taxi mode
    // @Range: 0 100
    // @Units: %
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("MAX_THR", 2, AP_Taxi, _max_thr, 50),

    // @Param: MAX_THR
    // @DisplayName: Ground speed in taxi mode
    // @Description: Default speed that is maintained during taxi
    // @Range: 0 150
    // @Units: cm/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SPD", 2, AP_Taxi, _g_speed, 150),

    AP_GROUPEND
};
