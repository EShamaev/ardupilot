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

//
//  UAVCAN GPS driver
//
#include "AP_GPS_UAVCAN.h"
#include <stdint.h>

#if HAL_WITH_UAVCAN
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

AP_GPS_UAVCAN::AP_GPS_UAVCAN(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    _new_data = false;
}

// For each instance we need to deregister from AP_UAVCAN class
AP_GPS_UAVCAN::~AP_GPS_UAVCAN()
{
    if (hal.can_mgr != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            ap_uavcan->remove_gps_listener(this);

            if (AP_BoardConfig::get_can_debug() >= 2) {
                hal.console->printf("AP_GPS_UAVCAN destructed\n\r");
            }
        }
    }
}

// Consume new data and mark it received
bool AP_GPS_UAVCAN::read(void)
{
    if (_new_data) {
        _new_data = false;
        return true;
    }

    return false;
}

void AP_GPS_UAVCAN::handle_gnss_msg(const AP_GPS::GPS_State &msg)
{
    state = msg;
    _new_data = true;
}

#endif // HAL_WITH_UAVCAN
