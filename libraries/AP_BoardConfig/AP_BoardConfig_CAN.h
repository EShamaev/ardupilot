#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_BoardConfig.h"
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <sys/ioctl.h>

#if HAL_WITH_UAVCAN
#define UAVCAN_PROTOCOL_ENABLE  1
#endif

class AP_BoardConfig_CAN {
public:
    // constructor
    AP_BoardConfig_CAN(void)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    void init(void);

    static const struct AP_Param::GroupInfo var_info[];

#if HAL_WITH_UAVCAN
    class CAN_var_info {
        friend class AP_BoardConfig_CAN;

    public:
        CAN_var_info()
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _can_enable;
        AP_Int8 _can_debug;
        AP_Int32 _can_bitrate;
    };

    class CAN_protocol_var_info {
        friend class AP_BoardConfig_CAN;

    public:
        CAN_protocol_var_info() : _uavcan(nullptr)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }
        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _protocol_enable;

        AP_UAVCAN* _uavcan;
    };
#endif //HAL_WITH_UAVCAN

    static int8_t get_can_enable(uint8_t i)
    {
#if HAL_WITH_UAVCAN
        if (i < MAX_NUMBER_OF_CAN_INTERFACES) {
            return _st_can_enable[i];
        }
#endif //HAL_WITH_UAVCAN
        return 0;
    }

    // returns number of enabled CAN interfaces
    static int8_t get_can_enable(void)
    {
#if HAL_WITH_UAVCAN
        uint8_t ret = 0;

        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_INTERFACES; i++) {
            if (get_can_enable(i)) {
                ret++;
            }
        }

        return ret;
#endif //HAL_WITH_UAVCAN
        return 0;
    }

    static int8_t get_can_debug(uint8_t i)
    {
#if HAL_WITH_UAVCAN
        if (i < MAX_NUMBER_OF_CAN_INTERFACES) {
            return _st_can_debug[i];
        }
#endif //HAL_WITH_UAVCAN
        return 0;
    }

    // return maximum level of debug
    static int8_t get_can_debug(void)
    {
        uint8_t ret = 0;
#if HAL_WITH_UAVCAN
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_INTERFACES; i++) {
            uint8_t dbg = get_can_debug(i);
            ret = (dbg > ret) ? dbg : ret;
        }
#endif //HAL_WITH_UAVCAN
        return ret;
    }

#if HAL_WITH_UAVCAN
    CAN_var_info _var_info_can[MAX_NUMBER_OF_CAN_INTERFACES];
    CAN_protocol_var_info _var_info_can_protocol[MAX_NUMBER_OF_CAN_DRIVERS];

    static int8_t _st_can_enable[MAX_NUMBER_OF_CAN_INTERFACES];
    static int8_t _st_can_debug[MAX_NUMBER_OF_CAN_INTERFACES];
#endif //HAL_WITH_UAVCAN

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    void px4_setup_canbus(void);
#endif // HAL_BOARD_PX4 || HAL_BOARD_VRBRAIN
};
