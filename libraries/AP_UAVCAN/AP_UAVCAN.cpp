/*
 * AP_UAVCAN.cpp
 *
 *      Author: Eugene Shamaev
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_UAVCAN.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

// Zubax GPS and other GPS, baro, magnetic sensors
#include <uavcan/equipment/gnss/Fix.hpp>
#include <uavcan/equipment/gnss/Auxiliary.hpp>
#include <uavcan/equipment/ahrs/MagneticFieldStrength.hpp>
#include <uavcan/equipment/ahrs/Solution.hpp>
#include <uavcan/equipment/air_data/StaticPressure.hpp>
#include <uavcan/equipment/air_data/StaticTemperature.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Command.hpp>
#include <uavcan/equipment/actuator/Status.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>

extern const AP_HAL::HAL& hal;

#define debug_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { hal.console->printf(fmt, ##args); }} while (0)

// Translation of all messages from UAVCAN structures into AP structures is done
// in AP_UAVCAN and not in corresponding drivers.
// The overhead of including definitions of DSDL is very high and it is best to
// concentrate in one place.

// TODO: temperature can come not only from baro. There should be separation on node ID
// to check where it belongs to. If it is not baro that is the source, separate layer
// of listeners/nodes should be added.

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_UAVCAN::var_info[] = {
    // @Param: NODE
    // @DisplayName: UAVCAN node that is used for this network
    // @Description: UAVCAN node should be set implicitly
    // @Range: 1 250
    // @User: Advanced
    AP_GROUPINFO("NODE", 1, AP_UAVCAN, _uavcan_node, 10),

    // @Param: SRV_BM
    // @DisplayName: RC Out channels to be transmitted as servo over UAVCAN
    // @Description: Bitmask with one set for channel to be transmitted as a servo command over UAVCAN
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15
    // @User: Advanced
    AP_GROUPINFO("SRV_BM", 2, AP_UAVCAN, _servo_bm, 255),

    // @Param: ESC_BM
    // @DisplayName: RC Out channels to be transmitted as ESC over UAVCAN
    // @Description: Bitmask with one set for channel to be transmitted as a ESC command over UAVCAN
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16
    // @User: Advanced
    AP_GROUPINFO("ESC_BM", 3, AP_UAVCAN, _esc_bm, 255),

    // @Param: BRCST_BM
    // @DisplayName: Bitmask to set broadcasted messages
    // @Description: Bitmask with one set message to be broadcasted over UAVCAN. Reboot needed after change.
    // @Bitmask: 0: Position 1, 1: Attitude 2
    // @User: Advanced
    AP_GROUPINFO("BR_BM", 4, AP_UAVCAN, _broadcast_bm, 0),

    // @Param: NODE
    // @DisplayName: UAVCAN GNSS Fix broadcast rate
    // @Description: UAVCAN GNSS Fix broadcast rate in times per second. It is active only if enabled with bitmask.
    // @Range: 1 20
    // @User: Advanced
    AP_GROUPINFO("FIX_R", 5, AP_UAVCAN, _broadcast_fix_rate, 10),

    // @Param: NODE
    // @DisplayName: UAVCAN attitude broadcast rate
    // @Description: UAVCAN attitude broadcast rate in times per second. It is active only if enabled with bitmask.
    // @Range: 1 20
    // @User: Advanced
    AP_GROUPINFO("ATT_R", 6, AP_UAVCAN, _broadcast_att_rate, 10),

    AP_GROUPEND
};

// publisher interfaces
static uavcan::Publisher<uavcan::equipment::gnss::Fix> *fix_out_array[MAX_NUMBER_OF_CAN_DRIVERS];
static uavcan::Publisher<uavcan::equipment::ahrs::Solution> *attitude_out_array[MAX_NUMBER_OF_CAN_DRIVERS];

static uavcan::equipment::gnss::Fix _fix_state;
static uavcan::equipment::ahrs::Solution _att_state;

void AP_UAVCAN::UAVCAN_AHRS_update(const AP_AHRS_NavEKF &ahrs)
{
    if (fix_out_array[_uavcan_i] != nullptr) {
        bool sem_ret = _fix_out_sem->take(1);

        if (sem_ret) {
            const AP_GPS cgps = ahrs.get_gps();
            Location loc;
            Vector3f vel_NED;

            if (ahrs.healthy()) {
                ahrs.get_location(loc);
                ahrs.get_velocity_NED(vel_NED);

                float velVar, posVar, hgtVar;
                Vector3f magVar;
                float tasVar;
                Vector2f offset;
                ahrs.get_variances(velVar, posVar, hgtVar, magVar, tasVar, offset);

                // Manual resize to diagonal form
                _fix_state.position_covariance.resize(3);
                _fix_state.position_covariance[0] = posVar;
                _fix_state.position_covariance[1] = posVar;
                _fix_state.position_covariance[2] = hgtVar;

                // Manual resize to diagonal form
                _fix_state.velocity_covariance.resize(3);
                _fix_state.velocity_covariance[0] = velVar;
                _fix_state.velocity_covariance[1] = velVar;
                _fix_state.velocity_covariance[2] = velVar;
            } else {
                loc = cgps.location();
                vel_NED = cgps.velocity();

                float vel_acc, hacc, vert_acc;
                cgps.horizontal_accuracy(hacc);
                cgps.vertical_accuracy(vert_acc);
                cgps.speed_accuracy(vel_acc);

                // Manual resize to diagonal form
                _fix_state.position_covariance.resize(3);
                _fix_state.position_covariance[0] = hacc * hacc;
                _fix_state.position_covariance[1] = _fix_state.position_covariance[0];
                _fix_state.position_covariance[2] = vert_acc * vert_acc;

                // Manual resize to diagonal form
                _fix_state.velocity_covariance.resize(3);
                _fix_state.velocity_covariance[0] = vel_acc * vel_acc;
                _fix_state.velocity_covariance[1] = _fix_state.velocity_covariance[0];
                _fix_state.velocity_covariance[2] = _fix_state.velocity_covariance[0];
            }

            _fix_state.height_msl_mm = loc.alt * 10;
            _fix_state.latitude_deg_1e8 = ((uint64_t) loc.lat) * 10;
            _fix_state.longitude_deg_1e8 = ((uint64_t) loc.lng) * 10;

            // Not saved in AP
            //_fix_state.height_ellipsoid_mm

            _fix_state.ned_velocity[0] = vel_NED.x;
            _fix_state.ned_velocity[1] = vel_NED.y;
            _fix_state.ned_velocity[2] = vel_NED.z;

            _fix_state.sats_used = cgps.num_sats();
            _fix_state.pdop = cgps.get_hdop();

            switch(cgps.status()) {
                case AP_GPS::GPS_Status::NO_GPS:
                    _fix_state.status = uavcan::equipment::gnss::Fix::STATUS_NO_FIX;
                    break;
                case AP_GPS::GPS_Status::NO_FIX:
                    _fix_state.status = uavcan::equipment::gnss::Fix::STATUS_NO_FIX;
                    break;
                case AP_GPS::GPS_Status::GPS_OK_FIX_2D:
                    _fix_state.status = uavcan::equipment::gnss::Fix::STATUS_2D_FIX;
                    break;
                case AP_GPS::GPS_Status::GPS_OK_FIX_3D:
                case AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS:
                case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT:
                case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED:
                    _fix_state.status = uavcan::equipment::gnss::Fix::STATUS_3D_FIX;
                    break;
            }

            _fix_state.gnss_time_standard = uavcan::equipment::gnss::Fix::GNSS_TIME_STANDARD_UTC;
            uavcan::Timestamp ts;
            ts.usec = 1000ULL * ((cgps.time_week() * AP_MSEC_PER_WEEK + cgps.time_week_ms()) + UNIX_OFFSET_MSEC);
            _fix_state.gnss_timestamp = ts;

            _fix_state.num_leap_seconds = 0;
            ts.usec = 0;
            _fix_state.timestamp = ts;

            _fix_out_sem->give();
        }
    }

    if (attitude_out_array[_uavcan_i] != nullptr) {
        bool sem_ret = _att_out_sem->take(1);

        if (sem_ret) {
            uavcan::Timestamp ts;
            ts.usec = AP_HAL::micros64();
            _att_state.timestamp = ts;

            Quaternion qt;
            Matrix3f rot = ahrs.get_rotation_body_to_ned();
            qt.from_rotation_matrix(rot);
            _att_state.orientation_xyzw[0] = qt.q1;
            _att_state.orientation_xyzw[1] = qt.q2;
            _att_state.orientation_xyzw[2] = qt.q3;
            _att_state.orientation_xyzw[3] = qt.q4;

            // TODO: extract from EKF
            //_att_state.orientation_covariance

            Vector3f av = ahrs.get_gyro();
            _att_state.angular_velocity[0] = av.x;
            _att_state.angular_velocity[1] = av.y;
            _att_state.angular_velocity[2] = av.z;

            // TODO: extract from EKF
            //_att_state.angular_velocity_covariance

            Vector3f la = ahrs.get_accel_ef();
            _att_state.linear_acceleration[0] = la.x;
            _att_state.linear_acceleration[1] = la.y;
            _att_state.linear_acceleration[2] = la.z;

            // TODO: extract from EKF
            //_att_state.linear_acceleration_covariance

            _att_out_sem->give();
        }
    }
}

static void gnss_fix_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg, uint8_t mgr)
{
    if (hal.can_mgr[mgr] != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr[mgr]->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            AP_GPS::GPS_State *state = ap_uavcan->find_gps_node(msg.getSrcNodeID().get());

            if (state != nullptr) {
                bool process = false;

                if (msg.status == uavcan::equipment::gnss::Fix::STATUS_NO_FIX) {
                    state->status = AP_GPS::GPS_Status::NO_FIX;
                } else {
                    if (msg.status == uavcan::equipment::gnss::Fix::STATUS_TIME_ONLY) {
                        state->status = AP_GPS::GPS_Status::NO_FIX;
                    } else if (msg.status == uavcan::equipment::gnss::Fix::STATUS_2D_FIX) {
                        state->status = AP_GPS::GPS_Status::GPS_OK_FIX_2D;
                        process = true;
                    } else if (msg.status == uavcan::equipment::gnss::Fix::STATUS_3D_FIX) {
                        state->status = AP_GPS::GPS_Status::GPS_OK_FIX_3D;
                        process = true;
                    }

                    if (msg.gnss_time_standard == uavcan::equipment::gnss::Fix::GNSS_TIME_STANDARD_UTC) {
                        uint64_t epoch_ms = uavcan::UtcTime(msg.gnss_timestamp).toUSec();
                        epoch_ms /= 1000;
                        uint64_t gps_ms = epoch_ms - UNIX_OFFSET_MSEC;
                        state->time_week = (uint16_t)(gps_ms / AP_MSEC_PER_WEEK);
                        state->time_week_ms = (uint32_t)(gps_ms - (state->time_week) * AP_MSEC_PER_WEEK);
                    }
                }

                if (process) {
                    Location loc = { };
                    loc.lat = msg.latitude_deg_1e8 / 10;
                    loc.lng = msg.longitude_deg_1e8 / 10;
                    loc.alt = msg.height_msl_mm / 10;
                    state->location = loc;
                    state->location.options = 0;

                    if (!uavcan::isNaN(msg.ned_velocity[0])) {
                        Vector3f vel(msg.ned_velocity[0], msg.ned_velocity[1], msg.ned_velocity[2]);
                        state->velocity = vel;
                        state->ground_speed = norm(vel.x, vel.y);
                        state->ground_course = wrap_360(degrees(atan2f(vel.y, vel.x)));
                        state->have_vertical_velocity = true;
                    } else {
                        state->have_vertical_velocity = false;
                    }

                    float pos_cov[9];
                    msg.position_covariance.unpackSquareMatrix(pos_cov);
                    if (!uavcan::isNaN(pos_cov[8])) {
                        if (pos_cov[8] > 0) {
                            state->vertical_accuracy = sqrtf(pos_cov[8]);
                            state->have_vertical_accuracy = true;
                        } else {
                            state->have_vertical_accuracy = false;
                        }
                    } else {
                        state->have_vertical_accuracy = false;
                    }

                    const float horizontal_pos_variance = MAX(pos_cov[0], pos_cov[4]);
                    if (!uavcan::isNaN(horizontal_pos_variance)) {
                        if (horizontal_pos_variance > 0) {
                            state->horizontal_accuracy = sqrtf(horizontal_pos_variance);
                            state->have_horizontal_accuracy = true;
                        } else {
                            state->have_horizontal_accuracy = false;
                        }
                    } else {
                        state->have_horizontal_accuracy = false;
                    }

                    float vel_cov[9];
                    msg.velocity_covariance.unpackSquareMatrix(vel_cov);
                    if (!uavcan::isNaN(vel_cov[0])) {
                        state->speed_accuracy = sqrtf((vel_cov[0] + vel_cov[4] + vel_cov[8]) / 3.0);
                        state->have_speed_accuracy = true;
                    } else {
                        state->have_speed_accuracy = false;
                    }

                    state->num_sats = msg.sats_used;
                } else {
                    state->have_vertical_velocity = false;
                    state->have_vertical_accuracy = false;
                    state->have_horizontal_accuracy = false;
                    state->have_speed_accuracy = false;
                    state->num_sats = 0;
                }

                state->last_gps_time_ms = AP_HAL::millis();

                // after all is filled, update all listeners with new data
                ap_uavcan->update_gps_state(msg.getSrcNodeID().get());
            }
        }
    }
}

static void gnss_fix_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg)
{   gnss_fix_cb(msg, 0); }
static void gnss_fix_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg)
{   gnss_fix_cb(msg, 1); }
static void (*gnss_fix_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Fix>& msg)
        = { gnss_fix_cb0, gnss_fix_cb1 };

static void gnss_aux_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg, uint8_t mgr)
{
    if (hal.can_mgr[mgr] != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr[mgr]->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            AP_GPS::GPS_State *state = ap_uavcan->find_gps_node(msg.getSrcNodeID().get());

            if (state != nullptr) {
                if (!uavcan::isNaN(msg.hdop)) {
                    state->hdop = msg.hdop * 100.0;
                }

                if (!uavcan::isNaN(msg.vdop)) {
                    state->vdop = msg.vdop * 100.0;
                }
            }
        }
    }
}

static void gnss_aux_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg)
{   gnss_aux_cb(msg, 0); }
static void gnss_aux_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg)
{   gnss_aux_cb(msg, 1); }
static void (*gnss_aux_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::gnss::Auxiliary>& msg)
        = { gnss_aux_cb0, gnss_aux_cb1 };

static void magnetic_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg, uint8_t mgr)
{
    if (hal.can_mgr[mgr] != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr[mgr]->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            AP_UAVCAN::Mag_Info *state = ap_uavcan->find_mag_node(msg.getSrcNodeID().get());
            if (state != nullptr) {
                state->mag_vector[0] = msg.magnetic_field_ga[0];
                state->mag_vector[1] = msg.magnetic_field_ga[1];
                state->mag_vector[2] = msg.magnetic_field_ga[2];

                // after all is filled, update all listeners with new data
                ap_uavcan->update_mag_state(msg.getSrcNodeID().get());
            }
        }
    }
}

static void magnetic_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg)
{   magnetic_cb(msg, 0); }
static void magnetic_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg)
{   magnetic_cb(msg, 1); }
static void (*magnetic_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength>& msg)
        = { magnetic_cb0, magnetic_cb1 };

static void air_data_sp_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg, uint8_t mgr)
{
    if (hal.can_mgr[mgr] != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr[mgr]->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            uint8_t nodeid = msg.getSrcNodeID().get();

            AP_UAVCAN::Baro_Info *state = ap_uavcan->find_baro_node(nodeid);

            if (ap_uavcan->get_baro()->register_uavcan_baro(mgr, nodeid))
            {
                if (state != nullptr) {
                    state->pressure = msg.static_pressure;
                    state->pressure_variance = msg.static_pressure_variance;

                    // after all is filled, update all listeners with new data
                    ap_uavcan->update_baro_state(msg.getSrcNodeID().get());
                }
            }
        }
    }
}

static void air_data_sp_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg)
{   air_data_sp_cb(msg, 0); }
static void air_data_sp_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg)
{   air_data_sp_cb(msg, 1); }
static void (*air_data_sp_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticPressure>& msg)
        = { air_data_sp_cb0, air_data_sp_cb1 };

// Temperature is not main parameter so do not update listeners when it is received
static void air_data_st_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg, uint8_t mgr)
{
    if (hal.can_mgr[mgr] != nullptr) {
        AP_UAVCAN *ap_uavcan = hal.can_mgr[mgr]->get_UAVCAN();
        if (ap_uavcan != nullptr) {
            uint8_t nodeid = msg.getSrcNodeID().get();

            AP_UAVCAN::Baro_Info *state = ap_uavcan->find_baro_node(nodeid);

            if (ap_uavcan->get_baro()->register_uavcan_baro(mgr, nodeid))
            {
                if (state != nullptr) {
                    state->temperature = msg.static_temperature;
                    state->temperature_variance = msg.static_temperature_variance;
                }
            }
        }
    }
}

static void air_data_st_cb0(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg)
{   air_data_st_cb(msg, 0); }
static void air_data_st_cb1(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg)
{   air_data_st_cb(msg, 1); }
static void (*air_data_st_cb_arr[2])(const uavcan::ReceivedDataStructure<uavcan::equipment::air_data::StaticTemperature>& msg)
        = { air_data_st_cb0, air_data_st_cb1 };

// publisher interfaces
static uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>* act_out_array[MAX_NUMBER_OF_CAN_DRIVERS];
static uavcan::Publisher<uavcan::equipment::esc::RawCommand>* esc_raw[MAX_NUMBER_OF_CAN_DRIVERS];

AP_UAVCAN::AP_UAVCAN() :
    _initialized(false), _rco_armed(false), _rco_safety(false), _rc_out_sem(nullptr), _fix_out_sem(nullptr), _att_out_sem(nullptr), _node_allocator(UAVCAN_NODE_POOL_SIZE, UAVCAN_NODE_POOL_SIZE)
{
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER; i++) {
        _rco_conf[i].active = false;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        _gps_nodes[i] = UINT8_MAX;
        _gps_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        _baro_nodes[i] = UINT8_MAX;
        _baro_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        _mag_nodes[i] = UINT8_MAX;
        _mag_node_taken[i] = 0;
    }

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        _gps_listener_to_node[i] = UINT8_MAX;
        _gps_listeners[i] = nullptr;

        _baro_listener_to_node[i] = UINT8_MAX;
        _baro_listeners[i] = nullptr;

        _mag_listener_to_node[i] = UINT8_MAX;
        _mag_listeners[i] = nullptr;
    }

    _rc_out_sem = hal.util->new_semaphore();
    _fix_out_sem = hal.util->new_semaphore();
    _att_out_sem = hal.util->new_semaphore();

    debug_uavcan(2, "AP_UAVCAN constructed\n\r");
}

AP_UAVCAN::~AP_UAVCAN()
{
}

bool AP_UAVCAN::try_init(void)
{
    if (_parent_can_mgr != nullptr) {
        if (_parent_can_mgr->is_initialized() && !_initialized) {

            _uavcan_i = UINT8_MAX;
            for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
                if (_parent_can_mgr == hal.can_mgr[i]) {
                    _uavcan_i = i;
                    break;
                }
            }

            if(_uavcan_i == UINT8_MAX) {
                return false;
            }

            auto *node = get_node();

            if (node != nullptr) {
                if (!node->isStarted()) {
                    uavcan::NodeID self_node_id(_uavcan_node);
                    node->setNodeID(self_node_id);

                    char ndname[20];
                    snprintf(ndname, sizeof(ndname), "org.ardupilot:%u", _uavcan_i);

                    uavcan::NodeStatusProvider::NodeName name(ndname);
                    node->setName(name);

                    uavcan::protocol::SoftwareVersion sw_version; // Standard type uavcan.protocol.SoftwareVersion
                    sw_version.major = AP_UAVCAN_SW_VERS_MAJOR;
                    sw_version.minor = AP_UAVCAN_SW_VERS_MINOR;
                    node->setSoftwareVersion(sw_version);

                    uavcan::protocol::HardwareVersion hw_version; // Standard type uavcan.protocol.HardwareVersion

                    hw_version.major = AP_UAVCAN_HW_VERS_MAJOR;
                    hw_version.minor = AP_UAVCAN_HW_VERS_MINOR;
                    node->setHardwareVersion(hw_version);

                    const int node_start_res = node->start();
                    if (node_start_res < 0) {
                        debug_uavcan(1, "UAVCAN: node start problem\n\r");
                    }

                    uavcan::Subscriber<uavcan::equipment::gnss::Fix> *gnss_fix;
                    gnss_fix = new uavcan::Subscriber<uavcan::equipment::gnss::Fix>(*node);

                    const int gnss_fix_start_res = gnss_fix->start(gnss_fix_cb_arr[_uavcan_i]);
                    if (gnss_fix_start_res < 0) {
                        debug_uavcan(1, "UAVCAN GNSS subscriber start problem\n\r");
                        return false;
                    }

                    uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary> *gnss_aux;
                    gnss_aux = new uavcan::Subscriber<uavcan::equipment::gnss::Auxiliary>(*node);
                    const int gnss_aux_start_res = gnss_aux->start(gnss_aux_cb_arr[_uavcan_i]);
                    if (gnss_aux_start_res < 0) {
                        debug_uavcan(1, "UAVCAN GNSS Aux subscriber start problem\n\r");
                        return false;
                    }

                    uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength> *magnetic;
                    magnetic = new uavcan::Subscriber<uavcan::equipment::ahrs::MagneticFieldStrength>(*node);
                    const int magnetic_start_res = magnetic->start(magnetic_cb_arr[_uavcan_i]);
                    if (magnetic_start_res < 0) {
                        debug_uavcan(1, "UAVCAN Compass subscriber start problem\n\r");
                        return false;
                    }

                    uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure> *air_data_sp;
                    air_data_sp = new uavcan::Subscriber<uavcan::equipment::air_data::StaticPressure>(*node);
                    const int air_data_sp_start_res = air_data_sp->start(air_data_sp_cb_arr[_uavcan_i]);
                    if (air_data_sp_start_res < 0) {
                        debug_uavcan(1, "UAVCAN Baro subscriber start problem\n\r");
                        return false;
                    }

                    uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature> *air_data_st;
                    air_data_st = new uavcan::Subscriber<uavcan::equipment::air_data::StaticTemperature>(*node);
                    const int air_data_st_start_res = air_data_st->start(air_data_st_cb_arr[_uavcan_i]);
                    if (air_data_st_start_res < 0) {
                        debug_uavcan(1, "UAVCAN Temperature subscriber start problem\n\r");
                        return false;
                    }

                    act_out_array[_uavcan_i] = new uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>(*node);
                    act_out_array[_uavcan_i]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
                    act_out_array[_uavcan_i]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

                    esc_raw[_uavcan_i] = new uavcan::Publisher<uavcan::equipment::esc::RawCommand>(*node);
                    esc_raw[_uavcan_i]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
                    esc_raw[_uavcan_i]->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

                    if(_broadcast_bm & AP_UAVCAN_BROADCAST_POSITION) {
                        fix_out_array[_uavcan_i] = new uavcan::Publisher<uavcan::equipment::gnss::Fix>(*node);
                        fix_out_array[_uavcan_i]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
                        fix_out_array[_uavcan_i]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);
                    }

                    if(_broadcast_bm & AP_UAVCAN_BROADCAST_ATTITUDE) {
                        attitude_out_array[_uavcan_i] = new uavcan::Publisher<uavcan::equipment::ahrs::Solution>(*node);
                        attitude_out_array[_uavcan_i]->setTxTimeout(uavcan::MonotonicDuration::fromMSec(20));
                        attitude_out_array[_uavcan_i]->setPriority(uavcan::TransferPriority::OneHigherThanLowest);
                    }

                    /*
                     * Informing other nodes that we're ready to work.
                     * Default mode is INITIALIZING.
                     */
                    node->setModeOperational();

                    _initialized = true;

                    debug_uavcan(1, "UAVCAN: init done\n\r");

                    return true;
                }
            }
        }

        if (_initialized) {
            return true;
        }
    }

    return false;
}

bool AP_UAVCAN::att_out_sem_take()
{
    bool sem_ret = _att_out_sem->take(10);
    if (!sem_ret) {
        debug_uavcan(1, "AP_UAVCAN attitude semaphore fail\n\r");
    }
    return sem_ret;
}

void AP_UAVCAN::att_out_sem_give()
{
    _att_out_sem->give();
}

bool AP_UAVCAN::fix_out_sem_take()
{
    bool sem_ret = _fix_out_sem->take(10);
    if (!sem_ret) {
        debug_uavcan(1, "AP_UAVCAN GNSS fix semaphore fail\n\r");
    }
    return sem_ret;
}

void AP_UAVCAN::fix_out_sem_give()
{
    _fix_out_sem->give();
}

bool AP_UAVCAN::rc_out_sem_take()
{
    bool sem_ret = _rc_out_sem->take(10);
    if (!sem_ret) {
        debug_uavcan(1, "AP_UAVCAN RCOut semaphore fail\n\r");
    }
    return sem_ret;
}

void AP_UAVCAN::rc_out_sem_give()
{
    _rc_out_sem->give();
}

void AP_UAVCAN::do_cyclic(void)
{
    static uint64_t fix_out_last_send_time = AP_HAL::millis64();
    static uint64_t att_out_last_send_time = AP_HAL::millis64();

    if (_initialized) {
        auto *node = get_node();

        const int error = node->spin(uavcan::MonotonicDuration::fromMSec(1));

        if (error < 0) {
            hal.scheduler->delay_microseconds(1000);
        } else {
            if (rc_out_sem_take()) {
                if (_rco_armed) {
                    bool repeat_send;

                    // if we have any Servos in bitmask
                    if (_servo_bm > 0) {
                        uint8_t starting_servo = 0;

                        do {
                            repeat_send = false;
                            uavcan::equipment::actuator::ArrayCommand msg;

                            uint8_t i;
                            // UAVCAN can hold maximum of 15 commands in one frame
                            for (i = 0; starting_servo < UAVCAN_RCO_NUMBER && i < 15; starting_servo++) {
                                uavcan::equipment::actuator::Command cmd;

                                /*
                                 * Servo output uses a range of 1000-2000 PWM for scaling.
                                 * This converts output PWM from [1000:2000] range to [-1:1] range that
                                 * is passed to servo as unitless type via UAVCAN.
                                 * This approach allows for MIN/TRIM/MAX values to be used fully on
                                 * autopilot side and for servo it should have the setup to provide maximum
                                 * physically possible throws at [-1:1] limits.
                                 */

                                if (_rco_conf[starting_servo].active && ((((uint32_t) 1) << starting_servo) & _servo_bm)) {
                                    cmd.actuator_id = starting_servo + 1;

                                    // TODO: other types
                                    cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_UNITLESS;

                                    // TODO: failsafe, safety
                                    cmd.command_value = constrain_float(((float) _rco_conf[starting_servo].pulse - 1000.0) / 500.0 - 1.0, -1.0, 1.0);

                                    msg.commands.push_back(cmd);

                                    i++;
                                }
                            }

                            if (i > 0) {
                                act_out_array[_uavcan_i]->broadcast(msg);

                                if (i == 15) {
                                    repeat_send = true;
                                }
                            }
                        } while (repeat_send);
                    }

                    // if we have any ESC's in bitmask
                    if (_esc_bm > 0) {
                        static const int cmd_max = uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max();
                        uavcan::equipment::esc::RawCommand esc_msg;

                        uint8_t active_esc_num = 0, max_esc_num = 0;
                        uint8_t k = 0;

                        // find out how many esc we have enabled and if they are active at all
                        for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER; i++) {
                            if ((((uint32_t) 1) << i) & _esc_bm) {
                                max_esc_num = i + 1;
                                if (_rco_conf[i].active) {
                                    active_esc_num++;
                                }
                            }
                        }

                        // if at least one is active (update) we need to send to all
                        if (active_esc_num > 0) {
                            k = 0;

                            for (uint8_t i = 0; i < max_esc_num && k < 20; i++) {
                                uavcan::equipment::actuator::Command cmd;

                                if ((((uint32_t) 1) << i) & _esc_bm) {
                                    // TODO: ESC negative scaling for reverse thrust and reverse rotation
                                    float scaled = cmd_max * (hal.rcout->scale_esc_to_unity(_rco_conf[i].pulse) + 1.0) / 2.0;

                                    scaled = constrain_float(scaled, 0, cmd_max);

                                    esc_msg.cmd.push_back(static_cast<int>(scaled));
                                } else {
                                    esc_msg.cmd.push_back(static_cast<unsigned>(0));
                                }

                                k++;
                            }

                            esc_raw[_uavcan_i]->broadcast(esc_msg);
                        }
                    }
                }

                for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER; i++) {
                    // mark as transmitted
                    _rco_conf[i].active = false;
                }

                rc_out_sem_give();
            }

            uint64_t c_send_time = AP_HAL::millis64();

            // TODO: add check by bitmask
            if (fix_out_array[_uavcan_i] != nullptr) {
                if ((c_send_time - fix_out_last_send_time) > abs(1000 / _broadcast_fix_rate)) {
                    if (fix_out_sem_take()) {
                        fix_out_array[_uavcan_i]->broadcast(_fix_state);
                        fix_out_sem_give();
                    }

                    fix_out_last_send_time = c_send_time;
                }
            }

            if (attitude_out_array[_uavcan_i] != nullptr) {
                if ((c_send_time - att_out_last_send_time) > abs(1000 / _broadcast_att_rate)) {
                    if (att_out_sem_take()) {
                        attitude_out_array[_uavcan_i]->broadcast(_att_state);
                        att_out_sem_give();
                    }

                    att_out_last_send_time = c_send_time;
                }
            }
        }
    } else {
        hal.scheduler->delay_microseconds(1000);
    }
}

uavcan::ISystemClock & AP_UAVCAN::get_system_clock()
{
    return SystemClock::instance();
}

uavcan::ICanDriver * AP_UAVCAN::get_can_driver()
{
    if (_parent_can_mgr != nullptr) {
        if (_parent_can_mgr->is_initialized() == false) {
            return nullptr;
        } else {
            return _parent_can_mgr;
        }
    }
    return nullptr;
}

uavcan::Node<0> *AP_UAVCAN::get_node()
{
    if (_node == nullptr && get_can_driver() != nullptr) {
        _node = new uavcan::Node<0>(*get_can_driver(), get_system_clock(), _node_allocator);
    }

    return _node;
}

void AP_UAVCAN::rco_set_safety_pwm(uint32_t chmask, uint16_t pulse_len)
{
    for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER; i++) {
        if (chmask & (((uint32_t) 1) << i)) {
            _rco_conf[i].safety_pulse = pulse_len;
        }
    }
}

void AP_UAVCAN::rco_set_failsafe_pwm(uint32_t chmask, uint16_t pulse_len)
{
    for (uint8_t i = 0; i < UAVCAN_RCO_NUMBER; i++) {
        if (chmask & (((uint32_t) 1) << i)) {
            _rco_conf[i].failsafe_pulse = pulse_len;
        }
    }
}

void AP_UAVCAN::rco_force_safety_on(void)
{
    _rco_safety = true;
}

void AP_UAVCAN::rco_force_safety_off(void)
{
    _rco_safety = false;
}

void AP_UAVCAN::rco_arm_actuators(bool arm)
{
    _rco_armed = arm;
}

void AP_UAVCAN::rco_write(uint16_t pulse_len, uint8_t ch)
{
    _rco_conf[ch].pulse = pulse_len;
    _rco_conf[ch].active = true;
}

uint8_t AP_UAVCAN::find_gps_without_listener(void)
{
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr && _gps_nodes[i] != UINT8_MAX) {
            return _gps_nodes[i];
        }
    }

    return UINT8_MAX;
}

uint8_t AP_UAVCAN::register_gps_listener(AP_GPS_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        if (preferred_channel != 0) {
            if (preferred_channel <= AP_UAVCAN_MAX_GPS_NODES) {
                _gps_listeners[sel_place] = new_listener;
                _gps_listener_to_node[sel_place] = preferred_channel - 1;
                _gps_node_taken[_gps_listener_to_node[sel_place]]++;
                ret = preferred_channel;

                debug_uavcan(2, "reg_GPS place:%d, chan: %d\n\r", sel_place, preferred_channel);
            }
        } else {
            for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
                if (_gps_node_taken[i] == 0) {
                    _gps_listeners[sel_place] = new_listener;
                    _gps_listener_to_node[sel_place] = i;
                    _gps_node_taken[i]++;
                    ret = i + 1;

                    debug_uavcan(2, "reg_GPS place:%d, chan: %d\n\r", sel_place, i);
                    break;
                }
            }
        }
    }

    return ret;
}

uint8_t AP_UAVCAN::register_gps_listener_to_node(AP_GPS_Backend* new_listener, uint8_t node)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
            if (_gps_nodes[i] == node) {
                _gps_listeners[sel_place] = new_listener;
                _gps_listener_to_node[sel_place] = i;
                _gps_node_taken[i]++;
                ret = i + 1;

                debug_uavcan(2, "reg_GPS place:%d, chan: %d\n\r", sel_place, i);
                break;
            }
        }
    }

    return ret;
}

void AP_UAVCAN::remove_gps_listener(AP_GPS_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_gps_listeners[i] == rem_listener) {
            _gps_listeners[i] = nullptr;

            // Also decrement usage counter and reset listening node
            if (_gps_node_taken[_gps_listener_to_node[i]] > 0) {
                _gps_node_taken[_gps_listener_to_node[i]]--;
            }
            _gps_listener_to_node[i] = UINT8_MAX;
        }
    }
}

AP_GPS::GPS_State *AP_UAVCAN::find_gps_node(uint8_t node)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] == node) {
            return &_gps_node_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] == UINT8_MAX) {
            _gps_nodes[i] = node;
            return &_gps_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

void AP_UAVCAN::update_gps_state(uint8_t node)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_GPS_NODES; i++) {
        if (_gps_nodes[i] == node) {
            for (uint8_t j = 0; j < AP_UAVCAN_MAX_LISTENERS; j++) {
                if (_gps_listener_to_node[j] == i) {
                    _gps_listeners[j]->handle_gnss_msg(_gps_node_state[i]);
                }
            }
        }
    }
}

uint8_t AP_UAVCAN::register_baro_listener(AP_Baro_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        if (preferred_channel != 0) {
            if (preferred_channel < AP_UAVCAN_MAX_BARO_NODES) {
                _baro_listeners[sel_place] = new_listener;
                _baro_listener_to_node[sel_place] = preferred_channel - 1;
                _baro_node_taken[_baro_listener_to_node[sel_place]]++;
                ret = preferred_channel;

                debug_uavcan(2, "reg_Baro place:%d, chan: %d\n\r", sel_place, preferred_channel);
            }
        } else {
            for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
                if (_baro_node_taken[i] == 0) {
                    _baro_listeners[sel_place] = new_listener;
                    _baro_listener_to_node[sel_place] = i;
                    _baro_node_taken[i]++;
                    ret = i + 1;

                    debug_uavcan(2, "reg_BARO place:%d, chan: %d\n\r", sel_place, i);
                    break;
                }
            }
        }
    }

    return ret;
}

uint8_t AP_UAVCAN::register_baro_listener_to_node(AP_Baro_Backend* new_listener, uint8_t node)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
            if (_baro_nodes[i] == node) {
                _baro_listeners[sel_place] = new_listener;
                _baro_listener_to_node[sel_place] = i;
                _baro_node_taken[i]++;
                ret = i + 1;

                debug_uavcan(2, "reg_BARO place:%d, chan: %d\n\r", sel_place, i);
                break;
            }
        }
    }

    return ret;
}


void AP_UAVCAN::remove_baro_listener(AP_Baro_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_baro_listeners[i] == rem_listener) {
            _baro_listeners[i] = nullptr;

            // Also decrement usage counter and reset listening node
            if (_baro_node_taken[_baro_listener_to_node[i]] > 0) {
                _baro_node_taken[_baro_listener_to_node[i]]--;
            }
            _baro_listener_to_node[i] = UINT8_MAX;
        }
    }
}

AP_UAVCAN::Baro_Info *AP_UAVCAN::find_baro_node(uint8_t node)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        if (_baro_nodes[i] == node) {
            return &_baro_node_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        if (_baro_nodes[i] == UINT8_MAX) {

            _baro_nodes[i] = node;
            return &_baro_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

void AP_UAVCAN::update_baro_state(uint8_t node)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_BARO_NODES; i++) {
        if (_baro_nodes[i] == node) {
            for (uint8_t j = 0; j < AP_UAVCAN_MAX_LISTENERS; j++) {
                if (_baro_listener_to_node[j] == i) {
                    _baro_listeners[j]->handle_baro_msg(_baro_node_state[i].pressure, _baro_node_state[i].temperature);
                }
            }
        }
    }
}

void AP_UAVCAN::set_baro(AP_Baro *baro)
{
    _baro = baro;
}

uint8_t AP_UAVCAN::register_mag_listener(AP_Compass_Backend* new_listener, uint8_t preferred_channel)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        if (preferred_channel != 0) {
            if (preferred_channel < AP_UAVCAN_MAX_MAG_NODES) {
                _mag_listeners[sel_place] = new_listener;
                _mag_listener_to_node[sel_place] = preferred_channel - 1;
                _mag_node_taken[_mag_listener_to_node[sel_place]]++;
                ret = preferred_channel;

                debug_uavcan(2, "reg_Compass place:%d, chan: %d\n\r", sel_place, preferred_channel);
            }
        } else {
            for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
                if (_mag_node_taken[i] == 0) {
                    _mag_listeners[sel_place] = new_listener;
                    _mag_listener_to_node[sel_place] = i;
                    _mag_node_taken[i]++;
                    ret = i + 1;

                    debug_uavcan(2, "reg_MAG place:%d, chan: %d\n\r", sel_place, i);
                    break;
                }
            }
        }
    }

    return ret;
}

uint8_t AP_UAVCAN::register_mag_listener_to_node(AP_Compass_Backend* new_listener, uint8_t node)
{
    uint8_t sel_place = UINT8_MAX, ret = 0;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] == nullptr) {
            sel_place = i;
            break;
        }
    }

    if (sel_place != UINT8_MAX) {
        for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
            if (_mag_nodes[i] == node) {
                _mag_listeners[sel_place] = new_listener;
                _mag_listener_to_node[sel_place] = i;
                _mag_node_taken[i]++;
                ret = i + 1;

                debug_uavcan(2, "reg_MAG place:%d, chan: %d\n\r", sel_place, i);
                break;
            }
        }
    }

    return ret;
}

void AP_UAVCAN::remove_mag_listener(AP_Compass_Backend* rem_listener)
{
    // Check for all listeners and compare pointers
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_LISTENERS; i++) {
        if (_mag_listeners[i] == rem_listener) {
            _mag_listeners[i] = nullptr;

            // Also decrement usage counter and reset listening node
            if (_mag_node_taken[_mag_listener_to_node[i]] > 0) {
                _mag_node_taken[_mag_listener_to_node[i]]--;
            }
            _mag_listener_to_node[i] = UINT8_MAX;
        }
    }
}

AP_UAVCAN::Mag_Info *AP_UAVCAN::find_mag_node(uint8_t node)
{
    // Check if such node is already defined
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] == node) {
            return &_mag_node_state[i];
        }
    }

    // If not - try to find free space for it
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] == UINT8_MAX) {
            _mag_nodes[i] = node;
            return &_mag_node_state[i];
        }
    }

    // If no space is left - return nullptr
    return nullptr;
}

/*
 * Find discovered not taken mag node with smallest node ID
 */
uint8_t AP_UAVCAN::find_smallest_free_mag_node()
{
    uint8_t ret = UINT8_MAX;

    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_node_taken[i] == 0) {
            ret = MIN(ret, _mag_nodes[i]);
        }
    }

    return ret;
}

void AP_UAVCAN::update_mag_state(uint8_t node)
{
    // Go through all listeners of specified node and call their's update methods
    for (uint8_t i = 0; i < AP_UAVCAN_MAX_MAG_NODES; i++) {
        if (_mag_nodes[i] == node) {
            for (uint8_t j = 0; j < AP_UAVCAN_MAX_LISTENERS; j++) {
                if (_mag_listener_to_node[j] == i) {
                    _mag_listeners[j]->handle_mag_msg(_mag_node_state[i].mag_vector);
                }
            }
        }
    }
}

#endif // HAL_WITH_UAVCAN
