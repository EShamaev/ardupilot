// Bank angle command based on adaptive optimal control.
// Akshath Singhal

#include <AP_HAL/AP_HAL.h>
#include "AP_LQR_Control.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_LQR_Control::var_info[] = {
    
    // @Param: LIM_BANK
    // @DisplayName: Loiter Radius Bank Angle Limit
    // @Description: The sealevel bank angle limit for a continous loiter. (Used to calculate airframe loading limits at higher altitudes). Setting to 0, will instead just scale the loiter radius directly
    // @Units: deg
    // @Range: 0 89
    // @User: Advanced
    AP_GROUPINFO_FRAME("LIM_BANK",   1, AP_LQR_Control, _loiter_bank_limit, 0.0f, AP_PARAM_FRAME_PLANE),

    // @Param: LQR_MAX_XTRACK
    // @DisplayName: LQR maximum allowed Crosstrack error
    // @Description: Maximum allowed crosstrack error(in centimeters) for LQR. A very small value can lead to high frequency of oscillations. A very large value leads to slow convergence to expected path.
    // @Range: 400 10000
    // @Units: cm
    // @Increment: 20
    // @User: Advanced
    AP_GROUPINFO("MAX_XTRCK",   2, AP_LQR_Control, _max_xtrack, 2000),

    // @Param: LQR_Q2_VAL
    // @DisplayName: LQR VALUE of Q2
    // @Description: The value of Q2 to be used. Divided by 100 before use.
    // @Range: 1 10000
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("Q2_VAL",   3, AP_LQR_Control, _q2_val, 100),

    // @Param: LQR_XTRACK_FACTOR
    // @DisplayName: LQR XTRACK Error weightage in acceleration.
    // @Description: The affect of Xtrackk error on lateral acceleration. Default is 100. Divided by 100 before use.
    // @Range: 10 1000
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("XTR_FAC",   4, AP_LQR_Control, _xtrack_fac, 100),

    // @Param: LQR_VELOCITY_FACTOR
    // @DisplayName: Weightage of rate of change of Xtrack in acceleration.
    // @Description: The affect of rate of change of Xtrackk error on lateral acceleration. Default is 150. Divided by 100 before use.
    // @Range: 10 1000
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("VEL_FAC",   5, AP_LQR_Control, _vel_fac, 150),

    // @Param: LQR_MAX_LATACC
    // @DisplayName: LQR maximum allowed lateral acceleration
    // @Description: Maximum allowed lateral acceleration(in centimeters/sec sqaure) for LQR. A very small value can lead to slow turning and higher overshoots. A very large value leads to very small turning radius causing unstable roll and yaw throws.
    // @Range: 0 10000
    // @Units: cm/s/s
    // @Increment: 25
    // @User: Advanced
    AP_GROUPINFO("LATACC",   6, AP_LQR_Control, _max_latacc, 10000),

    // @Param: LQR_TARGET_SPEED
    // @DisplayName: Set speed of the target to be tracked.
    // @Description: The autopilot assumes an imaginary target of speed moving towards north.
    // @Range: 0 25
    // @Units: m/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LQR_T_SPD", 7, AP_LQR_Control, _tar_speed, 0),

    AP_GROUPEND
};

/*
  Wrap AHRS yaw if in reverse - radians
 */
float AP_LQR_Control::get_yaw()
{
    if (_reverse) {
        return wrap_PI(M_PI + _ahrs.yaw);
    }
    return _ahrs.yaw;
}

/*
  Wrap AHRS yaw sensor if in reverse - centi-degress
 */
float AP_LQR_Control::get_yaw_sensor()
{
    if (_reverse) {
        return wrap_180_cd(18000 + _ahrs.yaw_sensor);
    }
    return _ahrs.yaw_sensor;
}

/*
  return the bank angle needed to achieve tracking from the last
  update_*() operation
 */
int32_t AP_LQR_Control::nav_roll_cd(void) const
{
    float ret;
    ret = cosf(_ahrs.pitch)*degrees(atanf(_latAccDem * 0.101972f) * 100.0f); // 0.101972 = 1/9.81
    ret = constrain_float(ret, -9000, 9000);
    return ret;
}

/*
  return the lateral acceleration needed to achieve tracking from the last
  update_*() operation
 */
float AP_LQR_Control::lateral_acceleration(void) const
{
    return _latAccDem;
}

int32_t AP_LQR_Control::nav_bearing_cd(void) const
{
    return wrap_180_cd(RadiansToCentiDegrees(_nav_bearing));
}

int32_t AP_LQR_Control::bearing_error_cd(void) const
{
    return RadiansToCentiDegrees(_bearing_error);
}

int32_t AP_LQR_Control::target_bearing_cd(void) const
{
    return wrap_180_cd(_target_bearing_cd);
}

/*
  this is the turn distance assuming a 90 degree turn
 */
float AP_LQR_Control::turn_distance(float wp_radius) const
{
    wp_radius *= sq(_ahrs.get_EAS2TAS());
    return wp_radius;
}

/*
  this approximates the turn distance for a given turn angle. If the
  turn_angle is > 90 then a 90 degree turn distance is used, otherwise
  the turn distance is reduced linearly.
  This function allows straight ahead mission legs to avoid thinking
  they have reached the waypoint early, which makes things like camera
  trigger and ball drop at exact positions under mission control much easier
 */
float AP_LQR_Control::turn_distance(float wp_radius, float turn_angle) const
{
    float distance_90 = turn_distance(wp_radius);
    turn_angle = fabsf(turn_angle);
    if (turn_angle >= 90) {
        return distance_90;
    }
    return distance_90 * turn_angle / 90.0f;
}

float AP_LQR_Control::loiter_radius(const float radius) const
{
    // prevent an insane loiter bank limit
    float sanitized_bank_limit = constrain_float(_loiter_bank_limit, 0.0f, 89.0f);
    float lateral_accel_sea_level = tanf(radians(sanitized_bank_limit)) * GRAVITY_MSS;

    float nominal_velocity_sea_level;
    if(_spdHgtControl == nullptr) {
        nominal_velocity_sea_level = 0.0f;
    } else {
        nominal_velocity_sea_level =  _spdHgtControl->get_target_airspeed();
    }

    float eas2tas_sq = sq(_ahrs.get_EAS2TAS());

    if (is_zero(sanitized_bank_limit) || is_zero(nominal_velocity_sea_level) ||
        is_zero(lateral_accel_sea_level)) {
        // Missing a sane input for calculating the limit, or the user has
        // requested a straight scaling with altitude. This will always vary
        // with the current altitude, but will at least protect the airframe
        return radius * eas2tas_sq;
    } else {
        float sea_level_radius = sq(nominal_velocity_sea_level) / lateral_accel_sea_level;
        if (sea_level_radius > radius) {
            // If we've told the plane that its sea level radius is unachievable fallback to
            // straight altitude scaling
            return radius * eas2tas_sq;
        } else {
            // select the requested radius, or the required altitude scale, whichever is safer
            return MAX(sea_level_radius * eas2tas_sq, radius);
        }
    }
}

bool AP_LQR_Control::reached_loiter_target(void)
{
    return _WPcircle;
}

/**
   prevent indecision in our turning by using our previous turn
   decision if we are in a narrow angle band pointing away from the
   target and the turn angle has changed sign
 */

// update L1 control for waypoint navigation
void AP_LQR_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min)
{

    struct Location _current_loc;
    
    uint32_t now = AP_HAL::micros();
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;
    if (dt > 0.1) {
        dt = 0.1;
    }
    _last_update_waypoint_us = now;

    // Get current position and velocity
    if (_ahrs.get_position(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    // update _target_bearing_cd
    _target_bearing_cd = get_bearing_cd(_current_loc, next_WP);

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw()), sinf(get_yaw())) * groundSpeed;
    }

    // Calculate the NE position of WP B relative to WP A
    Vector2f AB = location_diff(prev_WP, next_WP);

    // Check for AB zero length and track directly to the destination
    // if too small
    if (AB.length() < 1.0e-6f) {
        AB = location_diff(_current_loc, next_WP);
        if (AB.length() < 1.0e-6f) {
            AB = Vector2f(cosf(get_yaw()), sinf(get_yaw()));
        }
    }
    AB.normalize();

    // Calculate the NE position of the aircraft relative to WP A
    Vector2f A_air = location_diff(prev_WP, _current_loc);

    // calculate distance to target track, for reporting
    _crosstrack_error = A_air % AB;


    _crosstrack_error=- _crosstrack_error;
    float min_turn_rad = 2*groundSpeed;
    if(_max_latacc == 0)
    { 
        _max_latacc = 100*groundSpeed*groundSpeed/min_turn_rad;
    }
    float q1= sqrtf(float((_max_xtrack/100)/(fabs((_max_xtrack/100)-_crosstrack_error))));
    float si = RadiansToCentiDegrees(get_yaw())/100;
    float si_p = (get_bearing_cd(prev_WP,next_WP))/100;
    float temp_sin=sinf(radians(si - si_p));
    float v_d= groundSpeed * temp_sin;
    float u =  - (((_xtrack_fac/100)*q1*_crosstrack_error)+(sqrtf((float)((_q2_val/100)+(2*q1)))*(_vel_fac/100)*v_d));
    

    if (fabs(u)>(_max_latacc/100))
    {
        if(u>0)
            u=(_max_latacc/100);
        if(u<0)
            u= - (_max_latacc/100);
    }
    hal.console->printf("LQR");
    _latAccDem= u;

    
    // Waypoint capture status is always false during waypoint following
    _WPcircle = false;

    

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update L1 control for loitering
void AP_LQR_Control::update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction)
{
    struct Location _current_loc;
    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    // scale loiter radius with square of EAS2TAS to allow us to stay
    // stable at high altitude
    radius = loiter_radius(radius);
    float groundSpeed=_groundspeed_vector.length();
    float si_p_dot=0;
    Vector2f location_difference=location_diff(center_WP, _current_loc);
    _crosstrack_error = location_difference.length() - radius;
        
    if (_tar_speed==0)
    {
        si_p_dot=groundSpeed/location_difference.length();
    }
    else
    {
        float x1=location_difference.x;
        float y1=location_difference.y;
        float dx1=_groundspeed_vector.x;
        float dy1=_groundspeed_vector.y;
        si_p_dot=(((x1*dy1)-y1*(dx1 -_tar_speed))/(location_difference.length()*location_difference.length()));
    }

    float min_turn_rad = 2*groundSpeed;
    if(_max_latacc == 0)
    {
        _max_latacc = 100*groundSpeed*groundSpeed/min_turn_rad;
    }

    if (_crosstrack_error < (radius))
    {
        float si_p = ((_target_bearing_cd + 9000)/100);
        float q1= sqrtf((float)((_max_xtrack/100)/(fabs((_max_xtrack/100)-_crosstrack_error))));
        float si = RadiansToCentiDegrees(get_yaw())/100;
        float temp_sin=sinf(radians(si - si_p));
        float v_d= groundSpeed * temp_sin;
        float u =   (((_xtrack_fac/100)*q1*0.5*_crosstrack_error)+(sqrtf((float)((_q2_val/100)+(2*q1)))*(_vel_fac/100)*v_d)+groundSpeed*si_p_dot);
        if (fabs(u)>(_max_latacc/100))
        {
            if(u>0)
                u=(_max_latacc/100);
            if(u<0)
                u= - (_max_latacc/100);
        }
        _latAccDem= - u;
    }
    else
    {

        float si_p = (_target_bearing_cd/100);
        float q1= sqrtf((float)((_max_xtrack/100)/(fabs((_max_xtrack/100)-_crosstrack_error))));
        float si = RadiansToCentiDegrees(get_yaw())/100;
        float temp_sin=sinf(radians(si - si_p));
        float v_d= groundSpeed * temp_sin;
        float u =   (((_xtrack_fac/100)*q1*0*_crosstrack_error)+(sqrtf((float)((_q2_val/100)+(2*q1)))*(_vel_fac/100)*v_d));
        if (fabs(u)>(_max_latacc/100))
        {
            if(u>0)
                u=(_max_latacc/100);
            if(u<0)
                u= - (_max_latacc/100);
        }
        _latAccDem= - u;
    }

    _data_is_stale = false; // status are correctly updated with current waypoint data
}


// update LQR control for heading hold navigation
void AP_LQR_Control::update_heading_hold(int32_t navigation_heading_cd)
{

    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = wrap_180_cd(navigation_heading_cd);

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw()), sinf(get_yaw())) * groundSpeed;
    }

    float si = RadiansToCentiDegrees(get_yaw())/100;
    float si_p = (_target_bearing_cd)/100;
    float temp_sin=sinf(radians(si - si_p));
    float v_d= groundSpeed * temp_sin;
    float u =  - ((_vel_fac/100)*v_d);
    

    if (fabs(u)>(_max_latacc/100))
    {
        if(u>0)
            u=(_max_latacc/100);
        if(u<0)
            u= - (_max_latacc/100);
    }
    _latAccDem= u;


    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update LQR control for level flight on current heading
void AP_LQR_Control::update_level_flight(void)
{
    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = _ahrs.yaw_sensor;
    _nav_bearing = _ahrs.yaw;
    _bearing_error = 0;
    _crosstrack_error = 0;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _latAccDem = 0;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}
