#include "Plane.h"

/*
  Runway centreline tracking for AUTO takeoff, start sanity check and
  below-V1 ground-roll abort. See runway_takeoff.h for an overview.
 */

#if AP_PLANE_RUNWAY_TAKEOFF_ENABLED

#include <stdarg.h>

// distance either side of the reference point used to build the
// tracked line. Must be well beyond the L1 distance and the distance
// flown before the takeoff altitude is reached.
#define RWY_LINE_HALF_LENGTH_M 10000.0f

// number of consecutive navigation cycles (10Hz) an abort condition
// must be present before the abort is triggered
#define RWY_ABORT_DEBOUNCE_COUNT 3

// height above the launch baro altitude at which we assume we are
// airborne and stop considering an abort
#define RWY_ABORT_MAX_HEIGHT_M 5.0f

#define RWY_REPORT_INTERVAL_MS 2000U
#define RWY_REFUSE_MSG_INTERVAL_MS 3000U

const AP_Param::GroupInfo RunwayTakeoff::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Runway takeoff enable
    // @Description: Enables runway centreline tracking for AUTO takeoffs. The NAV_TAKEOFF mission item lat/lon must be a surveyed point on the runway centreline ahead of the aircraft (ideally the departure end) and RWY_HDG must be the true runway bearing in the takeoff direction. When enabled, auto takeoff is refused if the runway is not defined or the start checks fail.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, RunwayTakeoff, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HDG
    // @DisplayName: Runway true heading
    // @Description: True bearing of the runway centreline in the takeoff direction. Best measured between two surveyed centreline points. -1 means not set.
    // @Units: deg
    // @Range: -1 360
    // @User: Advanced
    AP_GROUPINFO("HDG", 2, RunwayTakeoff, rwy_hdg, -1),

    // @Param: STRT_XTRK
    // @DisplayName: Max start cross-track error
    // @Description: Takeoff is refused if the aircraft is further than this from the runway centreline when the takeoff roll would start.
    // @Units: m
    // @Range: 0.5 20
    // @User: Advanced
    AP_GROUPINFO("STRT_XTRK", 3, RunwayTakeoff, start_xtrk_max, 3),

    // @Param: STRT_HDG
    // @DisplayName: Max start heading error
    // @Description: Takeoff is refused if the aircraft heading differs from RWY_HDG by more than this when the takeoff roll would start.
    // @Units: deg
    // @Range: 1 30
    // @User: Advanced
    AP_GROUPINFO("STRT_HDG", 4, RunwayTakeoff, start_hdg_max, 5),

    // @Param: STRT_DIST
    // @DisplayName: Min runway remaining at start
    // @Description: If non-zero, takeoff is refused unless the NAV_TAKEOFF reference point is at least this far ahead along the runway. Only meaningful when the reference point is the departure end of the runway.
    // @Units: m
    // @Range: 0 5000
    // @User: Advanced
    AP_GROUPINFO("STRT_DIST", 5, RunwayTakeoff, start_dist_min, 0),

    // @Param: ABT_XTRK
    // @DisplayName: Abort cross-track limit
    // @Description: If non-zero, the takeoff is aborted (engine stopped, full brakes) if the cross-track error exceeds this while below RWY_V1. Set this so the main gear stays well inside the runway edge.
    // @Units: m
    // @Range: 0 30
    // @User: Advanced
    AP_GROUPINFO("ABT_XTRK", 6, RunwayTakeoff, abort_xtrk, 0),

    // @Param: ABT_HDG
    // @DisplayName: Abort heading error limit
    // @Description: If non-zero, the takeoff is aborted if the heading error relative to RWY_HDG exceeds this while below RWY_V1.
    // @Units: deg
    // @Range: 0 45
    // @User: Advanced
    AP_GROUPINFO("ABT_HDG", 7, RunwayTakeoff, abort_hdg, 0),

    // @Param: V1
    // @DisplayName: Takeoff decision speed
    // @Description: Decision speed (equivalent airspeed estimate, or GPS ground speed if no airspeed estimate is available). Below this speed an abort condition stops the engine and applies full brakes. At or above it the takeoff continues. Required if RWY_ABT_XTRK or RWY_ABT_HDG is set, and must not exceed TKOFF_ROTATE_SPD.
    // @Units: m/s
    // @Range: 0 60
    // @User: Advanced
    AP_GROUPINFO("V1", 8, RunwayTakeoff, v1, 0),

    // @Param: BRK_CHAN
    // @DisplayName: Wheel brake servo output
    // @Description: Servo output number (1-based) driving the wheel brakes. On abort this output is held at RWY_BRK_PWM, overriding whatever function the output has (including RC passthrough). 0 disables brake control.
    // @Range: 0 32
    // @User: Advanced
    AP_GROUPINFO("BRK_CHAN", 9, RunwayTakeoff, brake_chan, 0),

    // @Param: BRK_PWM
    // @DisplayName: Wheel brake full PWM
    // @Description: PWM sent to RWY_BRK_CHAN for full braking during an abort.
    // @Units: PWM
    // @Range: 800 2200
    // @User: Advanced
    AP_GROUPINFO("BRK_PWM", 10, RunwayTakeoff, brake_pwm, 1900),

    AP_GROUPEND
};

RunwayTakeoff::RunwayTakeoff(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  set up the runway line from the NAV_TAKEOFF item
 */
void RunwayTakeoff::init(const Location &cmd_loc)
{
    // reset per-takeoff state. The abort latch deliberately survives
    // a mission restart; it is only cleared in update_outputs()
    state.line_valid = false;
    state.roll_started = false;
    state.past_v1 = false;
    state.tracking_msg_sent = false;
    state.abort_count = 0;
    state.max_abs_xtrk = 0;
    state.last_refuse_msg_ms = 0;

    if (!enabled()) {
        return;
    }

    if (cmd_loc.lat == 0 && cmd_loc.lng == 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "RWY: TAKEOFF item has no runway lat/lon");
        return;
    }
    if (rwy_hdg < 0 || rwy_hdg > 360) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "RWY: RWY_HDG not set");
        return;
    }

    state.ref = cmd_loc;
    state.line_start = cmd_loc;
    state.line_end = cmd_loc;
    state.line_start.offset_bearing(wrap_360(rwy_hdg + 180), RWY_LINE_HALF_LENGTH_M);
    state.line_end.offset_bearing(rwy_hdg.get(), RWY_LINE_HALF_LENGTH_M);
    state.line_valid = true;

    float xtrk, along;
    get_line_errors(xtrk, along);
    // lat/lon printed as integers (1e-7 deg) to keep full precision
    gcs().send_text(MAV_SEVERITY_INFO, "RWY: hdg %.1f ref %ld %ld",
                    (double)rwy_hdg.get(),
                    (long)cmd_loc.lat,
                    (long)cmd_loc.lng);
    gcs().send_text(MAV_SEVERITY_INFO, "RWY: now xt %.1fm hdg err %.1f rem %.0fm",
                    (double)xtrk, (double)heading_error_deg(), (double)-along);
}

/*
  start-of-roll sanity checks. Returns false (with a rate limited
  message) if the takeoff must not start
 */
bool RunwayTakeoff::launch_allowed(bool in_auto)
{
    if (state.aborted) {
        refuse("RWY: abort latched, disarm to clear");
        return false;
    }
    if (!enabled()) {
        return true;
    }
    if (!in_auto) {
        refuse("RWY: runway takeoff needs AUTO (or RWY_ENABLE=0)");
        return false;
    }
    if (!state.line_valid) {
        refuse("RWY: no runway, check TAKEOFF lat/lon+RWY_HDG");
        return false;
    }
    if (abort_enabled()) {
        if (v1 <= 0) {
            refuse("RWY: abort enabled but RWY_V1 not set");
            return false;
        }
        if (plane.g.takeoff_rotate_speed > 0 && v1 > plane.g.takeoff_rotate_speed) {
            refuse("RWY: RWY_V1 %.1f > TKOFF_ROTATE_SPD %.1f",
                   (double)v1.get(), (double)plane.g.takeoff_rotate_speed.get());
            return false;
        }
    }

    float xtrk, along;
    get_line_errors(xtrk, along);
    const float hdg_err = heading_error_deg();

    if (fabsf(xtrk) > start_xtrk_max) {
        refuse("RWY: start xt %.1fm > %.1fm", (double)xtrk, (double)start_xtrk_max.get());
        return false;
    }
    if (fabsf(hdg_err) > start_hdg_max) {
        refuse("RWY: start hdg err %.1f > %.1f", (double)hdg_err, (double)start_hdg_max.get());
        return false;
    }
    if (along > 0) {
        refuse("RWY: ref point %.0fm behind aircraft", (double)along);
        return false;
    }
    if (start_dist_min > 0 && -along < start_dist_min) {
        refuse("RWY: runway rem %.0fm < %.0fm", (double)-along, (double)start_dist_min.get());
        return false;
    }
    return true;
}

void RunwayTakeoff::launch_started()
{
    if (!active()) {
        return;
    }
    state.roll_started = true;
    state.past_v1 = false;
    state.abort_count = 0;
    state.max_abs_xtrk = 0;
    state.tracking_msg_sent = false;
    state.last_report_ms = AP_HAL::millis();

    float xtrk, along;
    get_line_errors(xtrk, along);
    gcs().send_text(MAV_SEVERITY_INFO, "RWY: roll xt %.1fm hdg err %.1f rem %.0fm",
                    (double)xtrk, (double)heading_error_deg(), (double)-along);
    if (abort_enabled()) {
        gcs().send_text(MAV_SEVERITY_INFO, "RWY: V1 %.1f abort xt %.1fm hdg %.0f",
                        (double)v1.get(), (double)abort_xtrk.get(), (double)abort_hdg.get());
        if (brake_chan <= 0) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RWY: no brake output set (RWY_BRK_CHAN)");
        }
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "RWY: abort disabled");
    }
}

int32_t RunwayTakeoff::runway_heading_cd() const
{
    return wrap_360_cd(int32_t(rwy_hdg * 100));
}

void RunwayTakeoff::update_l1()
{
    plane.nav_controller->update_waypoint(state.line_start, state.line_end);
    if (!state.tracking_msg_sent) {
        state.tracking_msg_sent = true;
        float xtrk, along;
        get_line_errors(xtrk, along);
        gcs().send_text(MAV_SEVERITY_INFO, "RWY: tracking centreline at %.1fm/s xt %.1fm",
                        (double)plane.gps.ground_speed(), (double)xtrk);
    }
}

void RunwayTakeoff::update_monitor()
{
    if (!active() || !state.roll_started) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    float xtrk, along;
    get_line_errors(xtrk, along);
    const float hdg_err = heading_error_deg();
    const float spd = speed_for_v1();

    state.max_abs_xtrk = MAX(state.max_abs_xtrk, fabsf(xtrk));
    write_log(xtrk, along, hdg_err, spd);

    if (now_ms - state.last_report_ms >= RWY_REPORT_INTERVAL_MS) {
        state.last_report_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_INFO, "RWY: %s xt %.1fm hdg err %.1f spd %.1f",
                        state.aborted ? "ABORTED" : "roll",
                        (double)xtrk, (double)hdg_err, (double)spd);
    }

    if (state.aborted) {
        return;
    }

    if (!state.past_v1 && abort_enabled()) {
        if (spd >= v1) {
            state.past_v1 = true;
            gcs().send_text(MAV_SEVERITY_INFO, "RWY: V1 %.1f, continuing takeoff (xt %.1fm)",
                            (double)spd, (double)xtrk);
        } else if (plane.barometer.get_altitude() - plane.auto_state.baro_takeoff_alt > RWY_ABORT_MAX_HEIGHT_M) {
            state.past_v1 = true;
            gcs().send_text(MAV_SEVERITY_WARNING, "RWY: airborne below V1, abort disabled");
        }
    }

    if (!abort_enabled() || state.past_v1) {
        return;
    }

    const char *reason = nullptr;
    if (abort_xtrk > 0 && fabsf(xtrk) > abort_xtrk) {
        reason = "xtrack";
    } else if (abort_hdg > 0 && fabsf(hdg_err) > abort_hdg) {
        reason = "heading";
    } else if (plane.gps.status() < AP_GPS_FixType::FIX_3D) {
        reason = "GPS fix lost";
    }

    if (reason == nullptr) {
        state.abort_count = 0;
        return;
    }
    if (++state.abort_count >= RWY_ABORT_DEBOUNCE_COUNT) {
        trigger_abort(reason, xtrk, hdg_err, spd);
    }
}

void RunwayTakeoff::takeoff_complete()
{
    if (!active() || !state.roll_started) {
        return;
    }
    state.roll_started = false;
    gcs().send_text(MAV_SEVERITY_INFO, "RWY: takeoff complete, max xt %.1fm",
                    (double)state.max_abs_xtrk);
}

void RunwayTakeoff::trigger_abort(const char *reason, float xtrk, float hdg_err, float spd)
{
    state.aborted = true;

    gcs().send_text(MAV_SEVERITY_CRITICAL, "RWY: ABORT %s xt %.1f he %.1f v %.1f",
                    reason, (double)xtrk, (double)hdg_err, (double)spd);

#if AP_ICENGINE_ENABLED
    // ignition off
    plane.g2.ice_control.engine_control(0, 0, 0, 0);
#endif

    // the emergency stop forces throttle, starter and engine run
    // enable outputs to their stopped values and stops the ICE
    // controller from restarting the engine
    if (!SRV_Channels::get_emergency_stop()) {
        SRV_Channels::set_emergency_stop(true);
        state.we_set_estop = true;
    }

    if (brake_chan > 0 && brake_chan <= NUM_SERVO_CHANNELS) {
        hal.rcout->enable_ch(brake_chan - 1);
    }
}

/*
  run every loop after the servo outputs have been written but before
  they are pushed to the hardware
 */
void RunwayTakeoff::update_outputs()
{
    if (!state.aborted) {
        return;
    }

    if (brake_chan > 0 && brake_chan <= NUM_SERVO_CHANNELS) {
        hal.rcout->write(brake_chan - 1, brake_pwm);
    }

    if (hal.util->get_soft_armed()) {
        return;
    }

    // disarmed. Only clear once the engine start switch (if any) is
    // not asking for the engine to run, so the engine cannot crank
    // as soon as the emergency stop is released
    const uint32_t now_ms = AP_HAL::millis();
    if (engine_switch_in_run()) {
        if (now_ms - state.last_clear_msg_ms > 5000U) {
            state.last_clear_msg_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_WARNING, "RWY: set engine switch to stop to clear abort");
        }
        return;
    }

    state.aborted = false;
    state.roll_started = false;
    if (state.we_set_estop) {
        SRV_Channels::set_emergency_stop(false);
        state.we_set_estop = false;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "RWY: abort cleared, brakes released");
}

void RunwayTakeoff::get_line_errors(float &xtrk, float &along) const
{
    const Vector2f d = state.ref.get_distance_NE(plane.current_loc);
    const float hdg = radians(rwy_hdg);
    const Vector2f u{cosf(hdg), sinf(hdg)};
    along = d.x * u.x + d.y * u.y;
    // +ve to the right of the centreline looking in the takeoff direction
    xtrk = u.x * d.y - u.y * d.x;
}

// +ve when the nose is right of the runway heading
float RunwayTakeoff::heading_error_deg() const
{
    return wrap_180(plane.ahrs.get_yaw_deg() - rwy_hdg);
}

float RunwayTakeoff::speed_for_v1() const
{
    float aspd;
    if (plane.ahrs.airspeed_EAS(aspd)) {
        return aspd;
    }
    return plane.gps.ground_speed();
}

bool RunwayTakeoff::abort_enabled() const
{
    return abort_xtrk > 0 || abort_hdg > 0;
}

/*
  true if an ICE start/stop RC switch is configured and is (or may be)
  asking for the engine to run. With no switch configured (engine
  started and stopped by MAVLink commands) this is always false, so the
  abort clears on disarm.
 */
bool RunwayTakeoff::engine_switch_in_run() const
{
#if AP_ICENGINE_ENABLED
    RC_Channel *c = rc().find_channel_for_option(RC_Channel::AUX_FUNC::ICE_START_STOP);
    if (c == nullptr) {
        return false;
    }
    if (!rc().has_valid_input()) {
        // can't see the switch: stay latched
        return true;
    }
    return c->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
#else
    return false;
#endif
}

void RunwayTakeoff::refuse(const char *fmt, ...)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - state.last_refuse_msg_ms < RWY_REFUSE_MSG_INTERVAL_MS) {
        return;
    }
    state.last_refuse_msg_ms = now_ms;
    va_list ap;
    va_start(ap, fmt);
    gcs().send_textv(MAV_SEVERITY_WARNING, fmt, ap);
    va_end(ap);
}

void RunwayTakeoff::write_log(float xtrk, float along, float hdg_err, float spd)
{
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: RWYT
    // @Description: Runway takeoff tracking
    // @Field: TimeUS: Time since system startup
    // @Field: XT: cross-track error from runway centreline, +ve right
    // @Field: AT: along-track distance past the runway reference point
    // @Field: HE: heading error relative to RWY_HDG, +ve nose right
    // @Field: Spd: speed used for the V1 decision
    // @Field: V1: true once past V1 (or airborne)
    // @Field: Abt: true once aborted
    AP::logger().WriteStreaming("RWYT", "TimeUS,XT,AT,HE,Spd,V1,Abt", "smmdn--", "F000000", "QffffBB",
                                AP_HAL::micros64(),
                                (double)xtrk,
                                (double)along,
                                (double)hdg_err,
                                (double)spd,
                                uint8_t(state.past_v1),
                                uint8_t(state.aborted));
#endif

    // live values for GCS graphing at 5Hz
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - state.last_log_ms >= 200U) {
        state.last_log_ms = now_ms;
        gcs().send_named_float("RWY_XT", xtrk);
        gcs().send_named_float("RWY_HE", hdg_err);
    }
}

#endif // AP_PLANE_RUNWAY_TAKEOFF_ENABLED
