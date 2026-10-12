#include "Plane.h"

/*
  Runway centreline tracking for AUTO takeoff, start sanity check and
  below-V1 ground-roll abort. See runway_takeoff.h for an overview.
 */

#if AP_PLANE_RUNWAY_TAKEOFF_ENABLED

#include <stdarg.h>

// distance the A->B line is extended both ways for the L1 controller.
// Must be well beyond the L1 distance and the distance flown before the
// takeoff altitude is reached.
#define RWY_LINE_EXTEND_M 10000.0f

// minimum A->B length, so survey errors give a small bearing error
#define RWY_MIN_LINE_LENGTH_M 200.0f

// how far behind point A the aircraft may start the roll
#define RWY_MAX_START_BEHIND_A_M 100.0f

// minimum lookahead distance for the ground steering law
#define RWY_MIN_LOOKAHEAD_M 10.0f

// below this ground speed after an abort the steering is centred
#define RWY_ABORT_CENTRE_SPEED 2.0f

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
    // @Description: Enables runway centreline tracking for AUTO takeoffs. The runway line runs from the NAV_TAKEOFF item lat/lon (point A, surveyed centreline point at the start end) to the next mission waypoint (point B, surveyed centreline point at the departure end or beyond). When enabled, auto takeoff is refused if the runway is not defined or the start checks fail.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, RunwayTakeoff, enable, 0, AP_PARAM_FLAG_ENABLE),

    // index 2 was HDG, removed: the runway direction now comes from A->B

    // @Param: STRT_XTRK
    // @DisplayName: Max start cross-track error
    // @Description: Takeoff is refused if the aircraft is further than this from the runway centreline when the takeoff roll would start.
    // @Units: m
    // @Range: 0.5 20
    // @User: Advanced
    AP_GROUPINFO("STRT_XTRK", 3, RunwayTakeoff, start_xtrk_max, 3),

    // @Param: STRT_HDG
    // @DisplayName: Max start heading error
    // @Description: Takeoff is refused if the aircraft heading differs from the A->B direction by more than this when the takeoff roll would start.
    // @Units: deg
    // @Range: 1 30
    // @User: Advanced
    AP_GROUPINFO("STRT_HDG", 4, RunwayTakeoff, start_hdg_max, 5),

    // @Param: STRT_DIST
    // @DisplayName: Min runway remaining at start
    // @Description: If non-zero, takeoff is refused unless point B (the waypoint after NAV_TAKEOFF) is at least this far ahead along the runway. Most useful when B is the departure end of the runway.
    // @Units: m
    // @Range: 0 5000
    // @User: Advanced
    AP_GROUPINFO("STRT_DIST", 5, RunwayTakeoff, start_dist_min, 0),

    // @Param: ABT_XTRK
    // @DisplayName: Abort cross-track limit
    // @Description: If non-zero, the takeoff is aborted (engine stopped, brakes applied) if the cross-track error exceeds this while below RWY_V1. Set this from the runway width and gear track, near where the main gear would leave the runway; a high speed abort is itself a risk.
    // @Units: m
    // @Range: 0 30
    // @User: Advanced
    AP_GROUPINFO("ABT_XTRK", 6, RunwayTakeoff, abort_xtrk, 0),

    // @Param: ABT_HDG
    // @DisplayName: Abort heading error limit
    // @Description: If non-zero, the takeoff is aborted if the heading error relative to the A->B direction exceeds this while below RWY_V1.
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

    // @Param: BLEND
    // @DisplayName: Steering fade-in time
    // @Description: Time from the start of the roll over which the runway steering correction is faded in, to avoid a step in the steering demand.
    // @Units: s
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("BLEND", 11, RunwayTakeoff, blend_time, 2),

    // @Param: MAX_CORR
    // @DisplayName: Max steering correction
    // @Description: Largest heading correction the runway steering will command on the ground, so no position jump or large error can command a hard turn. The steering rate demand is this divided by STEER2SRV_TCONST.
    // @Units: deg
    // @Range: 2 30
    // @User: Advanced
    AP_GROUPINFO("MAX_CORR", 12, RunwayTakeoff, max_corr, 10),

    // @Param: LOOKAHEAD
    // @DisplayName: Steering lookahead time
    // @Description: The runway steering aims at a point on the centreline this many seconds of travel ahead (minimum 10 m). Larger values correct cross-track more gently.
    // @Units: s
    // @Range: 1 10
    // @User: Advanced
    AP_GROUPINFO("LOOKAHEAD", 13, RunwayTakeoff, lookahead, 3),

    // @Param: BRK_RAMP
    // @DisplayName: Abort brake ramp time
    // @Description: Time over which the brake output is ramped from its value at the abort to RWY_BRK_PWM. 0 applies full brakes at once.
    // @Units: s
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("BRK_RAMP", 14, RunwayTakeoff, brake_ramp, 1),

    AP_GROUPEND
};

RunwayTakeoff::RunwayTakeoff(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  set up the runway line from the NAV_TAKEOFF item (A) and the next
  navigation item, which must be a waypoint (B)
 */
void RunwayTakeoff::init(const AP_Mission::Mission_Command &cmd)
{
    // reset per-takeoff state. The abort latch deliberately survives
    // a mission restart; it is only cleared in update_outputs()
    state.line_valid = false;
    state.roll_started = false;
    state.past_v1 = false;
    state.abort_count = 0;
    state.max_abs_xtrk = 0;
    state.last_steer_err = 0;
    state.last_refuse_msg_ms = 0;

    if (!enabled()) {
        return;
    }

    const Location &a = cmd.content.location;
    if (a.lat == 0 && a.lng == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: TAKEOFF item has no lat/lon (point A)");
        return;
    }

    // find the next navigation command without changing mission state
    // (get_next_nav_cmd() can modify DO_JUMP counters)
    AP_Mission::Mission_Command next {};
    bool found = false;
    const uint16_t num_cmds = plane.mission.num_commands();
    for (uint16_t i = cmd.index + 1; i < num_cmds; i++) {
        if (!plane.mission.read_cmd_from_storage(i, next)) {
            break;
        }
        if (AP_Mission::is_nav_cmd(next)) {
            found = true;
            break;
        }
    }
    if (!found || next.id != MAV_CMD_NAV_WAYPOINT) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: need WAYPOINT after TAKEOFF (point B)");
        return;
    }
    const Location &b = next.content.location;
    const float len = a.get_distance(b);
    if (len < RWY_MIN_LINE_LENGTH_M) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: A to B only %.0fm, need %.0fm",
                      (double)len, (double)RWY_MIN_LINE_LENGTH_M);
        return;
    }

    state.point_a = a;
    state.point_b = b;
    state.length_m = len;
    state.bearing_deg = wrap_360(degrees(a.get_bearing(b)));
    state.l1_start = a;
    state.l1_end = b;
    state.l1_start.offset_bearing(wrap_360(state.bearing_deg + 180), RWY_LINE_EXTEND_M);
    state.l1_end.offset_bearing(state.bearing_deg, RWY_LINE_EXTEND_M);
    state.line_valid = true;

    float xtrk, along;
    get_line_errors(xtrk, along);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: A->B brg %.1f len %.0fm",
                  (double)state.bearing_deg, (double)state.length_m);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: now xt %.1fm hdg err %.1f rem %.0fm",
                  (double)xtrk, (double)heading_error_deg(), (double)(state.length_m - along));
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
        refuse("RWY: no runway, check TAKEOFF and next WP");
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
    const float remaining = state.length_m - along;
    if (remaining <= 0) {
        refuse("RWY: point B %.0fm behind aircraft", (double)-remaining);
        return false;
    }
    if (along < -RWY_MAX_START_BEHIND_A_M) {
        refuse("RWY: %.0fm behind point A", (double)-along);
        return false;
    }
    if (start_dist_min > 0 && remaining < start_dist_min) {
        refuse("RWY: runway rem %.0fm < %.0fm", (double)remaining, (double)start_dist_min.get());
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
    state.launch_ms = AP_HAL::millis();
    state.last_report_ms = state.launch_ms;

    // steer along the runway from the start of the roll. A non -1
    // hold_course_cd selects calc_nav_yaw_course(), which takes its
    // error from ground_steering_error()
    plane.steer_state.hold_course_cd = runway_heading_cd();

    float xtrk, along;
    get_line_errors(xtrk, along);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: roll xt %.1fm hdg err %.1f rem %.0fm",
                  (double)xtrk, (double)heading_error_deg(), (double)(state.length_m - along));
    if (abort_enabled()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: V1 %.1f abort xt %.1fm hdg %.0f",
                      (double)v1.get(), (double)abort_xtrk.get(), (double)abort_hdg.get());
        if (brake_chan <= 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: no brake output set (RWY_BRK_CHAN)");
        }
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: abort disabled");
    }
}

int32_t RunwayTakeoff::runway_heading_cd() const
{
    return wrap_360_cd(int32_t(state.bearing_deg * 100));
}

void RunwayTakeoff::update_l1()
{
    plane.nav_controller->update_waypoint(state.l1_start, state.l1_end);
}

/*
  ground steering law. Aim at a point on the centreline a lookahead
  distance ahead, cap the correction, and fade it in from the start of
  the roll. After an abort, hold the runway direction instead of
  chasing the line, and centre the steering once nearly stopped.
  Result is a heading error (desired - current), centidegrees.
 */
void RunwayTakeoff::ground_steering_error(int32_t &bearing_error_cd)
{
    if (!active() || plane.control_mode != &plane.mode_auto ||
        !(state.roll_started || state.aborted)) {
        return;
    }

    const float cap = constrain_float(max_corr, 1, 45);
    const float gspd = plane.ahrs.groundspeed();
    float err;

    if (state.aborted) {
        if (gspd < RWY_ABORT_CENTRE_SPEED) {
            err = 0;
        } else {
            err = wrap_180(state.bearing_deg - plane.ahrs.get_yaw_deg());
        }
    } else {
        float xtrk, along;
        get_line_errors(xtrk, along);
        const float look = MAX(lookahead * gspd, RWY_MIN_LOOKAHEAD_M);
        const float xt_corr = constrain_float(-degrees(atan2f(xtrk, look)), -cap, cap);
        err = wrap_180(state.bearing_deg + xt_corr - plane.ahrs.get_yaw_deg());

        // fade in from the start of the roll
        if (is_positive(blend_time)) {
            const float t = (AP_HAL::millis() - state.launch_ms) * 0.001f;
            err *= constrain_float(t / blend_time, 0, 1);
        }
    }

    err = constrain_float(err, -cap, cap);
    state.last_steer_err = err;
    bearing_error_cd = int32_t(err * 100);
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: %s xt %.1fm hdg err %.1f spd %.1f",
                      state.aborted ? "ABORTED" : "roll",
                      (double)xtrk, (double)hdg_err, (double)spd);
    }

    if (state.aborted) {
        return;
    }

    if (!state.past_v1 && abort_enabled()) {
        if (spd >= v1) {
            state.past_v1 = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: V1 %.1f, continuing takeoff (xt %.1fm)",
                          (double)spd, (double)xtrk);
        } else if (plane.barometer.get_altitude() - plane.auto_state.baro_takeoff_alt > RWY_ABORT_MAX_HEIGHT_M) {
            state.past_v1 = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: airborne below V1, abort disabled");
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
    } else if (plane.gps.status() < AP_GPS::GPS_OK_FIX_3D) {
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
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: takeoff complete, max xt %.1fm",
                  (double)state.max_abs_xtrk);
}

void RunwayTakeoff::trigger_abort(const char *reason, float xtrk, float hdg_err, float spd)
{
    state.aborted = true;

    state.abort_ms = AP_HAL::millis();
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "RWY: ABORT %s xt %.1f he %.1f v %.1f",
                  reason, (double)xtrk, (double)hdg_err, (double)spd);

    // the steering integrator holds the trim built up on the roll. Under
    // braking the nosewheel is suddenly more effective, so start the
    // stop from a clean integrator
    plane.steerController.reset_I();

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
        // ramp from whatever the brake output was at the abort
        state.brake_start_pwm = hal.rcout->read(brake_chan - 1);
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
        uint16_t pwm = brake_pwm;
        const float elapsed = (AP_HAL::millis() - state.abort_ms) * 0.001f;
        if (is_positive(brake_ramp) && elapsed < brake_ramp && state.brake_start_pwm != 0) {
            const float frac = elapsed / brake_ramp;
            pwm = uint16_t(state.brake_start_pwm + (brake_pwm - state.brake_start_pwm) * frac);
        }
        hal.rcout->write(brake_chan - 1, pwm);
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
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RWY: set engine switch to stop to clear abort");
        }
        return;
    }

    state.aborted = false;
    state.roll_started = false;
    if (state.we_set_estop) {
        SRV_Channels::set_emergency_stop(false);
        state.we_set_estop = false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RWY: abort cleared, brakes released");
}

void RunwayTakeoff::get_line_errors(float &xtrk, float &along) const
{
    const Vector2f d = state.point_a.get_distance_NE(plane.current_loc);
    const float hdg = radians(state.bearing_deg);
    const Vector2f u{cosf(hdg), sinf(hdg)};
    along = d.x * u.x + d.y * u.y;
    // +ve to the right of the centreline looking in the takeoff direction
    xtrk = u.x * d.y - u.y * d.x;
}

// +ve when the nose is right of the A->B direction
float RunwayTakeoff::heading_error_deg() const
{
    return wrap_180(plane.ahrs.get_yaw_deg() - state.bearing_deg);
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
    // @Field: AT: along-track distance past point A towards point B
    // @Field: HE: heading error relative to the A->B direction, +ve nose right
    // @Field: Spd: speed used for the V1 decision
    // @Field: SE: runway steering heading error demand, +ve turn right
    // @Field: V1: true once past V1 (or airborne)
    // @Field: Abt: true once aborted
    AP::logger().WriteStreaming("RWYT", "TimeUS,XT,AT,HE,Spd,SE,V1,Abt", "smmdnd--", "F0000000", "QfffffBB",
                                AP_HAL::micros64(),
                                (double)xtrk,
                                (double)along,
                                (double)hdg_err,
                                (double)spd,
                                (double)state.last_steer_err,
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
