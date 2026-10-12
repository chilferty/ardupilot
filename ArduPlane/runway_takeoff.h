#pragma once

/*
  Runway centreline tracking for AUTO takeoff, with a start sanity
  check and a below-V1 ground-roll abort.

  The runway line is defined by two surveyed centreline points carried
  in the mission, mirroring how a landing uses the waypoint before
  NAV_LAND:
    - A: the lat/lon of the NAV_TAKEOFF item (start end of the runway)
    - B: the next navigation item, which must be a NAV_WAYPOINT
         (departure end, or further out on the extended centreline)

  From the start of the roll the nosewheel steers along A->B using a
  simple lookahead law (RWY_LOOKAHEAD), faded in over RWY_BLEND and
  capped at RWY_MAX_CORR. The L1 controller is fed the same line so that
  after liftoff the roll demand follows the extended centreline until
  the takeoff altitude is reached.

  After an abort the steering stops chasing the line and holds the
  runway direction (straight stop), and the brakes are ramped to full
  over RWY_BRK_RAMP.

  With RWY_ENABLE=0 the stock takeoff behaviour is unchanged.
 */

#ifndef AP_PLANE_RUNWAY_TAKEOFF_ENABLED
#define AP_PLANE_RUNWAY_TAKEOFF_ENABLED 1
#endif

#if AP_PLANE_RUNWAY_TAKEOFF_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <AP_Mission/AP_Mission.h>

class RunwayTakeoff
{
public:
    RunwayTakeoff(void);

    CLASS_NO_COPY(RunwayTakeoff);

    static const struct AP_Param::GroupInfo var_info[];

    bool enabled() const { return enable.get() != 0; }

    // called from do_takeoff() with the NAV_TAKEOFF mission item
    void init(const AP_Mission::Mission_Command &cmd);

    // true when a valid runway line has been set up for this takeoff
    bool active() const { return enabled() && state.line_valid; }

    // true from the start of the roll until takeoff complete
    bool roll_started() const { return state.roll_started; }

    // gate called from auto_takeoff_check() before the throttle is
    // released. Returns true if it is OK to start the takeoff roll.
    bool launch_allowed(bool in_auto);

    // called when the takeoff roll has been triggered
    void launch_started();

    // runway (A->B) bearing in centidegrees, for steer_state.hold_course_cd
    int32_t runway_heading_cd() const;

    // feed the L1 controller with the runway line (roll demand)
    void update_l1();

    // called from calc_nav_yaw_course(). Replaces the L1 bearing error
    // with the runway ground steering error while on the takeoff roll
    // or stopping after an abort.
    void ground_steering_error(int32_t &bearing_error_cd);

    // called at the navigation rate from verify_takeoff() for
    // monitoring, abort checks, GCS reporting and logging
    void update_monitor();

    // called when the takeoff altitude has been reached
    void takeoff_complete();

    // true once an abort has been triggered (latched until disarmed)
    bool aborted() const { return state.aborted; }

    // called every loop from servos_output() after the outputs have
    // been calculated. Applies the brake override while aborted and
    // clears the abort latch when safe.
    void update_outputs();

private:
    // parameters
    AP_Int8  enable;
    AP_Float start_xtrk_max;   // m
    AP_Float start_hdg_max;    // deg
    AP_Float start_dist_min;   // m
    AP_Float abort_xtrk;       // m, 0 disables
    AP_Float abort_hdg;        // deg, 0 disables
    AP_Float v1;               // m/s
    AP_Int8  brake_chan;       // servo output number, 0 disables
    AP_Int16 brake_pwm;        // PWM for full brakes
    AP_Float blend_time;       // s
    AP_Float max_corr;         // deg
    AP_Float lookahead;        // s
    AP_Float brake_ramp;       // s

    struct {
        Location point_a;      // TAKEOFF item: start of the runway line
        Location point_b;      // next waypoint: far end of the line
        Location l1_start;     // A->B extended behind A, for L1
        Location l1_end;       // A->B extended beyond B, for L1
        float bearing_deg;     // A->B
        float length_m;        // A->B
        bool line_valid;
        bool roll_started;
        bool past_v1;
        bool aborted;
        bool we_set_estop;
        uint8_t abort_count;   // consecutive cycles with an abort condition
        float max_abs_xtrk;
        float last_steer_err;  // deg, for logging
        uint32_t launch_ms;
        uint32_t abort_ms;
        uint16_t brake_start_pwm;
        uint32_t last_report_ms;
        uint32_t last_refuse_msg_ms;
        uint32_t last_log_ms;
        uint32_t last_clear_msg_ms;
    } state;

    // geometry relative to the runway line. xtrk +ve = right of the
    // centreline looking from A to B, along = metres past A towards B
    void get_line_errors(float &xtrk, float &along) const;
    float heading_error_deg() const;
    float speed_for_v1() const;
    bool abort_enabled() const;

    void trigger_abort(const char *reason, float xtrk, float hdg_err, float spd);
    bool engine_switch_in_run() const;
    void refuse(const char *fmt, ...) FMT_PRINTF(2, 3);
    void write_log(float xtrk, float along, float hdg_err, float spd);
};

#endif // AP_PLANE_RUNWAY_TAKEOFF_ENABLED
