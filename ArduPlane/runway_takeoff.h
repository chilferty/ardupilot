#pragma once

/*
  Runway centreline tracking for AUTO takeoff, with a start sanity
  check and a below-V1 ground-roll abort.

  The runway is defined by:
    - the lat/lon of the NAV_TAKEOFF mission item: any surveyed point on
      the runway centreline AHEAD of the aircraft (ideally the departure
      end of the runway), and
    - RWY_HDG: the true bearing of the runway in the takeoff direction.

  Once the stock code would lock a takeoff heading (GPS ground speed
  > GPS_GND_CRS_MIN_SPD) the L1 controller tracks that line instead of
  holding a heading, so any cross-track error is steered out rather than
  held. The same L1 output drives the nosewheel (calc_nav_yaw_course)
  and, after rotation, the roll demand, so the aircraft continues along
  the extended centreline until the takeoff altitude is reached.

  With RWY_ENABLE=0 the stock takeoff behaviour is unchanged.
 */

#ifndef AP_PLANE_RUNWAY_TAKEOFF_ENABLED
#define AP_PLANE_RUNWAY_TAKEOFF_ENABLED 1
#endif

#if AP_PLANE_RUNWAY_TAKEOFF_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>

class RunwayTakeoff
{
public:
    RunwayTakeoff(void);

    CLASS_NO_COPY(RunwayTakeoff);

    static const struct AP_Param::GroupInfo var_info[];

    bool enabled() const { return enable.get() != 0; }

    // called from do_takeoff() with the NAV_TAKEOFF mission item location
    void init(const Location &cmd_loc);

    // true when a valid runway line has been set up for this takeoff
    bool active() const { return enabled() && state.line_valid; }

    // gate called from auto_takeoff_check() just before the throttle is
    // released. Returns true if it is OK to start the takeoff roll.
    bool launch_allowed(bool in_auto);

    // called when the takeoff roll has been triggered
    void launch_started();

    // runway heading in centidegrees, for steer_state.hold_course_cd
    int32_t runway_heading_cd() const;

    // feed the L1 controller with the runway line
    void update_l1();

    // called at the navigation rate from verify_takeoff() for
    // monitoring, abort checks, GCS reporting and logging
    void update_monitor();

    // called when the takeoff altitude has been reached
    void takeoff_complete();

    // true once an abort has been triggered (latched until disarmed
    // and the engine start switch is not in the run position)
    bool aborted() const { return state.aborted; }

    // called every loop from servos_output() after the outputs have
    // been calculated. Applies the brake override while aborted and
    // clears the abort latch when safe.
    void update_outputs();

private:
    // parameters
    AP_Int8  enable;
    AP_Float rwy_hdg;          // deg true, takeoff direction
    AP_Float start_xtrk_max;   // m
    AP_Float start_hdg_max;    // deg
    AP_Float start_dist_min;   // m
    AP_Float abort_xtrk;       // m, 0 disables
    AP_Float abort_hdg;        // deg, 0 disables
    AP_Float v1;               // m/s
    AP_Int8  brake_chan;       // servo output number, 0 disables
    AP_Int16 brake_pwm;        // PWM for full brakes

    struct {
        Location ref;          // reference point on centreline (from mission)
        Location line_start;   // far behind the aircraft on the centreline
        Location line_end;     // far beyond the reference point
        bool line_valid;
        bool roll_started;
        bool past_v1;
        bool aborted;
        bool we_set_estop;
        bool tracking_msg_sent;
        uint8_t abort_count;   // consecutive cycles with an abort condition
        float max_abs_xtrk;
        uint32_t last_report_ms;
        uint32_t last_refuse_msg_ms;
        uint32_t last_log_ms;
        uint32_t last_clear_msg_ms;
    } state;

    // geometry relative to the runway line. xtrk +ve = right of
    // centreline, along = metres past the reference point (negative
    // when the reference point is still ahead)
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
