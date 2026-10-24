#ifndef FEEDER_CONTROLLER_H
#define FEEDER_CONTROLLER_H

#include "can_frame.h"
#include <stdint.h>

#define FEEDER_ENCODER_COUNTS 8192U
#define FEEDER_FEEDBACK_TIMEOUT_MS 20U
#define FEEDER_CONTROL_MAX_GAP_MS 10U
#define FEEDER_JUMP_SLACK_COUNTS 256U
/* CAN receipt uses integer HAL_GetTick(): a difference of N ticks can span
 * almost N+1 milliseconds. This margin is only for encoder plausibility. */
#define FEEDER_GAP_UNCERTAINTY_MS 1U
#define FEEDER_STREAM_MAX_PENDING 2U
#define FEEDER_CURRENT_PROTOCOL_LIMIT 10000
#define FEEDER_CURRENT_CONFIG_LIMIT 3000

typedef enum {
    FEEDER_FAULT_NONE = 0U,
    FEEDER_FAULT_FEEDBACK = 1U << 0,
    FEEDER_FAULT_DATA = 1U << 1,
    FEEDER_FAULT_JUMP = 1U << 2,
    FEEDER_FAULT_OVERSPEED = 1U << 3,
    FEEDER_FAULT_CURRENT = 1U << 4,
    FEEDER_FAULT_CAN = 1U << 5,
    FEEDER_FAULT_TIMEOUT = 1U << 6,
    FEEDER_FAULT_JAM = 1U << 7,
    FEEDER_FAULT_TIMING = 1U << 8
} FeederFault;

typedef struct {
    /* This project's measured installation: 40 output degrees, 35.68 rotor
     * turns/output turn, clockwise viewed from above is encoder-negative.
     * Compiled calibration never grants a zero or ARM on boot. */
    float step_deg, gear_ratio;
    int8_t direction;
    uint8_t calibrated;
    /* Position error is rotor degrees, position P output is rotor RPM;
     * speed PI output/current_limit are C610 raw command units, not amperes. */
    float pos_kp, speed_kp, speed_ki;
    float max_rpm, max_feedback_rpm, current_limit;
    float continuous_max_rpm, continuous_brake_kp, continuous_phase_ms;
    /* tolerance_deg is controlled-shaft degrees; all RPM are rotor RPM. */
    float tolerance_deg, settle_rpm, jam_rpm, jam_current_raw;
    uint32_t settle_ms, motion_timeout_ms, jam_ms;
} FeederConfig;

enum {
    FEEDER_JUMP_DELTA = 1U,
    FEEDER_JUMP_HALF_TURN = 2U
};
/* First suspect sample since the last successful CLEAR. Capture before rebase;
 * later samples/STOP cannot replace it. events counts jumps since reset. */
typedef struct {
    uint32_t events, at_ms, gap_ms, bound_gap_ms, age_ms, sequence, flags;
    uint16_t previous_angle, angle;
    int16_t previous_rpm, rpm, feedback_raw, command_raw;
    int32_t delta_counts;
    float limit_counts, expected_counts;
    uint8_t valid, busy, zero_valid, data[8];
} FeederJumpDiagnostic;

typedef enum {
    FEEDER_MOTION_IDLE = 0U, FEEDER_MOTION_SINGLE,
    FEEDER_MOTION_STREAM, FEEDER_MOTION_TAIL
} FeederMotionKind;

typedef struct {
    FeederConfig config;
    FeederJumpDiagnostic jump;
    int64_t accumulated_counts, zero_counts;
    double position_deg, target_deg;
    double axis_position_deg, axis_target_deg;
    double stream_reference_deg, next_step_deg;
    uint16_t raw_angle;
    int16_t raw_rpm, raw_current, command_raw;
    float target_rpm, integral;
    uint32_t faults, bus_epoch, last_feedback_ms, feedback_sequence;
    uint32_t timestamp_allowances;
    uint32_t last_control_ms, last_control_sequence, last_pi_feedback_ms;
    uint32_t motion_started_ms, settle_started_ms, jam_started_ms;
    uint32_t last_shot_sequence, accepted, completed, rejected, aborted;
    uint32_t cancelled, single_completed, continuous_completed;
    uint32_t pending_started_ms[FEEDER_STREAM_MAX_PENDING];
    uint32_t last_motion_ms, last_completed_ms, burst_first_ms, burst_completed, burst_elapsed_ms;
    uint32_t stream_last_event_ms;
    float measured_hz, last_motion_error_deg;
    uint8_t have_feedback, have_zero, busy, have_control_tick;
    uint8_t settling, jamming, have_shot_sequence, recovery_samples;
    uint8_t motion_kind, outstanding, rate_blocked;
} FeederController;

void Feeder_Init(FeederController *controller);
uint8_t Feeder_ConfigValid(const FeederConfig *config);
/* Configuration updates require idle; changing shaft units/direction or the
 * acknowledgement invalidates the zero and requires another explicit Zero. */
uint8_t Feeder_SetConfig(FeederController *controller, const FeederConfig *config);
/* Only standard CAN2 0x205 with DLC exactly 8 is accepted. C610 bytes 6/7 are
 * reserved; no temperature is invented from these bytes. now_ms is receipt
 * processing time; frame received_ms and fault_epoch must match live context. */
uint8_t Feeder_OnFrame(FeederController *controller, const CanFrame *frame,
                      uint32_t now_ms, uint32_t bus_epoch);
/* Return the C610 raw command for CAN2 0x1FF bytes 0/1. Requests are one-shot
 * sequence events, never a queue. Real wheel readiness gates each new shot;
 * an accepted shot may finish while wheel speed recovers. allowed is still
 * required on every tick, including fresh feedback and all remote interlocks. */
int16_t Feeder_Step(FeederController *controller, uint32_t now_ms,
                    uint8_t allowed, uint8_t wheels_ready,
                    uint8_t shot_event, uint32_t sequence);
/* continuous is a live, release-qualified held-trigger signal. At most two
 * already accepted boundaries may be outstanding; late/rejected pulses are
 * never queued. Release/speed dip cancels lookahead and finishes one boundary;
 * loss of allowed/fault still cancels immediately. */
int16_t Feeder_Run(FeederController *controller, uint32_t now_ms,
                   uint8_t allowed, uint8_t wheels_ready,
                   uint8_t shot_event, uint32_t sequence, uint8_t continuous);
float Feeder_ContinuousRpm(const FeederController *controller);
/* Stop cancels target, integrator and an in-flight shot immediately; it does
 * not invalidate healthy calibration/zero. No future call resumes old work. */
void Feeder_Stop(FeederController *controller);
/* Clear is explicit, idle and requires fresh healthy stationary feedback.
 * A fault always loses the zero; Clear cannot resume an interrupted step. */
uint8_t Feeder_Clear(FeederController *controller, uint32_t now_ms);
uint8_t Feeder_Zero(FeederController *controller, uint32_t now_ms);

#endif
