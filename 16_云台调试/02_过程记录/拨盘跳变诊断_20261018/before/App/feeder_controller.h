#ifndef FEEDER_CONTROLLER_H
#define FEEDER_CONTROLLER_H

#include "can_frame.h"
#include <stdint.h>

#define FEEDER_ENCODER_COUNTS 8192U
#define FEEDER_FEEDBACK_TIMEOUT_MS 20U
#define FEEDER_CONTROL_MAX_GAP_MS 10U
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
    /* step_deg is the user-confirmed output-shaft 40 degrees per shot; the
     * feeder is coaxial. gear_ratio is rotor turns / output-shaft turns and
     * must still be measured. Defaults 1, direction 0, calibrated 0 prohibit motion.
     * calibrated is a RAM-only acknowledgement, never a persisted calibration. */
    float step_deg, gear_ratio;
    int8_t direction;
    uint8_t calibrated;
    /* Position error is rotor degrees, position P output is rotor RPM;
     * speed PI output/current_limit are C610 raw command units, not amperes. */
    float pos_kp, speed_kp, speed_ki;
    float max_rpm, max_feedback_rpm, current_limit;
    /* tolerance_deg is controlled-shaft degrees; all RPM are rotor RPM. */
    float tolerance_deg, settle_rpm, jam_rpm, jam_current_raw;
    uint32_t settle_ms, motion_timeout_ms, jam_ms;
} FeederConfig;

typedef struct {
    FeederConfig config;
    int64_t accumulated_counts, zero_counts;
    double position_deg, target_deg;
    double axis_position_deg, axis_target_deg;
    uint16_t raw_angle;
    int16_t raw_rpm, raw_current, command_raw;
    float target_rpm, integral;
    uint32_t faults, bus_epoch, last_feedback_ms, feedback_sequence;
    uint32_t last_control_ms, last_control_sequence, last_pi_feedback_ms;
    uint32_t motion_started_ms, settle_started_ms, jam_started_ms;
    uint32_t last_shot_sequence, accepted, completed, rejected, aborted;
    uint8_t have_feedback, have_zero, busy, have_control_tick;
    uint8_t settling, jamming, have_shot_sequence, recovery_samples;
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
 * sequence events, never a queue. allowed and wheels_ready must both be true.
 * A commissioned jog uses the same guards with authorization in its backend. */
int16_t Feeder_Step(FeederController *controller, uint32_t now_ms,
                    uint8_t allowed, uint8_t wheels_ready,
                    uint8_t shot_event, uint32_t sequence);
/* Stop cancels target, integrator and an in-flight shot immediately; it does
 * not invalidate healthy calibration/zero. No future call resumes old work. */
void Feeder_Stop(FeederController *controller);
/* Clear is explicit, idle and requires fresh healthy stationary feedback.
 * A fault always loses the zero; Clear cannot resume an interrupted step. */
uint8_t Feeder_Clear(FeederController *controller, uint32_t now_ms);
uint8_t Feeder_Zero(FeederController *controller, uint32_t now_ms);

#endif
