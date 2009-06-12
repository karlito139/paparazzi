#ifndef CSC_AP_LINK_H
#define CSC_AP_LINK_H

#include "std.h"
#include "string.h"
#include "csc_can.h"
#include "csc_msg_def.h"


extern int32_t csc_ap_link_error_cnt;

void csc_ap_link_init(void);
void csc_ap_send_msg(uint8_t msg_id, const uint8_t *buf, uint8_t len);
void csc_ap_link_send_status(uint32_t loops, uint32_t msgs);
void csc_ap_link_send_adc(float adc1, float adc2);
void csc_ap_link_set_servo_cmd_cb(void (* cb)(struct CscServoCmd *cmd));
void csc_ap_link_set_motor_cmd_cb(void (* cb)(struct CscMotorMsg *msg));
void csc_ap_link_set_rc_cmd_cb(void (* cb)(struct CscRCMsg *msg));

#endif /* CSC_AP_LINK_H */

