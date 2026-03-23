#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

extern int maxServoValue;
extern int minServoValue;

void InitServo();
void RunServo();

#ifdef __cplusplus
}
#endif

#endif // SERVO_H