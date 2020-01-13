/* 
 * File:   macros.h
 * Author: Zac
 *
 * Created on April 30, 2017, 12:46 AM
 */

#ifndef MACROS_H
#define	MACROS_H
#include <stdbool.h>
#ifdef	__cplusplus
extern "C" {
#endif

void updateMacroCommunications(void);
int getPerformNavigationCommand(void);
void setPerformNavigationCommand(int c);
void setNavigationCommandData(int d);
int getNavigationCommandData(void);
void doNavigationCommand(void);
void stopNavigationCommand(void);
void turnDegrees(float _degrees);
void CAN_ResumeMotor(int _newDist);
void CAN_PauseMotor();
void travelDistance(void);
void updateMotors(int _Dir,float updatedAngle);
void moveMotors(float _degrees,float currentAngle);
void StartMotorEncoderMode();
void StopMotorCommand(void);
int CAN_getAVGDistTraveled();
void setPauseData(int d);
void MacroModeComplete();
 
#define TURN_COMMAND    1
#define ANGLE_COMMAND   2
 



#ifdef	__cplusplus
}
#endif

#endif	/* MACROS_H */

