#ifndef MYTIMER_H_
#define MYTIMER_H_

#include "include.h"

void ConfigIntervalTimer(uint32_t TimerIntervalms);
void ConfigUpdateTimer(uint32_t TimerIntervalms);

void TimerInterruptHandler(void);
void TimerUpdateHandler(void);

void CalculateAngleRMS(double setvalue1, double getvalue1, double setvalue2, double getvalue2, double setvalue3, double getvalue3);


#endif /* MYTIMER_H_ */
