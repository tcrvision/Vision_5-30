//
// Created by czh on 2021/5/5.
//

#ifndef TCR_WINDMILL_SYSTIME_H
#define TCR_WINDMILL_SYSTIME_H
typedef double systime;

void getsystime(systime &t);
double getTimeIntervalms(const systime &now, const systime &last);
#include <sys/time.h>

#endif //TCR_WINDMILL_SYSTIME_H
