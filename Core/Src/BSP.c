/*
 * BSP.c
 *
 *  Created on: May 3, 2026
 *      Author: HP
 */
#include <math.h>

float getAltitude(double press, double temp) {
  return ((pow((1013.25 / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}
