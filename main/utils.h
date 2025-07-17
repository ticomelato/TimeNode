#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "data_structures.h"

int calc_orientation(Point p, Point q, Point r);

void print_time(const char *label, int64_t start_time, int64_t end_time);

double nmea_to_decimal(const char *coord, char direction);

#endif