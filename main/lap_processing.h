#ifndef LAP_PROCESSING_H
#define LAP_PROCESSING_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "data_structures.h"

bool check_line_crossing(Point car_last_pos, Point car_current_pos, Point line_start_point, Point line_end_point);

void process_virtual_lap(Point current_pos, Point last_pos);

void process_nmea_line(const char *line)

#endif