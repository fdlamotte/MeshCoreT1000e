#pragma once

#include <MicroNMEA.h>

extern MicroNMEA nmea;

extern void gps_setup();
extern void gps_feed_nmea();
extern void gps_loop();