#pragma once

#include <stdbool.h>

bool mqtt_init(const char *broker_ip);
bool mqtt_publish_test(void);