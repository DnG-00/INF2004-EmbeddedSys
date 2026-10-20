#ifndef MQTT_TOPICS_H
#define MQTT_TOPICS_H

// Topics used for the PICO W to publish data from the car's sensors
#define TOPIC_ULTRASONIC "car/telemetry/ultrasonic"
#define TOPIC_IMU "car/telemetry/imu"
#define TOPIC_MOTION "car/telemetry/motion"
#define TOPIC_LINE "car/telemetry/line"
#define TOPIC_BARCODE "car/telemetry/barcode"

// Topic(s) used to check if the heartbeat of WIFI and MQTT connections are still active
#define TOPIC_WIFI "car/status/wifi" 
#define TOPIC_MQTT "car/status/mqtt" 

#endif
