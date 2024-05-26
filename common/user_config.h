#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

/** Interval in milliseconds that system should wait if error occupied. */
#define RETRY_CONNECTION_INTERVAL_MS 5000
/** UART speed to terminal. */
#define UART_BAUD_RATE 230400
/** Current firmware version. */
#define FIRMWARE_VERSION "0.1"
/** Git revision */
#ifndef FIRMWARE_GIT_REVISION
#define FIRMWARE_GIT_REVISION "unknown"
#endif

/** SSID of Wi-Fi network for wireless configuration. */
#define WIFI_CONFIGURATION_SSID "LightIvy"
/** Default server url */
#define DEFAULT_SERVER "http://www.lightivy.net/junior"
/** Name for discovering in mDNS. Not more then 60 chars*/
#define MDNS_SERVICE_NAME "_esp8266-lightivy._tcp"
/** HTTP webserver and RESTful service port. */
#define HTTPD_PORT 80

#endif /* _USER_CONFIG_H_ */
