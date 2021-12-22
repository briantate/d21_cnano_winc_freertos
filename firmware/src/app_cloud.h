/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_cloud.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_CLOUD_Initialize" and "APP_CLOUD_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_CLOUD_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _APP_CLOUD_H
#define _APP_CLOUD_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "definitions.h"
#include "configuration.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
#define WLAN_SSID           "NotForYou"//"UNITE-5ACA"
#define WLAN_AUTH_WPA_PSK
//#define WLAN_AUTH_OPEN
#define WLAN_PSK            "BTHotspot1!"//"32757028"

#define MQTT_BROKER_NAME "broker.hivemq.com"
#define TCP_SERVER_IP_ADDR  "192.168.1.3"
#define TCP_SERVER_PORT_NUM 80

#define TCP_BUFFER_SIZE     1460
#define TCP_SEND_MESSAGE    "TCP message from WINC module."
    
#define MQTT_BROKER_ENDPOINT "broker.hivemq.com"
#define MQTT_BROKER_PORT ( 1883 )
    
// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    APP_WIFI_STATE_INIT=0,
    APP_WIFI_STATE_INIT_READY,
    APP_WIFI_STATE_CONFIG_STACK,
    APP_WIFI_STATE_CONNECTING,
    APP_WIFI_STATE_CONNECTED,
    APP_WIFI_STATE_DISCONNECTED,
    APP_WIFI_STATE_ERROR,
} WIFI_STATES;

typedef enum
{
    APP_MQTT_STATE_WAIT_FOR_NETWORK=0,
    APP_MQTT_STATE_WAIT_FOR_DNS,
    APP_MQTT_STATE_CONNECT_SERVER,
    APP_MQTT_STATE_CONNECT_BROKER,
    APP_MQTT_STATE_SUBSCRIBE,
    APP_MQTT_STATE_BROKER_CONNECTED,
    APP_MQTT_STATE_ERROR,
} MQTT_STATES;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    WIFI_STATES wifiState;
    MQTT_STATES mqttState;
    bool isNetworkConnected;
    bool isBrokerAddressResolved;
    bool isServerConnected;
    bool isMqttBrokerConnected;
    bool isTimeToPublish;
    /* TODO: Define any additional data used by the application. */

} APP_CLOUD_DATA;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_CLOUD_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    APP_CLOUD_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_CLOUD_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_CLOUD_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_CLOUD_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_CLOUD_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_CLOUD_Tasks( void );

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APP_CLOUD_H */

/*******************************************************************************
 End of File
 */

