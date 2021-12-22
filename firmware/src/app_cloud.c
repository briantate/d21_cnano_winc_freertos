/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_cloud.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app_cloud.h"
#include <stdint.h>
#include "system/debug/sys_debug.h"
#include "peripheral/port/plib_port.h"
#include "system/time/sys_time.h"
#include "wdrv_winc_client_api.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

extern SYSTEM_OBJECTS sysObj; //WiFi stack, Timer, etc...

APP_CLOUD_DATA app_cloudData;
DRV_HANDLE wdrvHandle;
static SOCKET mqttClientSocket;
uint32_t brokerAddress;
//static uint8_t tcpRxBuffer[256]; //placeholder for receiving data

// *****************************************************************************
// *****************************************************************************
// Section: Private function prototypes
// *****************************************************************************
// *****************************************************************************
 
//state machines
static void WifiConnectionStateMachine(void);
static void MqttBrokerStateMachine(void);

//user application
static void CommunicateWithCloud(void);

//callbacks
static void WifiConnectionCallback(DRV_HANDLE handle, WDRV_WINC_ASSOC_HANDLE assocHandle, WDRV_WINC_CONN_STATE currentState, WDRV_WINC_CONN_ERROR errorCode);
static void WifiSocketEventCallback(SOCKET socket, uint8_t eventType, void *pMessage);
static void DhcpAddressEventCallback (DRV_HANDLE handle,uint32_t ipAddress );
static void DnsResolveCallback (uint8_t* domainName, uint32_t serverIp);
//static void MqttPublishTimerCallback(void);

//helper functions
static void parseMqttSocketEvents(uint8_t eventType,void *pMessage);
static void DelayBlocking_MS(uint32_t ms);   

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

void APP_CLOUD_Initialize ( void )
{
    app_cloudData.wifiState = APP_WIFI_STATE_INIT;
    app_cloudData.isNetworkConnected = false;
    app_cloudData.isBrokerAddressResolved = false;
    app_cloudData.isServerConnected = false;
    app_cloudData.isMqttBrokerConnected = false;
    app_cloudData.isTimeToPublish = false;
    app_cloudData.mqttState = APP_MQTT_STATE_CONNECT_SERVER;
}

void APP_CLOUD_Tasks ( void )
{
    //ToDo: is this really 2 state machines, or should it just be one?
    WifiConnectionStateMachine();
//    MqttBrokerStateMachine();
//    CommunicateWithCloud();
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

static void DhcpAddressEventCallback (DRV_HANDLE handle, uint32_t ipAddress )
{
    char s[20];
    SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "My IP address: %s\r\n", inet_ntop(AF_INET, &ipAddress, s, sizeof(s)));
}

static void WifiConnectionCallback(DRV_HANDLE handle, WDRV_WINC_ASSOC_HANDLE assocHandle, WDRV_WINC_CONN_STATE currentState, WDRV_WINC_CONN_ERROR errorCode)
{
    if (WDRV_WINC_CONN_STATE_CONNECTED == currentState)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "WiFi Connected\r\n");
        LED_On();
        app_cloudData.wifiState = APP_WIFI_STATE_CONNECTED;
        app_cloudData.isNetworkConnected = true;
    }
    else if (WDRV_WINC_CONN_STATE_DISCONNECTED == currentState)
    {
        app_cloudData.isNetworkConnected = false;
        LED_Off();
        if (APP_WIFI_STATE_CONNECTING == app_cloudData.wifiState)
        {
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "WiFi Failed to connect\r\n");
            app_cloudData.wifiState = APP_WIFI_STATE_CONFIG_STACK;
        }
        else
        {
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "WiFi Disconnected\r\n");

            if (-1 != mqttClientSocket)
            {
                shutdown(mqttClientSocket);
                mqttClientSocket = -1;
            }

            app_cloudData.wifiState = APP_WIFI_STATE_DISCONNECTED;
        }
    }
}

static void DnsResolveCallback (uint8_t* domainName, uint32_t serverIp)
{
    brokerAddress = serverIp;
    char s[20];
    SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "Broker IP address: %s\r\n", inet_ntop(AF_INET, &serverIp, s, sizeof(s)));
    app_cloudData.isBrokerAddressResolved = true;
}

static void parseMqttSocketEvents(uint8_t eventType,void *pMessage)
{
    switch(eventType)
    {
        case SOCKET_MSG_CONNECT:
        {
            //ToDo:
            break;
        }
        case SOCKET_MSG_SEND:
        {
            //ToDo: 
            break;
        }
        case SOCKET_MSG_RECV:
        {
            //ToDo:
            break;
        }
        default:
        {
            break;
        }
    }
}


static void WifiSocketEventCallback(SOCKET socket, uint8_t eventType, void *pMessage)
{
    if(socket == mqttClientSocket)
        parseMqttSocketEvents(eventType, pMessage);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


//static void prvCreateMQTTConnectionWithBroker( MQTTContext_t * pxMQTTContext,
//                                               NetworkContext_t * pxNetworkContext )
//{
//    MQTTStatus_t xResult;
//    MQTTConnectInfo_t xConnectInfo;
//    bool xSessionPresent;
//    TransportInterface_t xTransport;
//
//    /***
//     * For readability, error handling in this function is restricted to the use of
//     * asserts().
//     ***/
//
//    /* Fill in Transport Interface send and receive function pointers. */
//    xTransport.pNetworkContext = pxNetworkContext;
//    xTransport.send = send; //WINC1500 TCP/IP send function
//    xTransport.recv = recv; //WINC1500 TCP/IP recv function
//
//    /* Initialize MQTT library. */
//    xResult = MQTT_Init( pxMQTTContext, &xTransport, prvGetTimeMs, prvEventCallback, &xBuffer );
//    configASSERT( xResult == MQTTSuccess );
//
//    /* Many fields not used in this demo so start with everything at 0. */
//    ( void ) memset( ( void * ) &xConnectInfo, 0x00, sizeof( xConnectInfo ) );
//
//    /* Start with a clean session i.e. direct the MQTT broker to discard any
//     * previous session data. Also, establishing a connection with clean session
//     * will ensure that the broker does not store any data when this client
//     * gets disconnected. */
//    xConnectInfo.cleanSession = true;
//
//    /* The client identifier is used to uniquely identify this MQTT client to
//     * the MQTT broker. In a production device the identifier can be something
//     * unique, such as a device serial number. */
//    xConnectInfo.pClientIdentifier = democonfigCLIENT_IDENTIFIER;
//    xConnectInfo.clientIdentifierLength = ( uint16_t ) strlen( democonfigCLIENT_IDENTIFIER );
//
//    /* Set MQTT keep-alive period. It is the responsibility of the application to ensure
//     * that the interval between Control Packets being sent does not exceed the Keep Alive value.
//     * In the absence of sending any other Control Packets, the Client MUST send a PINGREQ Packet. */
//    xConnectInfo.keepAliveSeconds = mqttexampleKEEP_ALIVE_TIMEOUT_SECONDS;
//
//    /* Send MQTT CONNECT packet to broker. LWT is not used in this demo, so it
//     * is passed as NULL. */
//    xResult = MQTT_Connect( pxMQTTContext,
//                            &xConnectInfo,
//                            NULL,
//                            mqttexampleCONNACK_RECV_TIMEOUT_MS,
//                            &xSessionPresent );
//    configASSERT( xResult == MQTTSuccess );
//}
/*-----------------------------------------------------------*/
static void DelayBlocking_MS(uint32_t ms)
{
    SYS_TIME_HANDLE timer = SYS_TIME_HANDLE_INVALID;

    if (SYS_TIME_DelayMS(ms, &timer) != SYS_TIME_SUCCESS)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "blocking timer error\r\n");
        // Handle error
    }
    else if(SYS_TIME_DelayIsComplete(timer) != true)
    {
        // Wait till the delay has not expired
        while (SYS_TIME_DelayIsComplete(timer) == false);
    }
}

static void MqttBrokerStateMachine(void)
{
    //test
    return;
    //end test
    
    switch(app_cloudData.mqttState)
    {
        case APP_MQTT_STATE_WAIT_FOR_NETWORK:
        {
            if(app_cloudData.isNetworkConnected)
            {
                gethostbyname(MQTT_BROKER_NAME);
                app_cloudData.mqttState = APP_MQTT_STATE_WAIT_FOR_DNS;
            }
            break;
        }
        case APP_MQTT_STATE_WAIT_FOR_DNS:
        {
            if(app_cloudData.isBrokerAddressResolved)
            {
                app_cloudData.mqttState = APP_MQTT_STATE_CONNECT_SERVER;
            }
            break;
        }
        case APP_MQTT_STATE_CONNECT_SERVER:
        {
            mqttClientSocket = socket(AF_INET, SOCK_STREAM, 0);
            if (mqttClientSocket >= 0)
            {
                struct sockaddr_in addr;

                /* Connect the socket to the server. */

                addr.sin_family = AF_INET;
                addr.sin_port = _htons(TCP_SERVER_PORT_NUM);
                addr.sin_addr.s_addr = brokerAddress;

                if (connect(mqttClientSocket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) >= 0)
                    app_cloudData.mqttState = APP_MQTT_STATE_CONNECT_BROKER;
                else
                    app_cloudData.mqttState = APP_MQTT_STATE_WAIT_FOR_NETWORK;
            }
            break;
        }
        case APP_MQTT_STATE_CONNECT_BROKER:
        {
            if(app_cloudData.isServerConnected)
            {
                TurnOnMqttLed();
                /* Connect to the MQTT broker using the already connected TCP socket. */
//                    prvCreateMQTTConnectionWithBroker( &xMQTTContext, &xNetworkContext );
                app_cloudData.mqttState = APP_MQTT_STATE_SUBSCRIBE;
            }
            break;
        }

        case APP_MQTT_STATE_SUBSCRIBE:
        {
            if(app_cloudData.isMqttBrokerConnected)
            {
//                prvMQTTSubscribeWithBackoffRetries( &xMQTTContext );
                app_cloudData.mqttState = APP_MQTT_STATE_BROKER_CONNECTED;
            }
            break;
        }
        case APP_MQTT_STATE_BROKER_CONNECTED:
        {
            app_cloudData.isMqttBrokerConnected = true;
        }
        case APP_MQTT_STATE_ERROR:
        {
            //ToDo: cleanup
//            TurnOnErrorLed();
            app_cloudData.mqttState = APP_MQTT_STATE_CONNECT_SERVER;
            break;
        }
        default:
        {
//            TurnOnErrorLed();
            break;
        }
    }
}

static void WifiConnectionStateMachine(void)
{
    /* Check the application's current state. */
    switch ( app_cloudData.wifiState )
    {
        /* Application's initial state. */
        case APP_WIFI_STATE_INIT:
        {
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "[APP_WIFI]Global Sensor Network Demo\r\n");
            LED_Off();
            if (SYS_STATUS_READY == WDRV_WINC_Status(sysObj.drvWifiWinc))
            {
                app_cloudData.wifiState = APP_WIFI_STATE_INIT_READY;
            }
        }
        
        case APP_WIFI_STATE_INIT_READY:
        {
            
            wdrvHandle = WDRV_WINC_Open(0, 0);

            if (DRV_HANDLE_INVALID != wdrvHandle)
            {
                SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "[APP_WIFI]WDRV Initialized\r\n");
                mqttClientSocket = -1;
                app_cloudData.wifiState = APP_WIFI_STATE_CONFIG_STACK;
            }
            break;
        }
        
        case APP_WIFI_STATE_CONFIG_STACK:
        {
            WDRV_WINC_AUTH_CONTEXT authCtx;
            WDRV_WINC_BSS_CONTEXT  bssCtx;

            /* Enable use of DHCP for network configuration, DHCP is the default
             but this also registers the callback for notifications. */
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "[APP_WIFI]WDRV use DHCP\r\n");
            WDRV_WINC_IPUseDHCPSet(wdrvHandle, &DhcpAddressEventCallback);
            registerSocketCallback(WifiSocketEventCallback, DnsResolveCallback);
            
            /* Preset the error state incase any following operations fail. */

            app_cloudData.wifiState = APP_WIFI_STATE_ERROR;

            /* Initialize the BSS context to use default values. */
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "[APP_WIFI]WDRV BSS defaults\r\n");
            
            if (WDRV_WINC_STATUS_OK != WDRV_WINC_BSSCtxSetDefaults(&bssCtx))
            {
                break;
            }

            /* Update BSS context with target SSID for connection. */
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "[APP_WIFI]WDRV set BSS SSID\r\n");
            
            if (WDRV_WINC_STATUS_OK != WDRV_WINC_BSSCtxSetSSID(&bssCtx, (uint8_t*)WLAN_SSID, strlen(WLAN_SSID)))
            {
                break;
            }
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "[APP_WIFI]WDRV set auth\r\n");
            
            DelayBlocking_MS(1000); //TODO: Why is this needed??? SPI fails without it
            
            if (WDRV_WINC_STATUS_OK != WDRV_WINC_AuthCtxSetWPA(&authCtx, (uint8_t*)WLAN_PSK, strlen(WLAN_PSK)))
            {
                break;
            }
            
            /* Connect to the target BSS with the chosen authentication. */
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "[APP_WIFI]WDRV BSS connect\r\n");
            if (WDRV_WINC_STATUS_OK == WDRV_WINC_BSSConnect(wdrvHandle, &bssCtx, &authCtx, &WifiConnectionCallback))
            {
                SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "[APP_WIFI] WDRV Connecting\r\n");
                app_cloudData.wifiState = APP_WIFI_STATE_CONNECTING;
            }

            /* Register callback for socket events. */
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "[APP_WIFI] register socket callback\r\n");
            WDRV_WINC_SocketRegisterEventCallback(wdrvHandle, &WifiSocketEventCallback);
            break;

        }
        
        case APP_WIFI_STATE_CONNECTING:
        {
            static uint8_t tmp = 0;
            if(0 == tmp)
            {
                SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "[APP_WIFI] waiting for connection\r\n");
                tmp = 1;
            }
            /* Wait for the BSS connect to trigger the callback and update
             the connection state. */
            break;
        }
        
        case APP_WIFI_STATE_CONNECTED:
        {
            break;
        }
        
        case APP_WIFI_STATE_ERROR:
        {
            //ToDo: handle errors
            break;
        }
        
        default:
        {
            break;
        }
    }
}

//Placeholder for determining when to publish
//static void MqttPublishTimerCallback(void)
//{
//    if(app_cloudData.isMqttBrokerConnected)
//        app_cloudData.isTimeToPublish = true;
//}

static void CommunicateWithCloud(void)
{
    if(app_cloudData.isTimeToPublish)
        asm("NOP"); //            MQTT_Publish();
}
/*******************************************************************************
 End of File
 */
