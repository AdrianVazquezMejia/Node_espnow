/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_STATION_MODE
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           6

#define ESPNOW_PAYLOAD_SIZE 200
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
    ESPNOW_SEND_CB,
    ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef enum {
    ESPNOW_TO_SEND,
    ESPNOW_FROM_CB,
} espnow_send_id_t;

typedef struct {
	uint8_t len;                              /* length of the data*/
	uint8_t *data;								/* pointing to the data*/
	uint8_t dir;								/* direction od the data*/
} __attribute__((packed)) esp_uart_data_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef union {
	espnow_event_send_cb_t send_cb;
	esp_uart_data_t send_data;
} espnow_send_data_t;

typedef struct {
	espnow_send_data_t info;
	espnow_send_id_t id;
} espnow_send_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_send_t send_t;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    EXAMPLE_ESPNOW_DATA_MAX,
};

enum{
	FORDWARD = 0,
	BACKWARD
};
enum{
	WRITE_COIL =5,
	WRITE_HOLDING_REGISTER
};
enum{
	READ_HOLDING =3,

};
enum{
	EX_SLAVE,
	NODE,

};

enum{
	SERIAL,
	NODECONFIG,
	JUMP
};
enum{
	ESP_NOW,
	UART
};

enum {
	RESTART,
	SAVE_RAM,
	SAVE_FLASH
} ;
typedef union
{
    uint16_t Val;
    struct
    {
        uint8_t LB;
        uint8_t HB;
    } byte;

} INT_VAL;
/* User defined field of ESPNOW data in this example. */
typedef struct {
    uint8_t type;                         //Broadcast or unicast ESPNOW data.
    bool state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint16_t seq_num;                     //Sequence number of ESPNOW data.
    uint16_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t dir;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint8_t Nodeid;
    uint8_t payload[200];
    uint8_t data_len;//Real payload of ESPNOW data.
} __attribute__((packed)) espnow_data_t;

/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
    bool state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint32_t dir;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    uint16_t delay;                       //Delay between sending two ESPNOW data, unit: ms.
    uint8_t len;                              //Length of ESPNOW data to be sent, unit: byte.
    uint8_t *buffer;                      //Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} espnow_send_param_t;

/**
 * @ struct for that data ESPNOW <-> UART
 */



#endif
