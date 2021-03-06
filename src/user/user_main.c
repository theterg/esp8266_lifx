/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "esp_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/udp.h"
#include "lwip/netbuf.h"
#include "lwip/api.h"

#include "gpio.h"

#include "user_config.h"
#include "lifx_interface.h"

// UDP Broadcast to all devices on port 56700 (LIFX protocol)
#define BROADCAST_ADDR "255.255.255.255"
#define BROADCAST_PORT 56700

// Uncomment to print all received packet contents to the debug UART
//#define PKT_DEBUG

#define BUTTON_GPIO_REG PERIPHS_IO_MUX_GPIO0_U
#define BUTTON_GPIO_NUM 0
#define BUTTON_DEBOUNCE_TIME 25  // Measured in FreeRTOS Ticks

#define LED_GPIO_REG PERIPHS_IO_MUX_GPIO5_U
#define LED_GPIO_NUM 5
#define LED_OFF() GPIO_OUTPUT_SET(LED_GPIO_NUM, 1)
#define LED_ON() GPIO_OUTPUT_SET(LED_GPIO_NUM, 0)

// What bulb should we toggle?
// If CONTROL_ALL_BULBS is uncommented, all detected lights will be toggled
// #define CONTROL_ALL_BULBS
// Otherwise use the following MAC address:
const uint8_t dest_mac[6] = {0xd0,0x73,0xd5,0x11,0x49,0x72};

typedef struct {
  struct ip_addr addr;
  struct pbuf * p;
} udp_msg_t;

uint8_t network_ready = 0;
xQueueHandle msgqueue = 0;
xQueueHandle buttonevt = 0;
uint32_t last_button_press = 0;
ip_addr_t myaddr;

// Callback executed from within the lwip subsystem
// WARN: Don't know where from, could be an ISR!
// discover_bulbs() engages and disengages this callback
void my_udp_recv(void * arg, struct udp_pcb * pcb, struct pbuf *p,
            struct ip_addr * addr, u16_t port) {
    int ret;
    udp_msg_t msg;
    if (p != NULL) {
      // We've received a new message
      // It's not a good idea to do much work in this function,
      // queue up the message and handle it elsewhere
      memcpy(&msg.addr, addr, sizeof(struct ip_addr));
      msg.p = p;
      if (!xQueueSend( msgqueue, &msg, 1000/portTICK_RATE_MS)) {
        os_printf("[discover] Couldn't queue!\r\n");
      }
      // WARN: parser MUST free the pbuf once it's been consumed!
    }
}


void ICACHE_FLASH_ATTR
msgparse_task(void *pvParameters)
{
    udp_msg_t msg;
    struct pbuf *q;
    char * data;
    int i;

    while(1) {
      if (xQueueReceive( msgqueue, &msg, portMAX_DELAY) == pdTRUE) {
        // A message was recvd!
        // Copy pbuf pointer so we can dive into it
        q = msg.p;
        // Iterate over each pbuf in the chain
        // Each time replacing q with q->next
        do {
          parseBroadcastMsg(msg.addr, (uint8_t *)q->payload, q->len);
#ifdef PKT_DEBUG
          os_printf("[RECV] (%d:%d) ", msg.p->len, msg.p->tot_len);
          data = (uint8_t *)q->payload;
          for (i=0;i < q->len; i++) {
            os_printf("%02x", data[i]);
          }
          os_printf("\r\n");
#endif //PKT_DEBUG

          q = q->next;
        } while(q != NULL);
        // MUST free the pbuf or we risk running out of RAM
        pbuf_free(msg.p);
      }
    }
    vTaskDelete(NULL);
}

void ICACHE_FLASH_ATTR
discover_bulbs(void) {
    struct udp_pcb *pcb;
    uint8_t buff[36];
    struct pbuf *p;
    err_t ret;

    LED_ON();
    // Using the RAW interface to LWIP
    // the socket and netconn interfaces didn't receive broadcast packets
    pcb = udp_new();
    ret = udp_bind(pcb, IP_ADDR_ANY, BROADCAST_PORT);
    if (ret != ERR_OK) {
      os_printf("[discover] Error bind %d\r\n", ret);
      return;
    }
    udp_recv(pcb, my_udp_recv, NULL);

    // Create pbuf for a new outgoing packet
    p = pbuf_alloc(PBUF_TRANSPORT, 36, PBUF_RAM);
    if (p == NULL) {
      os_printf("[discover] error pbuf_alloc %d\r\n", ret);
      return;
    }

    getBroadcastPkt((uint8_t *)p->payload, 36);

    ret = udp_sendto(pcb, p, IP_ADDR_BROADCAST, BROADCAST_PORT);
    if (ret != ERR_OK) {
      os_printf("[discover] Error sending %d\r\n", ret);
      pbuf_free(p);
      return;
    }
    os_printf("[discover] Broadcast sent\r\n");
    pbuf_free(p);
    // MUST wait a given timeout for messages to be received before
    // tearing down the connection - udp_remove() will disable the callback
    vTaskDelay(500/portTICK_RATE_MS);
    LED_OFF();
    vTaskDelay(2000/portTICK_RATE_MS);
    udp_remove(pcb);
}

void ICACHE_FLASH_ATTR registerInterrupt(int pin, GPIO_INT_TYPE mode)
{
   portENTER_CRITICAL();

   // disable open drain
   GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(pin)), 
       GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(pin))) & (~ GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE))); 
   //clear interrupt status
   GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(pin));

        // set the mode   
   gpio_pin_intr_state_set(GPIO_ID_PIN(pin), mode);

   portEXIT_CRITICAL();
}

void ICACHE_FLASH_ATTR
monitor_task(void *pvParameters)
{
    while(1) {
      if (network_ready) {
        discover_bulbs();
        // Check again in approximately 1 minute
        vTaskDelay(10000/portTICK_RATE_MS);
      }
      vTaskDelay(200/portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

void ICACHE_FLASH_ATTR
wifi_handle_event_cb(System_Event_t *evt) {
    // Respond to connection events
    switch(evt->event_id) {
    case EVENT_STAMODE_SCAN_DONE:
      os_printf("[EVT] SCAN_DONE \r\n");
      break;
    case EVENT_STAMODE_CONNECTED:
      os_printf("[EVT] CONNECTED \r\n");
      break;
    case EVENT_STAMODE_DISCONNECTED:
      os_printf("[EVT] DISCONNECTED \r\n");
      network_ready = 0;
      break;
    case EVENT_STAMODE_AUTHMODE_CHANGE:
      os_printf("[EVT] AUTHMODE CHANGE \r\n");
      network_ready = 0;
      break;
    case EVENT_STAMODE_GOT_IP:
      os_printf("[EVT] GOT IP\r\n");
      memcpy(&myaddr, &evt->event_info.got_ip.ip, sizeof(myaddr));
      network_ready = 1;
      break;
    case EVENT_STAMODE_DHCP_TIMEOUT:
      os_printf("[EVT] DHCP_TIMEOUT\r\n");
      network_ready = 0;
      break;
    default:
      os_printf("[EVT] Unknown %x\r\n", evt->event_id);
    }
}


void gpio_intr_handler(void *arg) {
  uint32_t gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
  uint32_t curr_ticks = xTaskGetTickCountFromISR();
  const uint8_t pin = 5;
  // Immediately clear the interrupt status register
  GPIO_REG_WRITE(GPIO_STATUS_ADDRESS, 0);
  if (gpio_status & (1 << BUTTON_GPIO_NUM)) {
    // Button has been pressed - check debounce
    if ((curr_ticks - last_button_press) > BUTTON_DEBOUNCE_TIME) {
      if (!xQueueSendFromISR( buttonevt, &pin, NULL)) {
        os_printf("[gpio] Couldn't queue!\r\n");
      }
      last_button_press = curr_ticks;
    }
  }
}

void ICACHE_FLASH_ATTR
_setBulbState(int idx, uint16_t state) {
  struct netconn * sendconn;
  struct netbuf * sendbuf;
  char * data;
  err_t ret;

  if (!network_ready) return;

  sendconn = netconn_new( NETCONN_UDP );
  if (sendconn == NULL) {
    os_printf("[setbulb] Couldn't get netconn\r\n");
    return;
  }
  sendbuf = netbuf_new();
  if (sendbuf == NULL) {
    os_printf("[setbulb] couldn't get sendbuff\r\n");
    return;
  }
  data = netbuf_alloc(sendbuf, 38);
  if (data == NULL) {
    os_printf("[setbulb] couldn't alloc data\r\n");
    return;
  }

  getPktSetBulbPower(data, 38, idx, state);

  ret = netconn_bind(sendconn, &myaddr, 34435);
  if (ret != ERR_OK) {
    os_printf("[setbulb] connect error: %d\r\n", ret);
    return;
  }

  netconn_set_recvtimeout( sendconn, 1000);
  ret = netconn_connect(sendconn, getBulbAddr(idx), 56700);
  if (ret != ERR_OK) {
    os_printf("[setbulb] connect error: %d\r\n", ret);
    return;
  }
  ret = netconn_send(sendconn, sendbuf);
  if (ret != ERR_OK) {
    os_printf("[setbulb] send error: %d\r\n", ret);
  } else {
  }

  netbuf_free(sendbuf);
  netbuf_delete(sendbuf);
  netconn_disconnect(sendconn);
  netconn_delete(sendconn);
}

void ICACHE_FLASH_ATTR
button_task(void *pvParameters) {
  uint8_t last_light_state = 0;
  uint8_t pin;
  uint8_t i;
  ip_addr_t * addr;

  while(1) {
    if (xQueueReceive( buttonevt, &pin, portMAX_DELAY) == pdTRUE) {
      // If we're disconnected, do nothing!
      if (!network_ready) continue;
#ifdef CONTROL_ALL_BULBS
      // Toggle the state of each bulb
      for (i=0;i<getNumBulbs();i++) {
        addr = getBulbAddr(i);
        os_printf("[button] Set bulb at %d.%d.%d.%d ",
          addr->addr & (0xFFUL << 0), (addr->addr & 0xFF00UL ) >> 8, (addr->addr & 0xFF0000UL) >> 16, (addr->addr & 0xFF000000UL ) >> 24);
        if (last_light_state) {
          os_printf("off\r\n");
          _setBulbState(0, 0);
        } else {
          os_printf("on\r\n");
          _setBulbState(0, 0xFFFF);
        }
      }
#else // !CONTROL_ALL_BULBS
      // Only toggle the stored bulb
      i = getBulbByMAC(dest_mac);
      if (i >= 0) {
        addr = getBulbAddr(i);
        os_printf("[button] Set bulb at %d.%d.%d.%d ",
          addr->addr & (0xFFUL << 0), (addr->addr & 0xFF00UL ) >> 8, (addr->addr & 0xFF0000UL) >> 16, (addr->addr & 0xFF000000UL ) >> 24);
        if (last_light_state) {
          os_printf("off\r\n");
          _setBulbState(i, 0);
        } else {
          os_printf("on\r\n");
          _setBulbState(i, 0xFFFF);
        }
        last_light_state = !last_light_state;
      }
#endif // CONTROL_ALL_BULBS
    }
  }

  vTaskDelete(NULL);
}


/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void ICACHE_FLASH_ATTR
user_init(void)
{
    struct station_config sta_conf = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASSWORD,
        .bssid_set = 0,
        .bssid = ""
    };
    printf("SDK version:%s\n", system_get_sdk_version());

    /* need to set opmode before you set config */
    wifi_set_opmode(STATION_MODE);

    wifi_set_event_handler_cb(wifi_handle_event_cb);

    // Store wifi credentials and start connection
    wifi_station_set_config(&sta_conf);
    wifi_station_connect();

    // Queue will hold pointers to pbuf structures
    // No need to copy memory around
    msgqueue = xQueueCreate( 25, sizeof(udp_msg_t));
    if (msgqueue == NULL) {
      os_printf("ERR: Couldn't create queue!\r\n");
      return;
    }
    buttonevt = xQueueCreate( 25, sizeof(uint8_t));
    if (buttonevt == NULL) {
      os_printf("ERR: Couldn't create buttonevent queue!\r\n");
      return;
    }

    // Start tasks (these will run indefinately)
    xTaskCreate(monitor_task, "mon_task", 1024, NULL, 2, NULL);
    xTaskCreate(msgparse_task, "parse_task", 1024, NULL, 2, NULL);
    xTaskCreate(button_task, "button_task", 256, NULL, 1, NULL);

    // Create button interrupt
    PIN_FUNC_SELECT(BUTTON_GPIO_REG, 0);   // Button pin as GPIO
    PIN_PULLUP_EN(BUTTON_GPIO_REG);        // Enable pullup
    _xt_isr_attach(ETS_GPIO_INUM, (_xt_isr)gpio_intr_handler, NULL);
    _xt_isr_unmask(1<<ETS_GPIO_INUM);      // Enable ALL gpio interrupts
    // Attach button interrupt, falling edge only
    registerInterrupt(BUTTON_GPIO_NUM,  GPIO_PIN_INTR_NEGEDGE);

    PIN_FUNC_SELECT(LED_GPIO_REG, 0);   // Button pin as GPIO
    GPIO_AS_OUTPUT(1 << LED_GPIO_NUM);
    LED_OFF();
}

