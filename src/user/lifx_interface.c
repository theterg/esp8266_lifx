#include "lifx_interface.h"
#include "esp_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/udp.h"
#include "lwip/api.h"
#include "lwip/netbuf.h"


// *****************************************************************************
// Implementation details glined from the python Lazylights module
// https://github.com/mpapi/lazylights
// Which in turn was based largely off lifxjs:
// https://github.com/magicmonkey/lifxjs/blob/master/Protocol.md
// *****************************************************************************

#define RESP_GATEWAY 0x03

#define MAX_BULBS 20

// How many seconds to say a bulb is no longer present
#define BULB_TIMEOUT (60)

// Globals
Bulb_t bulbs[MAX_BULBS];
uint8_t num_bulbs = 0;
uint32_t sys_start_date = 0;

void ICACHE_FLASH_ATTR
getBroadcastPkt(uint8_t * data, int len) {
  MsgHeader_t * tmp;

  if (len < 36) return;
  // See https://github.com/mpapi/lazylights/blob/master/lazylights.py
  // or https://github.com/magicmonkey/lifxjs/blob/master/Protocol.md
  // <HHxxxx6sxx6sxxQHxxA
  // Essentially, query all listening bulbs for what TCP port to connect to
  memset(data, 0, 36);
  tmp = (MsgHeader_t *)data;
  tmp->size = 36;
  tmp->protocol = 0x3400; // PROTOCOL_DISCOVERY
  tmp->packet_type = 2;   // REQ_GATEWAY
}

void ICACHE_FLASH_ATTR
getPktSetBulbPower(uint8_t * data, int len, int idx, uint16_t state) {
  MsgHeader_t * tmp;

  if (len < 38) return;
  memset(data, 0, 38);
  tmp = (MsgHeader_t *)data;

  tmp->size = 38;
  tmp->protocol = 0x1400;  // PROTOCOL_COMMAND
  tmp->packet_type = 0x15; // Get power state
  memcpy(&tmp->target_mac, &bulbs[idx].mac, 6);
  strcpy((char *)&tmp->mac, "LIFXV2"); // Don't know if this matters...
  // The payload
  memcpy(&data[36], &state, 2);
}


inline uint32_t getUnixTime() {
    return (xTaskGetTickCount()/100) + sys_start_date;
}

ip_addr_t * ICACHE_FLASH_ATTR
getBulbAddr(uint8_t idx) {
  if (idx >= num_bulbs) {
    return NULL;
  }
  return &bulbs[idx].addr;
}

int ICACHE_FLASH_ATTR getBulbByMAC(const uint8_t * mac) {
  uint8_t i, j, found;
  // Check the bulbs list for the first instance with the same MAC
  for (i=0;i<num_bulbs;i++) {
    found = 1;
    for (j=0;j<6;j++) {
      if (mac[j] != bulbs[i].mac[j]) {
        found = 0;
        break;
      }
    }
    if (found)
      return i;
  }
  return -1;
}

uint8_t ICACHE_FLASH_ATTR getNumBulbs() {
  return num_bulbs;
}


// Using a static array is likely not efficient
// TODO - Upgrade bulb storage to use a linked list
void ICACHE_FLASH_ATTR
_removeBulb(uint8_t idx) {
  int i;
  for (i=idx;i<num_bulbs-1;i++) {
    memcpy(&bulbs[i], &bulbs[i+1], sizeof(Bulb_t));
  }
  num_bulbs--;
}

void ICACHE_FLASH_ATTR
_bulbTimeout() {
  int i;
  uint32_t curr_time = getUnixTime();
  for (i=0;i<num_bulbs;i++) {
    if (curr_time > bulbs[i].last_seen + BULB_TIMEOUT) {
      os_printf("[timeout] Removing bulb[%d]: ip: %d.%d.%d.%d, mac: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
          i,
          bulbs[i].addr.addr & (0xFFUL << 0), (bulbs[i].addr.addr & 0xFF00UL ) >> 8, (bulbs[i].addr.addr & 0xFF0000UL) >> 16, (bulbs[i].addr.addr & 0xFF000000UL ) >> 24,
          bulbs[i].mac[0], bulbs[i].mac[1], bulbs[i].mac[2], bulbs[i].mac[3], bulbs[i].mac[4], bulbs[i].mac[5]);
      _removeBulb(i);
    }
  }
}


void ICACHE_FLASH_ATTR
_checkBulb(MsgHeader_t * pkt, struct ip_addr addr) {
  int i, j;
  uint8_t found = 0;

  _bulbTimeout();
  if (num_bulbs >= MAX_BULBS) return;

  // Check the bulbs list for the first instance with the same MAC
  for (i=0;i<num_bulbs;i++) {
    found = 1;
    for (j=0;j<6;j++) {
      if (pkt->target_mac[j] != bulbs[i].mac[j]) {
        found = 0;
        break;
      }
    }
    if (found) break;
  }

  // If bulb already exists, just update it's last_seen timestamp
  if (found) {
    bulbs[i].last_seen = getUnixTime();
  } else {
    os_printf("[checkBulb] Adding New bulb: ip: %d.%d.%d.%d, mac: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
        addr.addr & (0xFFUL << 0), (addr.addr & 0xFF00UL ) >> 8, (addr.addr & 0xFF0000UL) >> 16, (addr.addr & 0xFF000000UL ) >> 24,
        pkt->target_mac[0], pkt->target_mac[1], pkt->target_mac[2], pkt->target_mac[3], pkt->target_mac[4], pkt->target_mac[5]);
    memcpy(&bulbs[num_bulbs].addr, &addr, sizeof(addr));
    memcpy(&bulbs[num_bulbs].mac, &(pkt->target_mac), sizeof(pkt->target_mac));
    bulbs[num_bulbs].last_seen = (xTaskGetTickCount()/100) + sys_start_date;
    num_bulbs++;
  }
}

void ICACHE_FLASH_ATTR
_clearBulbs(void) {
  num_bulbs = 0;
}

void ICACHE_FLASH_ATTR
parseBroadcastMsg(struct ip_addr addr, uint8_t * pkt, int len) {
    uint8_t * data;
    MsgHeader_t * tmp;
    uint32_t time;
    int i;
    // Additional sanity check
    if (pkt == NULL)
      return;
    // Minimum Lifx packet length
    if (len < 36)
      return;
    tmp = (MsgHeader_t *)pkt;
    // The ESP8266 doesn't do 64-bit math very well
    // Timestamp is specified in nanoseconds - convert to seconds
    time = tmp->timestamp/1000000000;
    // Use the discovery packet as a bootleg NTP server
    // Assuming the bulbs have accurate time, and that
    // the in-flight packet time is less than a second,
    // Use the received timestamp as the current UNIX time
    if (sys_start_date == 0) {
      sys_start_date = time - xTaskGetTickCount()/100;
    }
    switch (tmp->packet_type) {
    case RESP_GATEWAY:
      /*os_printf("[parse] New bulb: ip: %d.%d.%d.%d, mac: %02x:%02x:%02x:%02x:%02x:%02x, service: %02x, port: %d\r\n",
          addr.addr & (0xFFUL << 0), (addr.addr & 0xFF00UL ) >> 8, (addr.addr & 0xFF0000UL) >> 16, (addr.addr & 0xFF000000UL ) >> 24,
          tmp->target_mac[0], tmp->target_mac[1], tmp->target_mac[2], tmp->target_mac[3], tmp->target_mac[4], tmp->target_mac[5],
          pkt[36], pkt[37] | (pkt[38] << 8) | (pkt[39] << 16) | (pkt[40] << 24));
      */
      _checkBulb(tmp, addr);
      break;
    default:
      os_printf("[parse] unknown pkt: size: %d, protocol: %04x, target_mac: %02x%02x%02x%02x%02x%02x, mac: %02x%02x%02x%02x%02x%02x, type: %02x\r\n",
          tmp->size, tmp->protocol,
          tmp->target_mac[0], tmp->target_mac[1], tmp->target_mac[2], tmp->target_mac[3], tmp->target_mac[4], tmp->target_mac[5],
          tmp->mac[0], tmp->mac[1], tmp->mac[2], tmp->mac[3], tmp->mac[4], tmp->mac[5],
          tmp->packet_type);
      break;
    }
}
