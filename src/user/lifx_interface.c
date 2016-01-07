#include "lifx_interface.h"
#include "esp_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/udp.h"


// *****************************************************************************
// Implementation details glined from the python Lazylights module
// https://github.com/mpapi/lazylights
// Which in turn was based largely off lifxjs:
// https://github.com/magicmonkey/lifxjs/blob/master/Protocol.md
// *****************************************************************************

#define RESP_GATEWAY 0x03

#define MAX_BULBS 20

// Globals
Bulb_t bulbs[20];
uint8_t num_bulbs = 0;


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
parseBroadcastMsg(struct ip_addr addr, uint8_t * pkt, int len) {
    uint8_t * data;
    MsgHeader_t * tmp;
    int i;
    // Additional sanity check
    if (pkt == NULL)
      return;
    // Minimum Lifx packet length
    if (len < 36)
      return;
    tmp = (MsgHeader_t *)pkt;
    switch (tmp->packet_type) {
    case RESP_GATEWAY:
      os_printf("[parse] New bulb: ip: %d.%d.%d.%d, mac: %02x:%02x:%02x:%02x:%02x:%02x, service: %02x, port: %d\r\n",
          addr.addr & (0xFFUL << 0), (addr.addr & 0xFF00UL ) >> 8, (addr.addr & 0xFF0000UL) >> 16, (addr.addr & 0xFF000000UL ) >> 24,
          tmp->target_mac[0], tmp->target_mac[1], tmp->target_mac[2], tmp->target_mac[3], tmp->target_mac[4], tmp->target_mac[5],
          pkt[36], pkt[37] | (pkt[38] << 8) | (pkt[39] << 16) | (pkt[40] << 24));
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
