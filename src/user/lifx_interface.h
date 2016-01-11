#ifndef __LIFX_INTERFACE_H_
#define __LIFX_INTERFACE_H_

#include "esp_common.h"


typedef struct __attribute__((packed)) {
  uint8_t second;
  uint8_t minute;
  uint8_t hour;
  uint8_t day;
  char month[3];
  uint8_t year;
} LifxTimestamp_t;

typedef struct __attribute__((packed)) {
  uint16_t size;
  uint16_t protocol;
  uint32_t reserved1;
  uint8_t target_mac[6];
  uint16_t reserved2;
  uint8_t mac[6];
  uint16_t reserved3;
  uint64_t timestamp;
  uint16_t packet_type;
  uint16_t reserved4;
} MsgHeader_t;

typedef struct __attribute__((packed)) {
  MsgHeader_t header;
  uint8_t service;
  uint32_t port;
} MsgPANGateway_t;

typedef struct {
  struct ip_addr addr;
  uint8_t mac[6];
  uint32_t last_seen;
} Bulb_t;


void ICACHE_FLASH_ATTR
getBroadcastPkt(uint8_t * data, int len);

void ICACHE_FLASH_ATTR
parseBroadcastMsg(struct ip_addr addr, uint8_t * pkt, int len);

#endif //__LIFX_INTERFACE_H_
