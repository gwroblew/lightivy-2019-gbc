#ifndef __CONTROL_H
#define __CONTROL_H

#include "system.h"

#define WORKBUF_SIZE    (32768)

void control_init();
void control_set_workbuf(uint8_t *wb);
void control_connect_callback(TCPCONN *conn);
void control_disconnect_callback(TCPCONN *conn);
void control_sent_callback(TCPCONN *conn);
uint32_t control_read_string(uint8_t **pptr, uint32_t *psize);
void control_recv_callback(TCPCONN *conn, void *data, uint32_t size);

#endif
