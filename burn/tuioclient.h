/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Practical 7 - FreeRTOS Networking
# Module name : TUIO Client
# Functionality: Extract TUIO /tuio/2dobj set messages on TCP Port 3000
# Hardware platform: NetduinoPlus (AT91SAM7)
#
# Author name: M. D'Souza
# Creation date: 060812
# Revision date (name): -
# Changes implemented (date): -
#(Comments): 
------------------------------------------------------------------------------*/

#ifndef __TUIOCLIENT_H__
#define __TUIOCLIENT_H__


/* Since this file will be included by uip.h, we cannot include uip.h
   here. But we might need to include uipopt.h if we need the u8_t and
   u16_t datatypes. */
#include "FreeRTOS.h"
#include "uipopt.h"

#include "task.h"
#include "psock.h"
#include "httpd-fs.h"
#include "httpd.h"

 /*Next, we define the uip_tcp_appstate_t datatype. This is the state
   of our application, and the memory required for this state is
   allocated together with each TCP connection. One application state
   for each TCP connection. */

typedef struct httpd_state uip_tcp_appstate_t;

/* Finally we define the application function to be called by uIP. */
void tuioclient_appcall(void);

void main_appcall(void);

extern struct uip_conn tuio_conn;
extern void vTuioConnectionHold(void);

#ifndef UIP_APPCALL
#define UIP_APPCALL main_appcall
#endif /* UIP_APPCALL */

#endif /* __TUIOCLIENT_H__ */
/** @} */
/** @} */

