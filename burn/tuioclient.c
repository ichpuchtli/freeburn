/* -----------------------------------------------------------------------------
# Company name: CSSE3010 - Embedded Systems Design & Interfacing
# Project name: Practical 7 - FreeRTOS Networking
# Module name : TUIO Client
# Functionality: Extract TUIO /tuio/2dobj set messages on TCP Port 3000
# See http://www.tuio.org/?specification TUIO 1.1 Specification.
# Hardware platform: NetduinoPlus (AT91SAM7)
#
# Author name: M. D'Souza
# Creation date: 060812
# Revision date (name): -
# Changes implemented (date): -
#(Comments):
------------------------------------------------------------------------------*/

#include "tuioclient.h"
#include "uip.h"
#include "queue.h"
#include "string.h"

#define BUF ((struct uip_tcpip_hdr*) &uip_buf[UIP_LLH_LEN])

#define	MSG_OFFSET	8
#define BREAK_MESSAGE_LOOP	3000
//#Uncomment to include debug messages
#define DEBUG 1

#ifdef DEBUG
#include "debug_printf.h"
#endif

extern xQueueHandle xQueueTuioData;
extern xQueueHandle xQueueGET;
extern char* json_string;

portBASE_TYPE xTUIOCount = 0;
portBASE_TYPE xHTTPCount = 0;
portBASE_TYPE xACKCount = 0;

/*---------------------------------------------------------------------------*/
/* Function called when TCP/IP message for TUIO port is received */
void tuioclient_appcall(void)
{
	unsigned char *h;
	int i, j, k, msg_index, msg_size;
        unsigned long xqData = 0;

	//TUIO Attributes: s i x y a X Y A m r
	int session_id, class_id; //TUIO session and class ID attributes
	float position_x, position_y, angle_a, velocity_x, velocity_y, rotation_velocity_a, motion_acceleration, rotation_acceleration;	//See TUIO 1.1 Specification, Table 1.
	unsigned int tuio_attribute[10];
 	
	//Example Message = HeaderPayload:
	//	Header = /tuio/2Dobj,siiffffffffset	
	//	Payload = sixyaXYAmr
	char *tuio_2dobj_set = "siiffffffff"; // /tuio/2Dobj message field to search for (restricted to 11 characters).
	
	
	//Pointer to TCP message application data field containing TUIO message.
	h = (unsigned char *) uip_appdata;

	msg_index = -1;
	i = 20;		//Skip the first 20 characters of the message.
	msg_size = strlen(tuio_2dobj_set);


	//Search for /tuio/2Dobj set message. Note: message is part of OSC (Open Sound Control) 1.1 bundle message.
	while (i < 100) {

		// Check for first character of /tuio/2Dobj set message
		if (h[i] == tuio_2dobj_set[0]) {

			k = 0;
			for (j=0; j < msg_size; j++) { 
			
				if (h[i+j] == tuio_2dobj_set[j]) {
					k++;
				}
			}

			// If /tuio/2Dobj set message detected, breakout of loop
			if (k == msg_size) {
				msg_index = i;
				i = BREAK_MESSAGE_LOOP;
			}
		}
		i++;
	}

	//If TUIO /tuio/2Dobj set message detected, extract TUIO attributes.
	if (msg_index >= 0) {

		//Extract the TUIO attributes. See TUIO 1.1 Specification, Table 1.
		for (i = 0; i < 10; i++) {
			tuio_attribute[i] = (unsigned int)((h[msg_index + msg_size + MSG_OFFSET + (4*i)] << 24) | (h[msg_index + msg_size + MSG_OFFSET + 1 + (4*i)] << 16) | (h[msg_index + msg_size + MSG_OFFSET + 2 + (4*i)] << 8) | h[msg_index + msg_size + MSG_OFFSET + 3 + (4*i)]);
		}

		//Set TUIO attributes. See TUIO 1.1 Specification, Table 1.
		session_id		= (int) tuio_attribute[0];			 
		class_id 		= (int) tuio_attribute[1];
		position_x 		= *(float *)&tuio_attribute[2]; 	//range 0...1
		position_y 		= *(float *)&tuio_attribute[3]; 	//range 0...1
		angle_a 		= *(float *)&tuio_attribute[4]; 	//range 0..2PI
		velocity_x		= *(float *)&tuio_attribute[5]; 
		velocity_y 		= *(float *)&tuio_attribute[6]; 
		rotation_velocity_a 	= *(float *)&tuio_attribute[7]; 
		motion_acceleration 	= *(float *)&tuio_attribute[8]; 
		rotation_acceleration 	= *(float *)&tuio_attribute[9];

                /* Don't block if full */
                xQueueSend(xQueueTuioData, (void*) tuio_attribute ,1);

#ifdef DEBUG
                debug_printf("s:%d i:%d x:%d y:%d a:%d ", session_id, class_id, (int)(position_x*100.0f), (int)(position_y*100.0f), (int)(angle_a*100.0f)); 
                debug_printf("X:%d Y:%d A:%d m:%d r:%d \n", (int)(velocity_x*100.0f), (int)(velocity_y*100.0f), (int)(rotation_velocity_a*100.0f), (int)(motion_acceleration*100.0f), (int)(rotation_acceleration*100.0f));	

#endif
	}	

}

/* TODO Note tuio_conn* resused for every connection deep copy tuio_conn first */
/* uip.h : extern struct uip_conn* uip_conn; */
void main_appcall(void){

    uint16_t srcPort, destPort;
    uint8_t *srcIP, *destIP;
    uint8_t* filename;

    uint8_t netduinoIP[4] = {192,168,0,4};

    srcPort = BUF->srcport;
    destPort = BUF->destport;

    srcIP = (uint8_t*) BUF->srcipaddr;
    destIP = (uint8_t*) BUF->destipaddr;

    filename = uip_conn->appstate.filename;

    struct httpd_state *s =(struct httpd_state*) (&(uip_conn->appstate));

    vTuioConnectionHold();

    if( srcPort == (&tuio_conn)->lport ){

        // Outgoing ACK
        //debug_printf("[TUIO ACK][%d]\r\n", xACKCount++);

    } else if( destPort == (&tuio_conn)->lport ){


        // Incoming TUIO Data
        //debug_printf("\t[TUIO][%d]\r\n", xTUIOCount++);
        tuioclient_appcall();

    } else {

        debug_printf("[HTTP][%d] GET %s\r\n", xHTTPCount++, filename);
        // Assume Incoming HTTP

        if( strstr( filename, "json") != NULL){

            debug_printf("PSOCK_INIT\r\n");

            if( uip_connected() ){
                PSOCK_INIT(&s->sin, NULL, 0);
            }

            PSOCK_BEGIN(&s->sin);

            PSOCK_SEND_STR(&s->sin, "HTTP/1.0 200 OK\r\n" \
                                    "Connection: close\r\n" \
                                    "Content-Type: text/plain\r\n\r\n"); 

            PSOCK_SEND_STR(&s->sin, json_string); 

            PSOCK_CLOSE(&s->sin);
            PSOCK_END(&s->sin);

        }

        else {

            xQueueSendToBack(xQueueGET, filename , 0);
            httpd_appcall();
        }

        if( (uip_conn)->tcpstateflags == UIP_TIME_WAIT ){
            (uip_conn)->tcpstateflags = UIP_ESTABLISHED;
        }

    }
}


