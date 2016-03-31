/**********************************************************************************/
/*  Copyright (c) 2011, Silicon Image, Inc.  All rights reserved.                 */
/*  No part of this work may be reproduced, modified, distributed, transmitted,   */
/*  transcribed, or translated into any language or computer format, in any form  */
/*  or by any means without written permission of: Silicon Image, Inc.,           */
/*  1140 East Arques Avenue, Sunnyvale, California 94085                          */
/**********************************************************************************/

/**
 * @file siidvrtest.c
 *
 * @brief
 *   The MHL TX device driver simple test application
 *
 *****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <sys/uio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>

#include "mhl_linuxdrv_ioctl.h"


// Convert a value specified in milliseconds to nanoseconds
#define MSEC_TO_NSEC(x)	(x * 1000000UL)

// Define interval used to poll driver for events to 40ms.
#define POLL_INTERVAL_MS	40

#define KEYCODE_STR_SIZE 	6


static pthread_mutex_t	cmdLock = PTHREAD_MUTEX_INITIALIZER;
static uint8_t	pollThreadExitFlag = 0;
static uint8_t	sendKeyCode = 0xFF;

// State of MHL connection
static uint8_t	mhlState = 0;
#define MHL_STATE_CONNECTED	0x01
#define MHL_STATE_RCP_READY	0x02
#define MHL_STATE_RCP_IP		0x04
#define MHL_STATE_RCP_SEND_ACK	0x08


// Function executed by driver event polling thread.
void* PollDriverEvents(void *ptr);


int gMhlDrv;



int main(int argc, char **argv) 
{
	pthread_t	pollThread;
	int			status;
	int			key;
	char		keyCodeStr[KEYCODE_STR_SIZE];
	long int	keyCode;
	char		*mhl_linuxdrv_name = "/dev/siI-833x";


	printf("Welcome to the Silicon Image Transmitter test application\nAttempting to open driver %s\n", mhl_linuxdrv_name);
  
	gMhlDrv = open(mhl_linuxdrv_name, O_RDONLY);
  
	if (gMhlDrv < 0)
	{
		printf("Error opening %s!\n", mhl_linuxdrv_name);
		return errno;
	}


	status = pthread_create(&pollThread, NULL, PollDriverEvents, NULL);

	printf("Command key codes:\n");
	printf("    E            = exit program\n");
	printf("    S <key code> = send remote control code\n");

	do {
		key = getchar();
		if(key == 'E' || key == 'e')
		{
			break;
		}

		if(key != 'S' && key != 's')
		{
			// Ignore invalid key press
			continue;
		}

		fgets(keyCodeStr, KEYCODE_STR_SIZE, stdin);
		keyCode = strtol(keyCodeStr, NULL, 0);
		if(keyCode >= 0x7F)
		{
			printf("Ignoring invalid key code: 0x%02lx\n", keyCode);
		}

		pthread_mutex_lock(&cmdLock);
		sendKeyCode = (uint8_t)keyCode;
		pthread_mutex_unlock(&cmdLock);

	} while(1);

  // Signal driver event polling thread to quit.
  pollThreadExitFlag = 1;
  pthread_join(pollThread, NULL);

  close(gMhlDrv);
  return 0;
}



void* PollDriverEvents(void *ptr)
{
	struct timespec		ts;
	mhlTxEventPacket_t	mhlEventPacket;
	int					status;


	ts.tv_sec = 0;
	ts.tv_nsec = MSEC_TO_NSEC(POLL_INTERVAL_MS);

	while(pollThreadExitFlag == 0)
	{
		nanosleep(&ts, NULL);

		status = ioctl(gMhlDrv,SII_MHL_GET_MHL_TX_EVENT, &mhlEventPacket );
		if(status < 0)
		{
			printf("SII_MHL_GET_MHL_TX_EVENT ioctl failed, status: %d\n", status);
		}
		else
		{
			pthread_mutex_lock(&cmdLock);

			switch(mhlEventPacket.event)
			{
				case MHL_TX_EVENT_NONE:				// No new events
					break;

				case MHL_TX_EVENT_DISCONNECTION:	// MHL connection has been lost
					printf("Disconnection event received\n");
					mhlState = 0;
					break;

				case MHL_TX_EVENT_CONNECTION:		// MHL connection has been established
					printf("Connection event received\n");
					mhlState |= MHL_STATE_CONNECTED;
					break;

				case MHL_TX_EVENT_RCP_READY:		// MHL connection is ready for RCP
					printf("Connection ready for RCP event received\n");
					mhlState |= MHL_STATE_RCP_READY;
					break;

				case MHL_TX_EVENT_RCP_RECEIVED:		// Received an RCP. Key Code in eventParam
					printf("RCP event received, key code: 0x%02x\n",
							mhlEventPacket.eventParam);
					mhlState |= MHL_STATE_RCP_SEND_ACK;
					break;

				case MHL_TX_EVENT_RCPK_RECEIVED:	// Received an RCPK message
					if((mhlState & MHL_STATE_RCP_IP) &&
						(mhlEventPacket.eventParam == sendKeyCode))
					{
						printf("RCPK received for sent key code: 0x%02x\n",
								mhlEventPacket.eventParam);
						// Received positive acknowledgment for key code
						// we sent so update our state to indicate that the
						// key code send is complete.
						mhlState &= ~MHL_STATE_RCP_IP;
						sendKeyCode = 0xFF;
					}
					else
					{
						printf("Unexpected RCPK event received, key code: 0x%02x\n",
								mhlEventPacket.eventParam);
					}
					break;

				case MHL_TX_EVENT_RCPE_RECEIVED:	// Received an RCPE message
					if(mhlState & MHL_STATE_RCP_IP)
					{
						printf("RCPE received for sent key code: 0x%02x\n",
								mhlEventPacket.eventParam);
						// Received negative acknowledgment for key code
						// we sent so update our state to indicate that the
						// key code send is complete.
						mhlState &= ~MHL_STATE_RCP_IP;
						sendKeyCode = 0xFF;
					}
					else
					{
						printf("Unexpected RCPE event received\n");
					}
					break;

				default:
					printf("Unknown event code: %d \n", mhlEventPacket.event);
			}

			if((mhlState & (MHL_STATE_RCP_READY | MHL_STATE_RCP_IP))
				== (MHL_STATE_RCP_READY))
			{
				// MHL connection is in a state where we can initiate the
				// sending of a Remote Control Protocol (RCP) code.  So if the
				// user has entered one go ahead and send it.
				if(sendKeyCode != 0xFF)
				{
					status = ioctl(gMhlDrv,SII_MHL_RCP_SEND, sendKeyCode );
					if(status < 0)
					{
						printf("SII_MHL_RCP_SEND ioctl failed, status: %d\n", status);
					}
					else
					{
						// Flag we're in the process of sending a RCP code.
						mhlState |= MHL_STATE_RCP_IP;
					}
				}
			}
			else if(mhlState & MHL_STATE_RCP_SEND_ACK)
			{
				// For now, we just positively acknowledge any key codes received.
				status = ioctl(gMhlDrv,SII_MHL_RCP_SEND_ACK, mhlEventPacket.eventParam );
				if(status < 0)
				{
					printf("SII_MHL_RCP_SEND_ACK ioctl failed, status: %d\n", status);
				}

				// Flag we no longer need to send an ack.
				mhlState &= ~MHL_STATE_RCP_SEND_ACK;
			}

			pthread_mutex_unlock(&cmdLock);
		}
	}

	return NULL;
}
