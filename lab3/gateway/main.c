/**
  * File: main.c
  * 18-748: Wireless Sensor Networks
  * Authors: Lucas Bruder, Jeremy Xiao, and Pallavi Kuman
  * Team 12
  * Lab 3: Multi-hop
  * Last modified: 2/14/2016
  *
  * This is code loaded on to Gateway firefly node for lab 3.
  *
  * TODO memset on transition to route start to clear results from last time, be careful about peerList 
  * also gateway_data.neighborTimeoutHappened = false, adjacnecy list, etc.
  *
  * Also add timeouts for each state where we transition back to the menu or something
  */


/*****************************************
 *                 INCLUDES              *
 *****************************************/

#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <hal.h>
#include <bmac.h>
#include <nrk_error.h>

/*****************************************
 *                 DEFINES               *
 *****************************************/

#define MAC_ADDR             		15 // MAC address of gateway node
#define CHANNEL              		12 // channel to communicate on
#define MAX_SECONDS_BUF_LEN   		 4 // maximum number of digits to accept for updated refresh or sampling rate time

#define MAX_NUM_SECONDS_WAITING_RTS	 5 
#define MAX_NUM_PEERS        		 5

#define TASK_PERIOD_NS		 500000000	// 2 Hz (500ms period)

#define NETWORK_REFRESH_DEFAULT_PERIOD	5	// every 120 seconds routing algorithm is ran

#define MAX_STATE_TIME		15		// max time we are allowed in a state before going back to menu

/*****************************************
 *                 TYPEDEFS              *
 *****************************************/

 typedef enum{
    STATE_OFF,

    /*
     * Menu related states
     */
    STATE_MENU,
    STATE_NEW_SAMPLE_PERIOD,
    STATE_NEW_REFRESH_PERIOD,

    /*
     * Routing related states
     */
    STATE_ROUTE_START,     // start routing, send out ROUTE message to all nodes
    STATE_ROUTE_RTS,       // listen to RTS messages from nodes directly next to GTW for certain amount of time
    STATE_ROUTE_CTS,       // a peer is currently transmitting, trying to find its neighbors
    STATE_ROUTE_PEER_DONE, // another node finished finding its neighbors, send ACK and transition to CTS
    STATE_ROUTE_DONE,      // routing is done, run algorithm to connect the nodes and send out results to network

    /*
     * Control which nodes are transmitting sensor values, starting from the outside in
     */
    STATE_RECEIVE_SENSOR_VALUES,

 } gateway_state_E;

 typedef struct{
    gateway_state_E desiredState;
    gateway_state_E presentState;;
    bool stateTransition;

    uint16_t networkRefreshPeriod;
    uint16_t sensorUpdatePeriod;

    uint8_t *receive_buf;
    uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
    uint8_t done_buf[RF_MAX_PAYLOAD_SIZE];

    uint8_t len;
    int8_t rssi;

    char serialInput;
    bool newSerialInput;

    bool transitionBackToMenu;

    nrk_time_t lastRouteTime;
    bool routingDone;

    nrk_time_t currentTime;
    nrk_time_t lastNeighborFoundTime;
    nrk_time_t stateTransitionTime;

    bool transitionToCTS;
    uint16_t current_peer_id;
    uint16_t done_buf_len;

    bool receivedDoneFromPeer;
    bool neighborTimeoutHappened;
    bool seenCtsState;

    int8_t peerList[MAX_NUM_PEERS];

    uint8_t adjacencyList[MAX_NUM_PEERS][MAX_NUM_PEERS];

 } gateway_data_S;

/*****************************************
 *             FUNCTION HEADERS          *
 *****************************************/

void gateway_task(void);
void nrk_create_taskset(void);
void nrk_register_drivers();

void gateway_init(void);

// Send and receive functions for wireless communication
bool gateway_receiveMessage(void);
bool gateway_sendMessage(void);

// State machine processing
void gateway_processData(void);
void gateway_getDesiredState(void);
void gateway_setCurrentState(void);

/*
 * State machine transitions
 */

// Menu transitions
inline bool gateway_allowTransitionMenuToRouteStart(void);
inline bool gateway_allowTransitionMenuToReceiveSensorValues(void);
inline bool gateway_allowTransitionMenuToNewSamplePeriod(void);
inline bool gateway_allowTransitionMenuToNewRefreshPeriod(void);

// Route transitions
inline bool gateway_allowTransitionRouteToMenu(void);
inline bool gateway_allowTransitionRouteToReceiveSensorValues(void);

// New sample transitions
inline bool gateway_allowTransitionNewSamplePeriodToMenu(void);

// New refresh transitions
inline bool gateway_allowTransitionNewRefreshPeriodToMenu(void);

// Receive sensor values transitions
inline bool gateway_allowTransitionReceiveSensorValuesToRouteStart(void);
inline bool gateway_allowTransitionReceiveSensorValuesToMenu(void);

// Off transitions
inline bool gateway_allowTransitionOffToMenu(void);

/*****************************************
 *              VARIABLES                *
 *****************************************/

/*
 * Gateway task
 */
nrk_task_type GATEWAY_TASK;
NRK_STK gateway_task_stack[NRK_APP_STACKSIZE];
uint8_t gateway_buf[RF_MAX_PAYLOAD_SIZE];

// RX and TX buffers
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
nrk_time_t start_time_receive;
nrk_time_t end_time_receive;
bool receivedMessage;

nrk_sig_t uart_rx_signal;

gateway_data_S gateway_data;

uint16_t peersDirectlyConnectedToGateway[MAX_NUM_PEERS]; // list of peers directly connected to the gateway

static const char* ROUTE_STRING = "GTW:ROUTE";

/*****************************************
 *          HELPER FUNCTIONS             *
 *****************************************/

/**
  *
  * Receives a message and puts it into local_buf.
  *
  * @return true if received a message, false otherwise
  *
  */
bool gateway_receiveMessage(void)
{
    bool received = false;
    /*
     *  Code takes care of receiving and sets timeout = 1 if there's a timeout receiving a packet
     */
    while(1)
    {
        if(bmac_rx_pkt_ready())
        {
            gateway_data.receive_buf = bmac_rx_pkt_get (&(gateway_data.len), &(gateway_data.rssi));
            // printf ("Got RX packet len=%d RSSI=%d [", gateway_data.len, gateway_data.rssi);
            // for (uint16_t i = 0; i < gateway_data.len; i++)
            // {
            //     printf ("%c", gateway_data.receive_buf[i]);
            //     printf ("]\r\n");
            // }
            
            received = true;

            // Release the RX buffer so future packets can arrive
            bmac_rx_pkt_release ();
            break;
        }
        
        // Implement timeouts
        nrk_time_get(&end_time_receive);
        if(end_time_receive.nano_secs > start_time_receive.nano_secs)
        {
            if(((end_time_receive.secs-start_time_receive.secs)*1000+(end_time_receive.nano_secs-start_time_receive.nano_secs)/1000000) > 1000)
            {
                received = false;
                break;
            }
        }
        else
        {
            if(((end_time_receive.secs-start_time_receive.secs)*1000-(start_time_receive.nano_secs-end_time_receive.nano_secs)/1000000) > 1000)
            {
                received = false;
                break;
            }
        }
    }

    return received;
}

/**
  * Sends a message placed in the tx_buf
  */
bool gateway_sendMessage(void)
{
    int8_t val;
    bool sent = true;

    val=bmac_tx_pkt(gateway_data.tx_buf, strlen(gateway_data.tx_buf)+1);

    if(val != NRK_OK)
    {
        nrk_kprintf(PSTR("Could not Transmit!\r\n"));
        sent = false;
    }

    nrk_time_get(&start_time_receive);

    return sent;
}

/**
  * Always allow transition from off to the menu
  */
inline bool gateway_allowTransitionOffToMenu(void)
{
    bool allowTransition = true;

    return allowTransition;
}

/**
  * Checks to see if we should transition from the menu to receiving sensor values from
  * the peers on the network
  *
  * @return true if we should transition, false otherwise
  */
inline bool gateway_allowTransitionMenuToReceiveSensorValues(void)
{
    bool allowTransition = false;

    if(gateway_data.newSerialInput == true && gateway_data.serialInput == 'e')
    {
        allowTransition = true;
    }

    return allowTransition;
}

/**
  * 
  */
inline bool gateway_allowTransitionRouteToMenu(void)
{
    bool allowTransition = false;

    return allowTransition;
}

/**
  * Allow transition from menu to the route immediately if we get new serial input
  * and its an R
  */
inline bool gateway_allowTransitionMenuToRouteStart(void)
{
    bool allowTransition = false;

    if(gateway_data.newSerialInput == true && gateway_data.serialInput == 'R')
    {
    	allowTransition = true;
    }

    return allowTransition;  
}

/**
  * If we've entered a new sample period, transition back to the menu
  */
inline bool gateway_allowTransitionNewSamplePeriodToMenu(void)
{
    bool allowTransition = false;

    if(gateway_data.transitionBackToMenu == true)
    {
        allowTransition = true;
        gateway_data.transitionBackToMenu = false;
    }

    return allowTransition;
}

/**
  * Allow transition from new refresh period state to menu if we've set the new
  * refresh period
  * @return true if transition is allowed, false otherwise
  */
inline bool gateway_allowTransitionNewRefreshPeriodToMenu(void)
{
    bool allowTransition = false;

    if(gateway_data.transitionBackToMenu == true)
    {
        allowTransition = true;
        gateway_data.transitionBackToMenu = false;
    }

    return allowTransition;
}

/**
  * Allow transition from menu to new sample period if we have new serial input
  * and the character entered is an 's'
  * @return true if transition is allowed, false otherwise
  */
inline bool gateway_allowTransitionMenuToNewSamplePeriod(void)
{
    bool allowTransition = false;

    if(gateway_data.newSerialInput == true && gateway_data.serialInput == 's')
    {
        allowTransition = true;
    }

    return allowTransition;
}

/**
  * Allow transition from menu to new refresh period if we have new serial input
  * and the character entered is an 'r'
  * @return true if transition is allowed, false otherwise
  */
inline bool gateway_allowTransitionMenuToNewRefreshPeriod(void)
{
    bool allowTransition = false;

    if(gateway_data.newSerialInput == true && gateway_data.serialInput == 'r')
    {
        allowTransition = true;
    }

    return allowTransition;
}

/**
  * Allow transition from routing to receiving sensor values if the routing is done
  *
  * @return true if transition is allowed, false otherwise
  */
inline bool gateway_allowTransitionRouteToReceiveSensorValues(void)
{
    bool allowTransition = false;

    if(gateway_data.routingDone == true)
    {
        allowTransition = true;
    }

    return allowTransition;
}

/**
  * Allow transition from receiving sensor values to routing if it's time to dynamically route
  *
  * @return true if transition is allowed, false otherwise
  */
inline bool gateway_allowTransitionReceiveSensorValuesToRouteStart(void)
{
    bool allowTransition = false;

    nrk_time_get(&gateway_data.currentTime);

    if((gateway_data.currentTime.secs - gateway_data.lastRouteTime.secs) > gateway_data.networkRefreshPeriod)
    {
        allowTransition = true;
    }

    return allowTransition;
}

/**
  * Allow transition during the receiving sensor value state if 'm' is pressed at any time
  */
inline bool gateway_allowTransitionReceiveSensorValuesToMenu(void)
{
    bool allowTransition = false;

    if(gateway_data.newSerialInput == true && gateway_data.serialInput == 'm')
    {
        allowTransition = true;
    }

    return allowTransition;
}

/**
  * After sending out the GTW:ROUTE message, transition to listening for RTS from nearby nodes
  */
inline bool gateway_allowTransitionRouteStartToRouteRTS(void)
{
    bool allowTransition = true;

    // TODO receive a RTS command to transition

    return allowTransition;
}

/**
  * After sending the CTS command, we let the peer do its thing. Once it's done, it will send a done
  * message, containing information about neighbors
  */
inline bool gateway_allowTransitionCtsToPeerDone(void)
{
	bool allowTransition = false;

	if(gateway_data.receivedDoneFromPeer == true)
	{
		allowTransition = true;
		gateway_data.receivedDoneFromPeer = false;
	}

	return allowTransition;
}

/**
  * We check to see if there are any more immediate neighbors in the transmission vicinity of the gateway.
  * if we haven't seen an immediate neighbor timeout yet
  */
inline bool gateway_allowTransitionPeerDoneToRouteStart(void)
{
	bool allowTransition = false;

	if(gateway_data.neighborTimeoutHappened == false)
	{
		allowTransition = true;
	}

	return allowTransition;
}


/**
  * Finds the next neighbor to visit in a list keeping track of which neighbors we haven't seen
  * or return -1 if we don't have anymore
  * 
  * Ensure after this function is called that the indice in the array is changed to 0 to ensure
  * it isn't used twice
  *
  */
inline int16_t gateway_findNextNeighborToVisit(void)
{
	int16_t indice = -1;

	for(uint8_t index = 0; index < MAX_NUM_PEERS; index++)
	{
		if(gateway_data.peerList[index] == 0)
		{
			indice = index;
			break;
		}
	}

	return indice;
}

/**
  * If we have seen a timeout from trying to find immediate neighbors and we 
  * have more nodes left to test, we will transition from being done with this peer
  * to sending out the CTS command to next peers to see who their neighbor is
  */
inline bool gateway_allowTransitionPeerDoneToCTS(void)
{
	bool allowTransition = false;
	
	if((gateway_data.neighborTimeoutHappened == true) && (gateway_findNextNeighborToVisit() != -1))
	{
		gateway_data.neighborTimeoutHappened = false;
		allowTransition = true;
	}

	return allowTransition;
}

/**
  * If we just finished with the last peer (aka no more left to visit) then we will be done routing
  *
  */
inline bool gateway_allowTransitionPeerDoneToRouteDone(void)
{
	bool allowTransition = false;

	// if we have no more people left to visit, we will stop routing and will then combine the information
	if(gateway_findNextNeighborToVisit() == -1)
	{
		allowTransition = true;
	}

	return allowTransition;
}	

/**
  *	Right now, we will always return true. Could change in the future.
  */
inline bool gateway_allowTransitionStateRouteDoneToReceiveSensorValues(void)
{
	bool allowTransition = true;

	return allowTransition;
}

/**
  *  Allow transition from listening for RTS to sending CTS if we received an RTS or
  *  we had a timeout with immediate neighbors and we have another neighbor to visit
  */
inline bool gateway_allowTransitionRTStoCTS(void)
{
	bool allowTransition = false;

	if((gateway_data.transitionToCTS == true))
	{
		allowTransition = true;
		gateway_data.transitionToCTS = false;
	}
	else if((gateway_data.neighborTimeoutHappened == true) && (gateway_findNextNeighborToVisit() != -1))
	{
		allowTransition = true;
	}

	return allowTransition;
}

/**
  * Allow transition from listening to RTS to saying route is done if we've had a neighbor timeout
  * and we have no more nodes left to visit
  */
inline bool gateway_allowTransitionRTStoRouteDone(void)
{
	bool allowTransition = false;

	if((gateway_data.neighborTimeoutHappened == true) && (gateway_findNextNeighborToVisit() == -1))
	{
		allowTransition = true;
	}

	return allowTransition;
}

/**
  * We should allow a transition to the menu if we're stuck in a state for too long
  */
inline bool gateway_allowTransitionToMenu(void)
{
	bool allowTransition = false;

	nrk_time_get(&(gateway_data.currentTime));

	if(gateway_data.currentTime.secs - gateway_data.stateTransitionTime.secs > MAX_STATE_TIME)
	{
		allowTransition = true;
	}

	return allowTransition;
}

/*
 * Process inputs
 */
void gateway_processData(void)
{
    /*
     * Get any serial input if available
     */
    if(nrk_uart_data_ready(NRK_DEFAULT_UART))
    {
        gateway_data.serialInput = getchar();
        gateway_data.newSerialInput = true;
        printf("%c\r\n", gateway_data.serialInput);
    }
    else
    {
        if(gateway_data.presentState == STATE_MENU)
        {
            nrk_event_wait(SIG(uart_rx_signal));
        }
        gateway_data.newSerialInput = false;
    }

    // receive any messages
    receivedMessage = gateway_receiveMessage();
}

/*
 * Get the present state and determine if a state transition shall be performed. If so, set the 
 * desired state to that new state.
 */
void gateway_getDesiredState(void)
{
  gateway_state_E desiredState = gateway_data.presentState;

  switch(desiredState)
  {
        case STATE_OFF:
            if(gateway_allowTransitionOffToMenu() == true)
            {
                desiredState = STATE_MENU;
            }
            else
            {
                // keep state
            }

            break;

        case STATE_MENU:
            if(gateway_allowTransitionMenuToNewSamplePeriod() == true)
            {
                desiredState = STATE_NEW_SAMPLE_PERIOD;
            }
            else if(gateway_allowTransitionMenuToNewRefreshPeriod() == true)
            {
                desiredState = STATE_NEW_REFRESH_PERIOD;
            }
            else if(gateway_allowTransitionMenuToRouteStart() == true)
            {
                desiredState = STATE_ROUTE_START;
            }
            else if(gateway_allowTransitionMenuToReceiveSensorValues() == true)
            {
                desiredState = STATE_RECEIVE_SENSOR_VALUES;
            }
            else
            {
                // keep state
            }

            break;

        case STATE_NEW_SAMPLE_PERIOD:
            if(gateway_allowTransitionNewSamplePeriodToMenu() == true)
            {
                desiredState = STATE_MENU;
            }
            else
            {
                // keep state
            }
            
            break;

        case STATE_NEW_REFRESH_PERIOD:
            if(gateway_allowTransitionNewRefreshPeriodToMenu() == true)
            {
                desiredState = STATE_MENU;
            }
            else
            {
                // keep state
            }

            break;

        case STATE_RECEIVE_SENSOR_VALUES:
            if(gateway_allowTransitionReceiveSensorValuesToRouteStart() == true)
            {
                desiredState = STATE_ROUTE_START;
            }
            else if(gateway_allowTransitionReceiveSensorValuesToMenu() == true)
            {
                desiredState = STATE_MENU;
            }
            else
            {
                // keep state
            }

            break;

        case STATE_ROUTE_START:
            if(gateway_allowTransitionRouteStartToRouteRTS() == true)
            {
                desiredState = STATE_ROUTE_RTS;
            }
            else
            {
                // keep state
            }

            break;

        case STATE_ROUTE_RTS:
            if(gateway_allowTransitionRTStoCTS() == true)
            {
                desiredState = STATE_ROUTE_CTS;
            }
            else if(gateway_allowTransitionRTStoRouteDone() == true)
            {
            	desiredState = STATE_ROUTE_DONE;
            }
            else
            {
                // keep state
            }

            break;
            
        case STATE_ROUTE_CTS:
        	if(gateway_allowTransitionCtsToPeerDone() == true)
        	{
        		desiredState = STATE_ROUTE_PEER_DONE;
        	}
        	else
        	{
        		// keep state
        	}

            break;
            
        case STATE_ROUTE_PEER_DONE:
        	if(gateway_allowTransitionPeerDoneToRouteStart() == true)
        	{
        		desiredState = STATE_ROUTE_START;
        	}
        	else if(gateway_allowTransitionPeerDoneToRouteDone() == true)
        	{
        		desiredState = STATE_ROUTE_DONE;
        	}
        	else if(gateway_allowTransitionPeerDoneToCTS() == true)
        	{
        		desiredState = STATE_ROUTE_CTS;
        	}
        	else
        	{
        		// keep state
        	}

            break;
            
        case STATE_ROUTE_DONE:
        	if(gateway_allowTransitionStateRouteDoneToReceiveSensorValues() == true)
        	{
        		desiredState = STATE_RECEIVE_SENSOR_VALUES;
        	}
        	else
        	{
        		// keep state
        	}

            break;
            
        default:
            // should never reach here, error
             break;
  }

    gateway_data.desiredState = desiredState;;
}

/*
 * Perform any transition and state related outputs
 */
void gateway_setCurrentState(void)
{
    gateway_state_E state = gateway_data.desiredState;

    char option;
    char seconds_buf[MAX_SECONDS_BUF_LEN];
    uint8_t index = 0;
    uint8_t *buf = gateway_data.receive_buf;
    uint16_t peer_id = 0;

    switch(state)
    {
        case STATE_MENU:
            if(gateway_data.stateTransition)
            {
                nrk_kprintf (PSTR ("Gateway Menu:\r\n"));
                nrk_kprintf (PSTR ("Press 's' to change the sensor sampling rate (s).\r\n"));
                nrk_kprintf (PSTR ("Press 'r' to change the network update rate (s).\r\n"));
                nrk_kprintf (PSTR ("Press 'R' to update the network right now.\r\n"));
                nrk_kprintf (PSTR ("Press 'e' to exit this menu.\r\n"));
                nrk_kprintf (PSTR ("-> Note: If at anytime you would like to return to the menu, press 'm'\r\n"));
            } 

            break;

        case STATE_NEW_REFRESH_PERIOD:
            if(gateway_data.stateTransition)
            {
                nrk_kprintf(PSTR("Enter a new network refresh period in seconds then press enter.\r\n"));
                printf("Current network refresh period = %d\r\n", gateway_data.networkRefreshPeriod);
            }

            // Get the new refresh period from serial 
            do{
                if(nrk_uart_data_ready(NRK_DEFAULT_UART))
                {
                    option = getchar();
                    seconds_buf[index] = option;
                    index++;
                }
                // TODO implement something to prevent TASK RESERVE

                if(index > (MAX_SECONDS_BUF_LEN - 1))
                {
                    break;
                }

            } while(option!='\n');

            gateway_data.networkRefreshPeriod = atoi(seconds_buf);

            printf("Refresh period is now = %d\r\n", gateway_data.networkRefreshPeriod);

            // Ensure we transition back to the menu
            gateway_data.transitionBackToMenu = true;

            break;

        case STATE_NEW_SAMPLE_PERIOD:
            if(gateway_data.stateTransition)
            {
                nrk_kprintf(PSTR("Enter a new sensor sample period in seconds then press enter.\r\n"));
                printf("Current sensor update period = %d\r\n", gateway_data.sensorUpdatePeriod);
            }

            // Get the new sample period from serial 
            do{
                if(nrk_uart_data_ready(NRK_DEFAULT_UART))
                {
                    option=getchar();
                    seconds_buf[index] = option;
                    index++;
                }

                // TODO implement something to prevent TASK RESERVE

                if(index > (MAX_SECONDS_BUF_LEN - 1))
                {
                    break;
                }
            } while(option!='\n');

            gateway_data.sensorUpdatePeriod = atoi(seconds_buf);

            printf("Sensor sample period is now = %d\r\n", gateway_data.sensorUpdatePeriod);

            // Ensure we transition back to the menu
            gateway_data.transitionBackToMenu = true;

            break;

        case STATE_ROUTE_START:
            if(gateway_data.stateTransition)
            {
            	nrk_kprintf (PSTR ("--> Starting to route\r\n"));

            	// Send out ROUTE command to peers
            	nrk_time_get(&(gateway_data.stateTransitionTime));
            }

            sprintf(gateway_data.tx_buf, ROUTE_STRING);
            gateway_sendMessage();	

            // perform state related outputs here

            break;
            
        case STATE_ROUTE_RTS:
            if(gateway_data.stateTransition)
            {
                // perform transition related outputs here
            	nrk_kprintf (PSTR ("--> Listening to RTS from nearby peers\r\n"));

            	// get time so we can check for a timeout later
                nrk_time_get(&(gateway_data.lastNeighborFoundTime));
            	nrk_time_get(&(gateway_data.stateTransitionTime));
            }

            /*
             * Listen for received messages and make sure its of the format "RTS:<MAC>"
             */ 
            if(receivedMessage == true)
            {
                if(gateway_data.len > 4 && (buf[0] == 'R' && buf[1] == 'T' && buf[2] == 'S' && buf[3] == ':'))
                {
                    // Find the mac address and add it to the list of peers directly connected to the gateway
                    nrk_kprintf (PSTR ("Received RTS message from MAC address: "));

                    buf = buf + 4;
                    // printf("%s\r\n", buf);
                    peer_id = atoi(buf);

                    printf("%d\r\n", peer_id);

                    peersDirectlyConnectedToGateway[peer_id] = 1;

                    gateway_data.transitionToCTS = true;

                    gateway_data.current_peer_id = peer_id;

                    // grab time
                    nrk_time_get(&(gateway_data.lastNeighborFoundTime));
                }
            }

            // compare time and set variable true

            nrk_time_get(&(gateway_data.currentTime));

            // only wait for a certain amount of time before we start building the network from 1-2 hop noes 
            if((gateway_data.currentTime.secs - gateway_data.lastNeighborFoundTime.secs) > MAX_NUM_SECONDS_WAITING_RTS)
            {
            	gateway_data.neighborTimeoutHappened = true;
            }

            break;
            
        case STATE_ROUTE_CTS:     
            if(gateway_data.stateTransition)
            {
            	nrk_kprintf (PSTR ("--> Peer is clear to enter TEST mode\r\n"));
            	// nrk_time_get(&(gateway_data.stateTransitionTime));

            	// if we've looped through once before
            	if(gateway_data.seenCtsState == true)
            	{
            		int16_t nextNeighbor = gateway_findNextNeighborToVisit();

            		if(nextNeighbor != -1)
            		{
            			gateway_data.current_peer_id = nextNeighbor;
            		}
            	}

            	gateway_data.seenCtsState = true;

            	nrk_kprintf (PSTR("Peer to visit is: "));
            	printf("%d\r\n", gateway_data.current_peer_id);
            }

            // Send CTS to current peer id in format CTS:<MAC>
            // Might need to move this code below
            sprintf((char*)gateway_data.tx_buf, "CTS:%d\r\n", gateway_data.current_peer_id);
            gateway_sendMessage();

            // this could be below, and we could wait to start hearing TEST messages from  the node thats CTS
			// // Send CTS to current peer id in format CTS:<MAC>
   //          sprintf((char*)gateway_data.tx_buf, "CTS:%d\r\n", gateway_data.current_peer_id);
   //          gateway_sendMessage();

            /*
             * After state transition occurs, we let the peers do their thing, trying to find
             * the nodes they can contact. One we receive a done signal, we transition to the done state where we parse
             * the response from the node
             */
            if(receivedMessage == true)
            {
            	uint8_t *buf = gateway_data.receive_buf;

            	// for(uint16_t i = 0; i < 40; i++)
            	// {
            	// 	printf("%c", buf[i]);
            	// }

            	// check to see if it contains done
            	if(gateway_data.len > 4 && buf[0] == 'D' && buf[1] == 'O' && buf[2] == 'N' && buf[3] == 'E')
            	{
            		uint16_t index;
            		// copy everything over
            		gateway_data.done_buf_len = gateway_data.len;

            		for(index = 0; index < gateway_data.len; index++)
            		{
            			gateway_data.done_buf[index] = gateway_data.receive_buf[index];
            		}

            		gateway_data.done_buf[index] = '\0';

            		gateway_data.receivedDoneFromPeer = true;
            	}
            }


            break;
            
        case STATE_ROUTE_PEER_DONE:     
            if(gateway_data.stateTransition)
            {
            	nrk_time_get(&(gateway_data.stateTransitionTime));
            	nrk_kprintf (PSTR ("--> Received done command from peer\r\n"));
                // perform transition related outputs here
            }

            // for(uint16_t index = 0; index < gateway_data.done_buf_len; index++)
            // {
            // 	printf("%c", gateway_data.done_buf[index]);
            // }

            uint16_t len = strlen(gateway_data.done_buf);

            // Find the peer Id
            char *firstColon = strchr(gateway_data.done_buf, ':');
            char *nextNewline = strchr(firstColon, '\n');
            *nextNewline = '\0';
            uint16_t peerId = atoi(firstColon + 1);

            gateway_data.peerList[peerId] = 1;

            // Send acknowledgement to peer id
            sprintf(gateway_data.tx_buf, "ACK:%d", peerId);
            gateway_sendMessage();

            printf("%s\r\n", nextNewline + 1);

            /*
             * Traverse to line under the SEEN to get neighbors
             */
            nextNewline = strchr(nextNewline + 1, '\n');
            nextNewline = strchr(nextNewline + 1, '\n');

            nextNewline += 1;

            while(1)
            {
            	// search for the next new line and atoi number
            	char *end = strchr(nextNewline, '\n');

            	printf("%s\r\n", end);

            	if(end == NULL)
            	{
            		break;
            	}

            	*end = '\0';

            	int16_t neighborId  = atoi(nextNewline);

            	printf("Neighbor ID: %d\r\n", neighborId);

            	gateway_data.adjacencyList[peerId][neighborId] = 1;

            	// If we haven't already visited this neighbor, add it to the list of neighbors to visit
            	if(gateway_data.peerList[neighborId] == -1)
            	{
            		gateway_data.peerList[neighborId] = 0;
            	}

            	nextNewline = end + 1;

            }

            break;
            
        case STATE_ROUTE_DONE:
            if(gateway_data.stateTransition)
            {
            	nrk_time_get(&(gateway_data.stateTransitionTime));
            	nrk_kprintf (PSTR ("--> All nodes are done routing\r\n"));
                // perform transition related outputs here
                nrk_time_get(&(gateway_data.lastRouteTime));
            }

            // done receiving routing information from all possible nodes
            // we need to assemble the graph and decide what the best path is
            // send out routing information to all the nodes at the end


            for(uint16_t i = 0; i < MAX_NUM_PEERS; i++)
            {
            	for(uint16_t j = 0; j < MAX_NUM_PEERS; j++)
				{
					printf("%d ", gateway_data.adjacencyList[i][j]);
				}
				nrk_kprintf(PSTR("\r\n"));
            }

            break;

        case STATE_RECEIVE_SENSOR_VALUES:
            if(gateway_data.stateTransition)
            {
            	nrk_kprintf (PSTR ("--> Reading sensor values from nodes\r\n"));
                // perform transition related outputs here
            }

            // send out command to all the sensor nodes telling them to send me their values
            // we will start from the farthest away nodes and work our way in
            // this will happen through sending CTS:<farthest 1>
            // wait a second or two
            // send CTS:<fathest 2>
            // wait a second or two
            // ... for n nodes
            // each node only transmits once
            // receive the values, print them to screen
            // also, implement timeout and indicate which paths didn't report their values and keep track of status
            // 

            break;

        case STATE_OFF:
        default:
            // should never reach here
            break;
    }

    gateway_data.presentState = state;
}

/*****************************************
 *              GATEWAY TASK             *
 *****************************************/

/**
  * Initialize the gateway data struct and other variables
  */
void gateway_init(void)
{
  gateway_data.desiredState = STATE_MENU;
  gateway_data.presentState = STATE_OFF;
  gateway_data.stateTransition = false;
  gateway_data.serialInput = ' ';
  gateway_data.seenCtsState = false;
  gateway_data.networkRefreshPeriod = NETWORK_REFRESH_DEFAULT_PERIOD;


  for(uint8_t index = 0; index < MAX_NUM_PEERS; index++)
  {
  	gateway_data.peerList[index] = -1;
  }

}

void gateway_task(void)
{
    nrk_kprintf (PSTR ("Gateway Task started\r\n"));

    gateway_init(); // Initialize gateway variables

    // Get the signal for UART RX
    uart_rx_signal = nrk_uart_rx_signal_get();

    // Register task to wait on signal
    nrk_signal_register(uart_rx_signal);

    // init bmac on channel 12 and max power
    bmac_init (CHANNEL);
    bmac_set_rf_power(32);

    // This can be called at anytime before releasing the packet
    // if you wish to do a zero-copy buffer switch
    bmac_rx_pkt_set_buffer(rx_buf, RF_MAX_PAYLOAD_SIZE);

    while(1)
    {
        /*
         * Below is the main state machine loop
         */

        // Grab inputs and do some processing
        gateway_processData();

        // See if state transition is necessary
        gateway_getDesiredState();

        // Set variable indicating state transition happened
        gateway_data.stateTransition = (gateway_data.presentState != gateway_data.desiredState);

        // Perform transition and state related outputs
        gateway_setCurrentState();

        nrk_wait_until_next_period();
    }

}

/*****************************************
 *             MAIN FUNCTION             *
 *****************************************/

/**
  * Creates a nrk gateway task
  */
void nrk_create_taskset ()
{
    GATEWAY_TASK.task = gateway_task;
    nrk_task_set_stk( &GATEWAY_TASK, gateway_task_stack, NRK_APP_STACKSIZE);
    GATEWAY_TASK.prio = 2;
    GATEWAY_TASK.FirstActivation = TRUE;
    GATEWAY_TASK.Type = BASIC_TASK;
    GATEWAY_TASK.SchType = PREEMPTIVE;
    GATEWAY_TASK.period.secs = 0;
    GATEWAY_TASK.period.nano_secs = TASK_PERIOD_NS;
    GATEWAY_TASK.cpu_reserve.secs = 1;
    GATEWAY_TASK.cpu_reserve.nano_secs = 0;
    GATEWAY_TASK.offset.secs = 0;
    GATEWAY_TASK.offset.nano_secs = 0;
    nrk_activate_task (&GATEWAY_TASK);
    
    nrk_kprintf ("Create done\r\n");
}

int main ()
{
    nrk_setup_ports ();
    nrk_setup_uart (UART_BAUDRATE_115K2);
    
    nrk_init ();
    
    nrk_led_clr (0);
    nrk_led_clr (1);
    nrk_led_clr (2);
    nrk_led_clr (3);
    
    nrk_time_set (0, 0);
    
    bmac_task_config ();
    
    nrk_create_taskset ();
    nrk_start ();
    
    return 0;
}