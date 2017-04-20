/**
  * File: main.c
  * 18-748: Wireless Sensor Networks
  * Authors: Lucas Bruder, Jeremy Xiao, and Pallavi Kuman
  * Team 12
  * Lab 3: Multi-hop
  * Last modified: 2/14/2016
  *
  * This is code loaded on to peer firefly node for lab 3.
  *
  *   TODO:
  *		  GOT_ACK state needs modification in the case of a mesh network, the first route found might not be the best
  * 	  * In transition to RTS state, need to clear all the strings and arrays so we don't get same results from last time
  * 	
  *		  Notes: Things to think about during routing algorithm:
  *		         How many times a node has to send TEST before something responds
  *				 How many hops
  *
  *		   TODO fix unexpected restart
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
#include <nrk_timer.h>
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <ff_basic_sensor.h>
 
/*****************************************
 *                 DEFINES               *
 *****************************************/

#define MAC_ADDR              (uint16_t)0 // MAC address of gateway node
#define CHANNEL             		   12 // channel to communicate on
#define MAX_SECONDS_BUF_LEN  		    4 // maximum number of digits to accept for updated refresh or sampling rate time

#define MAX_SECONDS_TEST_STATE_NO_ACK	3 // maximum number of seconds to wait in the TEST state without an ACK message

#define MAX_TIMES_SEND_SEEN				3	
#define MAX_NUM_PEERS	4

#define MAX_SENDING_ACK_SECS			5

/*****************************************
 *                 TYPEDEFS              *
 *****************************************/

 typedef enum{

 	/*
 	 * All routing related states
 	 */
    STATE_WAIT,	// initial turn on state, waiting for route command from gateway
    STATE_RTS,	// sending out RTS messages, waiting for CTS from gateway
    STATE_TEST, // received CTS command for this node, sending out test messages and waiting for neighbor nodes to respond with ACK
    STATE_LISTENING, // received CTS command but not this node, start listening for test messages. 
    STATE_SENDING_ACKS, // received test message, start sending out ACK:<my mac>
    STATE_GOT_ACK, // got acknowledged by another peer with SEEN message, wait for CTS from gateway
    STATE_DONE, // node didn't receive an ACK from another node for certain period of time, now transmit neighbor information to GTW
    STATE_FORWARDING, // waiting for DONE messages from other nodes with information to forward to gateway

 } peer_state_E;

 typedef struct{
    peer_state_E desiredState;
    peer_state_E presentState;;
    bool stateTransition;

    uint16_t networkRefreshPeriod;
    uint16_t sensorUpdatePeriod;

    uint8_t *receive_buf;
    uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
    uint8_t len;
    int8_t rssi;

    nrk_time_t currentTime;
    nrk_time_t transitionTime;
    nrk_time_t testStateLastReceivedAckMessage;

    bool receivedTestMessage;
    bool receivedSeenMessage;
    bool receivedCtsFromGateway;
    bool receivedAckFromGateway;
    bool sendSeen;
    uint8_t sendSeenCount;

    char done_message[RF_MAX_PAYLOAD_SIZE];

    uint8_t nodesSeen[MAX_NUM_PEERS];

 } peer_data_S;

/*****************************************
 *             FUNCTION HEADERS          *
 *****************************************/

void peer_task(void);
void nrk_create_taskset(void);
void nrk_register_drivers();

void peer_init(void);

// Send and receive functions for wireless communication
bool peer_receiveMessage(void);
bool peer_sendMessage(void);

// State machine processing
void peer_processData(void);
void peer_getDesiredState(void);
void peer_setCurrentState(void);

/*

 * State machine transitions
 */


/*****************************************
 *              VARIABLES                *
 *****************************************/

/*
 * Gateway task
 */
nrk_task_type PEER_TASK;
NRK_STK peer_task_stack[NRK_APP_STACKSIZE];
uint8_t peer_buf[RF_MAX_PAYLOAD_SIZE];

// RX and TX buffers
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
nrk_time_t start_time_receive;
nrk_time_t end_time_receive;
bool receivedMessage;

nrk_sig_t uart_rx_signal;

peer_data_S peer_data;

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
bool peer_receiveMessage(void)
{
    bool received = false;
    /*
     *  Code takes care of receiving and sets timeout = true if there's a timeout receiving a packet
     */
    while(1)
    {
        if(bmac_rx_pkt_ready())
        {
            peer_data.receive_buf = bmac_rx_pkt_get (&(peer_data.len), &(peer_data.rssi));
            received = true;
            // printf ("Got RX packet len=%d RSSI=%d [", len, rssi);
            // for (i = 0; i < len; i++)
            // {
            //     // printf ("%c", local_buf[i]);
            //     // printf ("]\r\n");
            // }
            
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
bool peer_sendMessage(void)
{
    int8_t val;
    bool sent = true;

    val=bmac_tx_pkt(peer_data.tx_buf, strlen(peer_data.tx_buf)+1);

    if(val != NRK_OK)
    {
        nrk_kprintf(PSTR("Could not Transmit!\r\n"));
        sent = false;
        nrk_led_set(RED_LED);
    }
    else
    {
    	nrk_led_clr(RED_LED);
    }

    nrk_time_get(&start_time_receive);

    return sent;
}

/**
  * Allow transition from waiting to sending RTS if we receive a route command from the 
  * gateway
  * @return true if transition is allowed, false otherwise
  */
inline bool peer_allowTransitionWaitToRTS(void)
{
	bool allowTransition = false;

	if((receivedMessage == true) && (strcmp(ROUTE_STRING, peer_data.receive_buf) == 0))
	{
		allowTransition = true;
	}

	return allowTransition;
}

/**
  * Allow transition from sending RTS to sending out TEST messages if we get a CTS:<my MAC>
  * from the Gateway
  */
inline bool peer_allowTransitionRTSToTest(void)
{
	bool allowTransition = false;
	uint16_t peerId = 0;

	if(receivedMessage == true)
	{
		char *buf = (char*)peer_data.receive_buf;

		// retreive peer id from Gateway command to see if it's this node's MAC
		if(peer_data.len > 4 && buf[0] == 'C' && buf[1] == 'T' && buf[2] == 'S' && buf[3] == ':')
		{
			buf = buf + 4;
			peerId = atoi(buf);

			if(peerId == MAC_ADDR)
			{
				allowTransition = true;
			}
		}
	}

	return allowTransition;
}

/**
  * Allow transition from sending RTS to listening for TEST messages if we get a CTS:<not my MAC> from the
  * Gateway
  */
inline bool peer_allowTransitionRTSToListening(void)
{
	bool allowTransition = false;
	uint16_t peerId = 0;

	if(receivedMessage == true)
	{
		char *buf = (char*)peer_data.receive_buf;

		// retreive peer id from Gateway command to see if it's this node's MAC
		if(peer_data.len > 4 && buf[0] == 'C' && buf[1] == 'T' && buf[2] == 'S' && buf[3] == ':')
		{
			buf = buf + 4;
			peerId = atoi((char*)buf);

			if(peerId != MAC_ADDR)
			{
				allowTransition = true;
			}
		}
	}

	return allowTransition;
}

/**
  * Allow transition from sending out TEST messages to DONE state if a certain amount of time has passed
  * where this node hasn't received an ACK message from another node
  */
inline bool peer_allowTransitionTestToDone(void)
{
	bool allowTransition = false;

	nrk_time_get(&(peer_data.currentTime));

	if((peer_data.currentTime.secs - peer_data.testStateLastReceivedAckMessage.secs) > MAX_SECONDS_TEST_STATE_NO_ACK)
	{
		allowTransition = true;
	}

	return allowTransition;
}

/**
  * Allow transition from the listening state to sending acknowledgement messages if we receive a test
  * message from another peer 
  */
inline bool peer_allowTransitionListeningToSendingAcks(void)
{
	bool allowTransition = false;

	if(peer_data.receivedTestMessage == true)
	{
		allowTransition = true;
		peer_data.receivedTestMessage = false;
	}

	return allowTransition;
}

/**
  * Allow transition from sending acknowledgements to getting an acknoledgement if 
  * the current peer transmitting sends us the SEEN:<my MAC>
  */
inline bool peer_allowTransitionSendingAcksToGotAck(void)
{
	bool allowTransition = false;

	nrk_time_get(&(peer_data.currentTime));

	if(peer_data.receivedSeenMessage == true || peer_data.currentTime.secs - peer_data.transitionTime.secs > MAX_SENDING_ACK_SECS)
	{
		allowTransition = true;
	}

	return allowTransition;
}

/**
  * Allow transition from getting acknowledged from another peer to sending out TEST messages
  * if we received a CTS from the gateway 
  */
inline bool peer_allowTransitionGotAckToTest(void)
{
	bool allowTransition = false;

	if(peer_data.receivedCtsFromGateway == true)
	{
		allowTransition = true;
	}


	return allowTransition;
}

/**
  * Allow transition from done to forwarding if we receive an ACK from the gateway
  */
inline bool peer_allowTransitionDoneToForwarding(void)
{
	bool allowTransition = false;

	if(peer_data.receivedAckFromGateway == true)
	{
		allowTransition = true;
	}

	return allowTransition;
}

/** 
  * Allow transitin from listening to TEST messages to sending out RTS messages if we receieve
  * a ROUTE command from the gateway
  */
inline bool peer_allowTransitionListeningToRTS(void)
{
	bool allowTransition = false;

	if((receivedMessage == true) && (strcmp(ROUTE_STRING, peer_data.receive_buf) == 0))
	{
		allowTransition = true;
	}

	return allowTransition;
}

inline bool peer_allowTransitionSendingAcksToRTS(void)
{
	bool allowTransition = false;

	if((receivedMessage == true) && peer_data.len > 5 && peer_data.receive_buf[0] == 'G' && peer_data.receive_buf[1] == 'T' 
		&& peer_data.receive_buf[2] == 'W' && peer_data.receive_buf[3] == ':' && peer_data.receive_buf[4] == 'R' )
	{
		allowTransition = true;
	}

	return allowTransition;
}

/*
 * Process inputs
 */
void peer_processData(void)
{
    // receive any messages
    receivedMessage = peer_receiveMessage();

    // receive any messages
}

/*
 * Get the present state and determine if a state transition shall be performed. If so, set the 
 * desired state to that new state.
 */
void peer_getDesiredState(void)
{
  peer_state_E desiredState = peer_data.desiredState;

  switch(desiredState)
  {
        case STATE_WAIT:
        	if(peer_allowTransitionWaitToRTS() == true)
        	{
        		desiredState = STATE_RTS;
        	}
        	else
        	{
        		// keep state
        	}

        	break;

	    case STATE_RTS:
	    	if(peer_allowTransitionRTSToTest() == true)
	    	{
	    		desiredState = STATE_TEST;
	    	}
	    	else if(peer_allowTransitionRTSToListening() == true)
	    	{
	    		desiredState = STATE_LISTENING;
	    	}
	    	else
	    	{
	    		// keep state
	    	}

	    	break;

	    case STATE_TEST:
	    	if(peer_allowTransitionTestToDone() == true)
	    	{
	    		desiredState = STATE_DONE;
	    	}
	    	else
	    	{
	    		// keep state
	    	}

	    	break;

	    case STATE_LISTENING:
	    	if(peer_allowTransitionListeningToSendingAcks() == true)
	    	{
	    		desiredState = STATE_SENDING_ACKS;
	    	}
	    	else if(peer_allowTransitionListeningToRTS() == true)
	    	{
	    		desiredState = STATE_RTS;
	    	}
	    	else
	    	{
	    		// keep state
	    	}

	    	break;

	    case STATE_SENDING_ACKS:
	    	if(peer_allowTransitionSendingAcksToGotAck() == true)
	    	{
	    		desiredState = STATE_GOT_ACK;
	    	}
	    	else if(peer_allowTransitionSendingAcksToRTS() == true)
	    	{
	    		desiredState = STATE_RTS;
	    	}
	    	else
	    	{
	    		// keep state
	    	}

	    	break;

	    case STATE_GOT_ACK:
	    	if(peer_allowTransitionGotAckToTest() == true)
	    	{
	    		desiredState = STATE_TEST;
	    	}
	    	else
	    	{
	    		// keep state
	    	}

	    	break;

	    case STATE_DONE:
	    	if(peer_allowTransitionDoneToForwarding() == true)
	    	{
	    		desiredState = STATE_FORWARDING;
	    	}
	    	else
	    	{
	    		// keep state
	    	}

	    	break;

	    case STATE_FORWARDING:

	    	break;

        default:
            // should never reach here, error
             break;
  }
    
    peer_data.desiredState = desiredState;
}

/*
 * Perform any transition and state related outputs
 */
void peer_setCurrentState(void)
{
    peer_state_E state = peer_data.desiredState;
    uint16_t peerId = 0;

    switch(state)
    {
        case STATE_WAIT:
        	if(peer_data.stateTransition == true)
        	{
            	nrk_kprintf (PSTR ("--> Node just turned on, waiting for route command\r\n"));
        	}

        	// do nothing in the wait state, just wait for GTW:ROUTE command

        	break;

	    case STATE_RTS:
        	if(peer_data.stateTransition == true)
        	{
    			nrk_kprintf (PSTR ("--> Received routing command, starting to send RTS messages\r\n"));
			nrk_led_clr(RED_LED);
			nrk_led_clr(GREEN_LED);
			nrk_led_clr(ORANGE_LED);
	        	nrk_led_set(BLUE_LED);

	        	// Clear everything
        	}

        	/*
        	 * Send out RTS messages with this nodes MAC address, waiting for the GTW to respond
        	 */
        	sprintf((peer_data.tx_buf), "RTS:%d", MAC_ADDR);
        	peer_sendMessage();
        
	    	break;

	    case STATE_TEST:
        	if(peer_data.stateTransition == true)
        	{
        		nrk_led_clr(BLUE_LED);
				nrk_led_clr(GREEN_LED);
				nrk_led_clr(ORANGE_LED);
        		nrk_led_set(RED_LED);

        		nrk_kprintf (PSTR ("--> Received CTS command from GTW. Testing to see if nodes are in vicinity.\r\n"));

        		// grab time transition into state
        		nrk_time_get(&(peer_data.testStateLastReceivedAckMessage));
        	}

        	if(receivedMessage == true)
        	{
        		uint8_t *buf = (peer_data.receive_buf);

        		// check to see if we received an ACK message from another peer and add them to the neighbors list
        		// to send to the gateway
        		if(peer_data.len > 4 && buf[0] == 'A' && buf[1] == 'C' && buf[2] == 'K' && buf[3] == ':')
        		{
        			buf = buf + 4;

        			peerId = atoi(buf);

        			peer_data.sendSeen = true;

        			if(peer_data.nodesSeen[peerId] == 0)
        			{
						nrk_time_get(&(peer_data.testStateLastReceivedAckMessage));

        				strcat(peer_data.done_message, buf);
        				strcat(peer_data.done_message, "\n");
						peer_data.nodesSeen[peerId] = 1;
        			}

        		}
        	}

        	if(peer_data.sendSeen == true)
        	{
				sprintf(peer_data.tx_buf, "SEEN:%d", peerId);
				peer_sendMessage();
				peer_data.sendSeenCount += 1;

				if(peer_data.sendSeenCount >= MAX_TIMES_SEND_SEEN)
				{
					peer_data.sendSeenCount = 0;
					peer_data.sendSeen = false;
				}
			}
			else
			{
	        	sprintf((peer_data.tx_buf), "TEST:%d", MAC_ADDR);
        		peer_sendMessage();
			}



	    	break;

	    case STATE_LISTENING:
        	if(peer_data.stateTransition == true)
        	{
        		nrk_kprintf (PSTR ("--> Listening for TEST commands from a nearby node \r\n"));
        	}

		nrk_led_clr(BLUE_LED);
		nrk_led_set(GREEN_LED);
		nrk_led_clr(ORANGE_LED);
        	nrk_led_clr(RED_LED);

        	uint8_t *buf = (peer_data.receive_buf);

        	if(receivedMessage == true)
        	{
        		// check to see if received message is a TEST message
        		if(peer_data.len > 5 && buf[0] == 'T' && buf[1] == 'E' && buf[2] == 'S' && buf[3] == 'T' && buf[4] == ':' )
        		{
        			peer_data.receivedTestMessage = true;
        		}
        	}

	    	break;

	    case STATE_SENDING_ACKS:
        	if(peer_data.stateTransition == true)
        	{
        	   nrk_kprintf (PSTR ("--> Saw a test command, attempting to send acknoledgement\r\n"));
        	   nrk_time_get(&(peer_data.transitionTime));
        	}

		nrk_led_set(BLUE_LED);
		nrk_led_set(GREEN_LED);
		nrk_led_clr(ORANGE_LED);
        	nrk_led_clr(RED_LED);
        	// Let the peer sending test messages know that I am there by sending ACK messages with my mac address
        	sprintf(peer_data.tx_buf, "ACK:%d", MAC_ADDR);
        	peer_sendMessage();

        	if(receivedMessage == true)
        	{
        		// check to see if received message is a TEST message
        		if(peer_data.len > 5 && buf[0] == 'S' && buf[1] == 'E' && buf[2] == 'E' && buf[3] == 'N' && buf[4] == ':' )
        		{
        			buf = buf + 5;
        			peerId = atoi(buf);

        			if(peerId == MAC_ADDR)
        			{
        				peer_data.receivedSeenMessage = true;
        			}
        		}
        	}

	    	break;

	    case STATE_GOT_ACK:
	    	if(peer_data.stateTransition == true)
        	{
        		nrk_kprintf (PSTR ("--> Received a SEEN command from nearby peer, waiting for CTS from GTW\r\n"));
        	}

		nrk_led_set(BLUE_LED);
		nrk_led_clr(GREEN_LED);
		nrk_led_clr(ORANGE_LED);
        	nrk_led_set(RED_LED);
        	// at this point, this node will be called by the gateway as CTS in the near future, so 
        	// we wait for a CTS:<my MAC> from the gateway before transitioning to the test state, looking
        	// for neighbor nodes
        	if(receivedMessage == true)
        	{
        		// check to see if received message is a TEST message
        		if(peer_data.len > 4 && buf[0] == 'C' && buf[1] == 'T' && buf[2] == 'S' && buf[3] == ':')
        		{
        			buf = buf + 4;
        			peerId = atoi(buf);

        			if(peerId == MAC_ADDR)
        			{
        				peer_data.receivedCtsFromGateway = true;
        			}
        		}
        	}

	    	break;

	    case STATE_DONE:
        	if(peer_data.stateTransition == true)
        	{
        		nrk_kprintf (PSTR ("--> Done receiving ACK messages from nearby peers, sending results to GTW\r\n"));
		    	nrk_led_clr(RED_LED);
		    	nrk_led_clr(BLUE_LED);
		    	nrk_led_set(GREEN_LED);
			nrk_led_set(ORANGE_LED);
        	}

        	// Done getting neighbors list, build information from nodes and send to gateway
        	sprintf(peer_data.tx_buf, "DONE\nFROM:%d\nTO:GTW\nSEEN\n", MAC_ADDR);      	
		strcat(peer_data.tx_buf, peer_data.done_message);

        	peer_sendMessage();

        	if(receivedMessage == true)
        	{
        		char* buf = peer_data.receive_buf;

        		if(peer_data.len > 4 && buf[0] == 'A' && buf[1] == 'C' && buf[2] == 'K' && buf[3] == ':')
        		{
        			buf = buf + 4;

        			if(atoi(buf) == MAC_ADDR)
        			{
        				peer_data.receivedAckFromGateway = true;
        			}
        		}
        	}

	    	break;

	    case STATE_FORWARDING:
        	if(peer_data.stateTransition == true)
        	{
        		nrk_kprintf (PSTR ("--> Waiting to forward any neighbor information from other nodes\r\n"));
        		nrk_led_set(BLUE_LED);
        		nrk_led_set(RED_LED);
        		nrk_led_set(GREEN_LED);
			nrk_led_set(ORANGE_LED);
	
        	}

        	// listen to messages from nearby peers and route any DONE messages to the gateway

	    	break;

        default:
            // should never reach here, error
             break;
    }

    peer_data.presentState = state;
}

/*****************************************
 *              GATEWAY TASK             *
 *****************************************/

/**
  * Initialize the peer data struct and other variables
  */
void peer_init(void)
{
  peer_data.desiredState = STATE_WAIT;
  peer_data.presentState = STATE_WAIT;
  peer_data.stateTransition = false;
}

void peer_task(void)
{
    nrk_kprintf (PSTR ("Peer Task started\r\n"));

    peer_init(); // Initialize gateway variables

    // Get the signal for UART RX
    uart_rx_signal = nrk_uart_rx_signal_get();

    // Register task to wait on signal
    nrk_signal_register(uart_rx_signal);

    // init bmac on channel 12 and least power to assist multihop
    bmac_init (CHANNEL);
    bmac_set_rf_power(5);

    // This can be called at anytime before releasing the packet
    // if you wish to do a zero-copy buffer switch
    bmac_rx_pkt_set_buffer(rx_buf, RF_MAX_PAYLOAD_SIZE);

    while(1)
    {

        /*
         * Below is the main state machine loop
         */

        // Grab inputs and do some processing
        peer_processData();

        // See if state transition is necessary
        peer_getDesiredState();

        // Set variable indicating state transition happened
        peer_data.stateTransition = (peer_data.presentState != peer_data.desiredState);

        // Perform transition and state related outputs
        peer_setCurrentState();

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
    PEER_TASK.task = peer_task;
    nrk_task_set_stk( &PEER_TASK, peer_task_stack, NRK_APP_STACKSIZE);
    PEER_TASK.prio = 2;
    PEER_TASK.FirstActivation = TRUE;
    PEER_TASK.Type = BASIC_TASK;
    PEER_TASK.SchType = PREEMPTIVE;
    PEER_TASK.period.secs = 0;
    PEER_TASK.period.nano_secs = 500000000;
    PEER_TASK.cpu_reserve.secs = 1;
    PEER_TASK.cpu_reserve.nano_secs = 0;
    PEER_TASK.offset.secs = 0;
    PEER_TASK.offset.nano_secs = 0;
    nrk_activate_task (&PEER_TASK);
    
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
