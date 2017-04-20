/******************************************************************************
 *  Nano-RK, a real-time operating system for sensor networks.
 *  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
 *  All rights reserved.
 *
 *  This is the Open Source Version of Nano-RK included as part of a Dual
 *  Licensing Model. If you are unsure which license to use please refer to:
 *  http://www.nanork.org/nano-RK/wiki/Licensing
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2.0 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *******************************************************************************/

/**
  * File: main.c
  * 18-748: Wireless Sensor Networks
  * Authors: Lucas Bruder, Jeremy Xiao, and Pallavi Kuman
  * Team 12
  * Lab 2
  * Last modified: 2/8/2016
  *
  * This is code loaded on to master firefly node for lab 2. Reponsible for implementing whack a mole game logic.
  *
  */

#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <hal.h>
#include <bmac.h>
#include <nrk_error.h>

#define MAC_ADDR                15  
#define NUM_SLAVES               3 
  
#define MAX_NUM_MOLES           10 // number of moles played before game is over

#define MAX_TIMEOUT_COUNTS      10 // max number of receive timeouts allowed before assuming the slave is dead
#define MOLE_TIMOUT_SECS        15 // number of seconds until mole timeouts and switches to different node
#define MOLE_TIMEOUT_PENALTY    20 // penalty from letting a mole timeout


/**
 * State enumeration for master device
 */
typedef enum
{
    STATE_MOLE,
    STATE_WHACKED,
    STATE_POLL
} states_E;

/*
 * Whacky task
 */
nrk_task_type WHACKY_TASK;
NRK_STK whacky_task_stack[NRK_APP_STACKSIZE];
uint8_t whacky_buf[RF_MAX_PAYLOAD_SIZE];

void whacky_task (void);
void nrk_create_taskset ();

char buffer[20];       // Buffer for receive decryption 
uint8_t num_moles = 0; // Number of moles seen (see MAX_NUM_MOLES to increase number of moles per game)

// RX and TX buffers
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];

uint16_t previous_slave_id; // keeps track of previous mole so we don't choose the same one twice
int16_t score = 0;         // score of game

states_E state = STATE_MOLE;    // state of game

uint8_t dead_devices[NUM_SLAVES]; // keeps track of dead devices

void nrk_register_drivers();

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

void whacky_task ()
{
    uint8_t i, len;
    int8_t rssi, val;
    uint8_t *local_buf;
    uint8_t slave_id;
    uint8_t timeout;
    uint8_t mole_timeout = 0; // set to 1 if the mole timed out, set to 0 otherwise
    char option;
    nrk_sig_t uart_rx_signal;

    uint16_t slave_id_received; // slave id received from the slave
    uint16_t light_value;       // light value received from the slave during polling state

    uint8_t timeout_counts = 0;

    //Timer management for receiving timeout
    nrk_time_t start_time_receive, end_time_receive;
    nrk_time_t start_time_mole, end_time_mole;
    nrk_time_t time_rx, time_tx;
    srand(time(NULL));

    // printf ("whacky_task PID=%d\r\n", nrk_get_pid ());
    
    // This shows you how to wait until a key is pressed to start
    nrk_kprintf( PSTR("Press 's' to start\r\n" ));
    
    // Get the signal for UART RX
    uart_rx_signal=nrk_uart_rx_signal_get();

    // Register task to wait on signal
    nrk_signal_register(uart_rx_signal);
    
    do{
        if(nrk_uart_data_ready(NRK_DEFAULT_UART))
        {
            option=getchar();
        }

        else nrk_event_wait(SIG(uart_rx_signal));

    } while(option!='s');
    
    // init bmac on channel 12 and max power
    bmac_init (12);
    bmac_set_rf_power(32);
    
    // This sets the next RX buffer.
    // This can be called at anytime before releasing the packet
    // if you wish to do a zero-copy buffer switch
    bmac_rx_pkt_set_buffer (rx_buf, RF_MAX_PAYLOAD_SIZE);
    
    // initialize the slave id
    slave_id = rand() % NUM_SLAVES;
    
    while (1)
    {
        // printf("State: %d\r\n", state);

        /*
         * TRANSMIT BUFFER CONSTRUCTION BASED ON STATE
         */
        if(state == STATE_POLL)
        {
            sprintf (tx_buf, "POLL: %u", slave_id);

            nrk_led_set (BLUE_LED);
            val=bmac_tx_pkt(tx_buf, strlen(tx_buf)+1);

            if(val != NRK_OK)
            {
                nrk_kprintf(PSTR("Could not Transmit!\r\n"));
            }

            // Task gets control again after TX complete
            nrk_kprintf (PSTR ("-> POLL request\r\n"));
            nrk_led_clr (BLUE_LED);
        }
        else if(state == STATE_MOLE)
        {
            sprintf (tx_buf, "MOLE: %u", slave_id);

            nrk_led_set (BLUE_LED);
            val=bmac_tx_pkt(tx_buf, strlen(tx_buf)+1);
            nrk_time_get(&time_tx);
            printf("time_Tx: %lu", time_tx.secs);
            printf("time_Tx: %lu", time_tx.nano_secs);

            if(val != NRK_OK)
            {
                nrk_kprintf(PSTR("Could not Transmit!\r\n"));
            }

            // Task gets control again after TX complete
            nrk_kprintf (PSTR ("-> MOLE request\r\n"));
            nrk_led_clr (BLUE_LED);
        }
        else if(state == STATE_WHACKED)
        {
            sprintf (tx_buf, "WHACKED: %u", slave_id);
            nrk_led_set (BLUE_LED);
            val=bmac_tx_pkt(tx_buf, strlen(tx_buf)+1);

            if(val != NRK_OK)
            {
                nrk_kprintf(PSTR("Could not Transmit!\r\n"));
            }

            // Task gets control again after TX complete
            nrk_kprintf (PSTR ("-> WHACKED Request.\r\n"));
            nrk_led_clr (BLUE_LED);
        }
        else
        {
            // error
        }
       
        // nrk_kprintf(PSTR("Waiting for Response\r\n"));
        // Get the RX packet
        nrk_led_set (ORANGE_LED);
        
        // Wait until an RX packet is received
        timeout = 0;

        // DEBUG_PRINTF(timeout);
        nrk_time_get(&start_time_receive);
        /*
         *  Code takes care of receiving and sets timeout = 1 if there's a timeout receiving a packet
         */
        while(1)
        {
            if(bmac_rx_pkt_ready())
            {
                local_buf = bmac_rx_pkt_get (&len, &rssi);
                // printf ("Got RX packet len=%d RSSI=%d [", len, rssi);
                for (i = 0; i < len; i++)
                {
                    // printf ("%c", local_buf[i]);
                    // printf ("]\r\n");
                }
                
                nrk_led_clr (ORANGE_LED);
                // Release the RX buffer so future packets can arrive
                bmac_rx_pkt_release ();
                break;
            }
            
            // Implement timeouts
            nrk_time_get(&end_time_receive);
            nrk_time_get(&time_rx);
            printf("time_rx: %lu\r\n",time_rx.secs);
            printf("time_rx: %lu\r\n",time_rx.nano_secs);
            // printf("%d \r\n",(end_time_receive.secs-start_time_receive.secs)*1000-(start_time_receive.nano_secs-end_time_receive.nano_secs)/1000000);
            if(end_time_receive.nano_secs > start_time_receive.nano_secs)
            {

                if(((end_time_receive.secs-start_time_receive.secs)*1000+(end_time_receive.nano_secs-start_time_receive.nano_secs)/1000000) > 1000)
                {
                    timeout = 1;
                    break;
                }
            }
            else
            {
                if(((end_time_receive.secs-start_time_receive.secs)*1000-(start_time_receive.nano_secs-end_time_receive.nano_secs)/1000000) > 1000)
                {
                    timeout = 1;
                    break;
                }
            }
        }
        
        /*
         * If there's a timeout, don't execute any code code below and go back to while loop
         */ 
        if(timeout == 1)
        {
            nrk_kprintf(PSTR("Rx Timed Out!\r\n"));
            timeout_counts++;

            /*
             * Self-healing logic
             * 
             * If we get too many timeouts from one node, we will set the master to the mole state, 
             * save the slave id that "died" so we don't get it again, and choose a new slave_id, ensuring
             * we choose a different one from last time AND we choose a node that hasn't been dead in the past
             * 
             * IMPORTANT: If there are only 2 slaves, this will cause a problem because if one of them is dead,
             * it leaves only one node and the while(1) below will spin infinitely because it can't choose a slave twice
             * in a row
             */
            if(timeout_counts >= MAX_TIMEOUT_COUNTS)
            {
                nrk_kprintf( PSTR("! Max timeout count reached !\r\n" ));
                // Set to mole state and choose a new slave_id
                state = STATE_MOLE;

                // Ensure we never choose this slave again because its dead
                dead_devices[slave_id] = 1;
                previous_slave_id = slave_id;

                // Choose new slave id
                while(1)
                {
                    slave_id = rand()%NUM_SLAVES;

                    if((dead_devices[slave_id] == 0) && (slave_id != previous_slave_id) )  // This loop becomes an infinite loop when there is only one node left
                    {
                        break;
                    }
                }

                timeout_counts = 0; // reset timeout counts for new slave device
            }
            continue;
        }

        /*
         *  DECODE RECEIVED STRING
         */
        if(state == STATE_POLL)
        {   
            slave_id_received = atoi(strtok((char*)local_buf,":"));

            // printf("%s\n", slave_id);
            light_value=atoi(strtok(NULL,"\n"));
            // printf("%s\n",light_value);

            // check length of time elapsed for each iteration of the loop
            nrk_time_get(&end_time_mole);

            uint16_t poll_time_elapsed = end_time_mole.secs - start_time_mole.secs;

            // Tell mole to whack itself, and set boolean to indicate to subtract from score 
            if(poll_time_elapsed > (MOLE_TIMOUT_SECS - num_moles))
            {
                nrk_kprintf( PSTR("! Mole timed out !\r\n" ));
                mole_timeout = 1;
                state = STATE_WHACKED;
            }
            else if ((light_value < 600) && (slave_id_received == slave_id))
            {
                nrk_kprintf( PSTR("! Mole whacked !\r\n" ));
                state = STATE_WHACKED;
            }

        }   
        else if(state == STATE_MOLE)
        {
            sprintf(buffer, "GOT_MOLE: %u", slave_id);
            /**
              * Change to polling state if received string is equal to "GOT_MOLE: <slave_id>"
              */ 
            if (strcmp(buffer,(char*)local_buf)==0)
            {
                num_moles++;
                state = STATE_POLL;

                // start a timer here
                nrk_time_get(&start_time_mole);
            }
        }
        else if(state == STATE_WHACKED)
        {
            sprintf(buffer,"GOT_WHACKED: %u", slave_id);

            /**
              * Change to polling state if received string is equal to "GOT_WHACKED: <slave_id>"
              */ 
            if (strcmp(buffer,(char*)local_buf)==0)
            {
                // Set previous slave id and find new one not equal to the old one
                previous_slave_id = slave_id;

                // Stop timer
                nrk_time_get(&end_time_mole);
                
                /*
                 * Choose new slave_id, ensuring we don't pick our previous one AND ensuring we don't pick a dead one
                 */
                while(1)
                {
                    slave_id = rand() % NUM_SLAVES;

                    // Only let this be the new slave_id if it's not dead and its not equal to the previous slave id
                    if((dead_devices[slave_id] == 0) && (slave_id != previous_slave_id)) 
                    {
                        break;
                    }
                }    

                uint16_t seconds_to_whack = end_time_mole.secs - start_time_mole.secs;
                printf("Recieve time: %lu", seconds_to_whack);

                if(mole_timeout == 0)
                {
                    score += seconds_to_whack + num_moles;
                }
                else
                {
                    score += MOLE_TIMEOUT_PENALTY + num_moles;
                    mole_timeout = 0;
                }

                /*
                 * Only play MAX_NUM_MOLES times
                 */
                if(num_moles >= MAX_NUM_MOLES)
                {
                    break;
                }

                state = STATE_MOLE;

                printf("Seconds to whack mole = %d\r\n", seconds_to_whack);
                printf("Your score after this turn is = %d\r\n", score);
            }
        }
        else
        {
            // error
        }
        
    }
    
    nrk_kprintf (PSTR ("The Game is Over.\r\n"));
    nrk_led_set (BLUE_LED);
    nrk_led_set (ORANGE_LED);
    nrk_led_set (RED_LED);
    nrk_led_set (GREEN_LED);

    // Print out score achieved
    printf("FINAL SCORE: %d\n", score); 

    // Game over
    while(1);
}

void nrk_create_taskset ()
{
    WHACKY_TASK.task = whacky_task;
    nrk_task_set_stk( &WHACKY_TASK, whacky_task_stack, NRK_APP_STACKSIZE);
    WHACKY_TASK.prio = 2;
    WHACKY_TASK.FirstActivation = TRUE;
    WHACKY_TASK.Type = BASIC_TASK;
    WHACKY_TASK.SchType = PREEMPTIVE;
    WHACKY_TASK.period.secs = 1;
    WHACKY_TASK.period.nano_secs = 0;
    WHACKY_TASK.cpu_reserve.secs = 2;
    WHACKY_TASK.cpu_reserve.nano_secs = 0;
    WHACKY_TASK.offset.secs = 0;
    WHACKY_TASK.offset.nano_secs = 0;
    nrk_activate_task (&WHACKY_TASK);
    
    nrk_kprintf ("Create done\r\n");
}