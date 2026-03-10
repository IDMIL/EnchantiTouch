/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the PSoC 4 MSC CapSense CSD Button
* Tuning code example for ModusToolbox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_canfd.h"
#include "cy_gpio.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "cycfg_peripherals.h"
#include "cycfg_pins.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <cy_systick.h>

/*******************************************************************************
* Macros
*******************************************************************************/
#define CAPSENSE_MSC0_INTR_PRIORITY      (3u)
#define CAPSENSE_MSC1_INTR_PRIORITY      (3u)
#define CY_ASSERT_FAILED                 (0u)
#define MSC_CAPSENSE_WIDGET_INACTIVE     (0u)

/* EZI2C interrupt priority must be higher than CapSense interrupt. */
#define EZI2C_INTR_PRIORITY              (2u)
#define CANFD_INTR_PRIORITY              (3u)
#define SPI_AUX_INTR_PRIORITY            (2u)

/* Define Board Mode Macros*/
#define TOUCHSIZE (120u)
#define BOARD_TOUCHSIZE (60u)
#define MAIN_TOUCH_BOARD (0u)
#define AUX_TOUCH_BOARD (1u)
#define SPI_TIMEOUT (1000)
#define SPI_BUFFERSIZE (128u)

/*******************************************************************************
* Global Variables
********************************************************************************/
cy_stc_scb_ezi2c_context_t ezi2c_context;
#if CY_CAPSENSE_BIST_EN
/* Variables to hold sensor parasitic capacitances and status*/
uint32_t button0_cp = 0, button1_cp = 0;
cy_en_capsense_bist_status_t button0_cp_status, button1_cp_status;
#endif /* CY_CAPSENSE_BIST_EN */
/* New touch variable (set to 1 if there is new touch data to send) **/
uint16_t newData = 0;
/* Touch data buffers*/
// Use 8bit subaddress size
typedef struct touchBuffer
{    
    uint8_t u8_reserve;                                    // addr 0, reserved for further use,
    uint8_t u8_boardmode;                                  // addr 1, is it the main board or aux board,
    uint8_t u8_numboards;                                  // addr 2, number of boards,
    uint8_t u8_touchmode;                                  // addr 3, reserved for further use,
    uint16_t u16_signal[TOUCHSIZE];                        // addr 4 - 124,
}touchBuffer;	

typedef struct board_config
{
    uint8_t position;       // Board Position
    uint8_t segment;        // Touch segment
    uint16_t scan_time;     // Scan time
} board_config_t;

struct touchBuffer touch1Data; // All touch data

/* Timing variables */
int start, end; // timing variable
float slot_scan_time = 0;
float sensor_scan_time = 0;
uint16_t scan_time = 0;
float total_t = 0;
int systick_count = 0;
float TRIALS = 20000.0f;
int SENSORS_PER_TRIAL = 30.0f;
int counter = 0;

/* CANFD Variables */
#define TOUCH_BUFFER_OFFSET 60 // 
cy_stc_canfd_context_t canfd0_context;
bool touch_data_received = false;
uint32_t touch_buffer1[CY_CANFD_MESSAGE_DATA_BUFFER_SIZE];
uint32_t touch_buffer2[CY_CANFD_MESSAGE_DATA_BUFFER_SIZE];
uint32_t touch_buffer3[CY_CANFD_MESSAGE_DATA_BUFFER_SIZE];
uint32_t touch_buffer4[CY_CANFD_MESSAGE_DATA_BUFFER_SIZE];
uint32_t BOARD_POSITION = 0;
uint32_t ENABLE_CANFD = 0;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static uint32_t get_board_position(void);
static void initialize_capsense(void);
static void capsense_msc0_isr(void);
static void capsense_msc1_isr(void);
static void saveTouchData(void);
static void ezi2c_isr(void);
static void initialize_i2c(void);
static void initialise_canfd(void);
static void canfd_isr();
static void sendTouch(void);
static float timedifference_msec(void);
static float timedifference_usec(void);
static void systick_isr(void);
static uint32_t get_tick(void);

#if CY_CAPSENSE_BIST_EN
static void measure_sensor_cp(void);
#endif /* CY_CAPSENSE_BIST_EN */

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - initialize CapSense
*  - perform Cp measurement if Built-in Self test (BIST) is enabled
*  - scan touch input continuously
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Enable timer */
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU, 0x00FFFFFF);
    Cy_SysTick_SetCallback(0UL, &systick_isr);

    /* touch buffer Initialization */
    uint16_t i;  

    touch1Data.u8_reserve = 0x00u; 
    touch1Data.u8_boardmode = 0x00u;
    touch1Data.u8_numboards = 0x01u;
    touch1Data.u8_touchmode = 0x00u;   

    /* Initialise CANFD buffers*/
    for(i=0; i<CY_CANFD_MESSAGE_DATA_BUFFER_SIZE; i++)
    {   
        // Main touch buffer
        touch_buffer1[i]   = 0x0000u;
        touch_buffer2[i]   = 0x0000u;
        touch_buffer3[i]   = 0x0000u;
        touch_buffer4[i]   = 0x0000u;
    }

    /* Find out board type */
    BOARD_POSITION = get_board_position();

    /* Initialize EZI2C */
    initialize_i2c();

    /* Initialize CANFD*/
    initialise_canfd();

    /* Initialize MSC CapSense */
    initialize_capsense();

#if CY_CAPSENSE_BIST_EN
    /* Measure the self capacitance of sensor electrode using BIST */
    measure_sensor_cp();
#endif /* CY_CAPSENSE_BIST_EN */
    /* start time */
    start = Cy_SysTick_GetValue();

    /* Start the first scan */
    Cy_CapSense_ScanAllSlots(&cy_capsense_context);

    for (;;)
    {
        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            // /* Save touch data to touch buffers */
            saveTouchData();
            
            /* Start the next scan */
            Cy_CapSense_ScanAllSlots(&cy_capsense_context);

            /* Compute sensor scan time */
            end = get_tick();
            total_t = timedifference_usec();
            scan_time = (uint16_t)total_t;
            start = end;

            /* Send data to host MCU */
            if ((newData == 1) && (BOARD_POSITION != 1)) {
                sendTouch();
            }

            // /* Toggles GPIO for refresh rate measurement. Probe at P3.4. */
            Cy_GPIO_Inv(CYBSP_SENSE_SCAN_RATE_PORT, CYBSP_SENSE_SCAN_RATE_NUM);

            // /* Sends interrupt if there is touch data */
            if (newData) {
                Cy_GPIO_Inv(CYBSP_EVT_PORT, CYBSP_EVT_NUM);
                newData = 0;
            }                
        }
    }
    // /* update time */
    // end = get_tick();
    // total_t = timedifference_sec();
    // slot_scan_time = 1000 * total_t / TRIALS; // scan time per 30 sensors (us)
    // sensor_scan_time = slot_scan_time / SENSORS_PER_TRIAL; // scan time per sensor (us)
    // printf("Time taken to scan %d sensors %f times: %f ms", SENSORS_PER_TRIAL, TRIALS, total_t);
}

/*******************************************************************************
* Function Name: get_board_position
********************************************************************************
* Summary:
*  Returns the board position based on the values of pos1, pos2 pins
*
*******************************************************************************/
static uint32_t get_board_position(void) {
    uint32_t pos1Value = 0;
    uint32_t pos2Value = 0;
    uint32_t board_pos = 0;

    /* Read board values */
    pos1Value = Cy_GPIO_Read(POS_P1_PORT, POS_P1_NUM);
    pos2Value = Cy_GPIO_Read(POS_P2_PORT, POS_P2_NUM);

    // Get board type
    if (!pos1Value && !pos2Value) {
        board_pos = 1;
    } else if (!pos1Value && pos2Value) {
        board_pos = 2;
    } else if (pos1Value && !pos2Value) {
        board_pos = 3;
    } else if (pos1Value && pos2Value) {
        board_pos = 4;
    } else {
        // Default to board position 1
        board_pos = 1;
    }

    // Return board position
    return board_pos;
}


/*******************************************************************************
* Function Name: get tick
********************************************************************************
* Summary:
*  Get current systick value
*
*******************************************************************************/

static uint32_t get_tick(void)
{
    return ((0xFFFFFF - Cy_SysTick_GetValue()) + (systick_count * 0x1000000));
}

/*******************************************************************************
* Function Name: timedifference_msec
********************************************************************************
* Summary:
*  This function returns the time difference in ms
*
*******************************************************************************/

static float timedifference_msec(void)
{
    int tick_dif = end - start;
    if (tick_dif < 0) {
        tick_dif = -1 * tick_dif;
    }
    float time_diff = tick_dif * 1000.0f;
    time_diff = time_diff / 16777216.0f;
    return time_diff;
}

/*******************************************************************************
* Function Name: timedifference_msec
********************************************************************************
* Summary:
*  This function returns the time difference in ms
*
*******************************************************************************/

static float timedifference_usec(void)
{
    int tick_dif = end - start;
    if (tick_dif < 0) {
        tick_dif = -1 * tick_dif;
    }
    float time_diff = tick_dif * 1000000.0f;
    time_diff = time_diff / 16777216.0f;
    return time_diff;
}


/*******************************************************************************
* Function Name: SysTick_Callback
****************************************************************************/
static void systick_isr(void)
{
    /* Some action */
    systick_count++;
}

/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configures the CapSense
*  interrupt.
*
*******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CapSense interrupt configuration MSC 0 */
    const cy_stc_sysint_t capsense_msc0_interrupt_config =
    {
        .intrSrc = CY_MSC0_IRQ,
        .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };

    /* CapSense interrupt configuration MSC 1 */
    const cy_stc_sysint_t capsense_msc1_interrupt_config =
    {
        .intrSrc = CY_MSC1_IRQ,
        .intrPriority = CAPSENSE_MSC1_INTR_PRIORITY,
    };

    /* Capture the MSC HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        /* Initialize CapSense interrupt for MSC 0 */
        Cy_SysInt_Init(&capsense_msc0_interrupt_config, capsense_msc0_isr);
        NVIC_ClearPendingIRQ(capsense_msc0_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc0_interrupt_config.intrSrc);

        /* Initialize CapSense interrupt for MSC 1 */
        Cy_SysInt_Init(&capsense_msc1_interrupt_config, capsense_msc1_isr);
        NVIC_ClearPendingIRQ(capsense_msc1_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc1_interrupt_config.intrSrc);

        /* Initialize the CapSense firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if(status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CapSense sensors are tuned
         * as per procedure give in the Readme.md file */
    }
}


/*******************************************************************************
* Function Name: capsense_msc0_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense MSC0 block.
*
*******************************************************************************/
static void capsense_msc0_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC0_HW, &cy_capsense_context);

    uint32_t intrStatus;
    /* Read interrupt status register */
    intrStatus = Cy_MSC_ReadReg(MSC0, CY_MSC_REG_OFFSET_INTR);
    /* Check an event that triggered the interrupt */
    if ((intrStatus & MSC_INTR_SET_SCAN_Msk) == MSC_INTR_SET_SCAN_Msk)
    {
        /* End of scan occurred, get the result and do something with it here */
    }
    /* Clear pending interrupt */
    Cy_MSC_WriteReg(MSC0, CY_MSC_REG_OFFSET_INTR, intrStatus);
    (void)Cy_MSC_ReadReg(MSC0, CY_MSC_REG_OFFSET_INTR);
}


/*******************************************************************************
* Function Name: capsense_msc1_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense MSC1 block.
*
*******************************************************************************/
static void capsense_msc1_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC1_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: saveTouchData(void)
********************************************************************************
* Summary:
*  Function for saving touch data to touch data buffers
*
*******************************************************************************/
static void saveTouchData(void){
    uint16_t i;
    for(i=0;i<BOARD_TOUCHSIZE;i++)
    {
        if (i < 30) {
            if (touch1Data.u16_signal[59-i] != cy_capsense_tuner.sensorContext[i].diff) {
                newData = 1;
            }
            touch1Data.u16_signal[59-i] = cy_capsense_tuner.sensorContext[i].diff;
        } else if (i >= 30) {
            if (touch1Data.u16_signal[i-30] != cy_capsense_tuner.sensorContext[i].diff) {
                newData = 1;
            }
            touch1Data.u16_signal[i-30] = cy_capsense_tuner.sensorContext[i].diff;
        }
        
    }
}


/*******************************************************************************
* Function Name: initialize_i2c
********************************************************************************
* Summary:
* EZI2C module to communicate with the CapSense Tuner tool.
*
*******************************************************************************/
static void initialize_i2c(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2c_intr_config =
    {
        .intrSrc = CYBSP_EZI2C_IRQ,
        .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezi2c_context);

    if(status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set the CapSense data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8_t *)&touch1Data,
                            sizeof(touch1Data), sizeof(touch1Data),
                            &ezi2c_context);

    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);
}

/*******************************************************************************
* Function Name: ezi2c_isr
********************************************************************************
* Summary:
* Wrapper function for handling interrupts from EZI2C block.
*
*******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezi2c_context);
}

/*******************************************************************************
* Function Name: initialize_canfd
********************************************************************************
* Summary:
*  This function initializes the CANFD peripheral
*
*******************************************************************************/

static void initialise_canfd(void)
{
    if(CY_CANFD_SUCCESS != Cy_CANFD_Init (CANFD0, 0, &CANFD0_config, &canfd0_context))
    {
        /* Error processing */
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Populate the configuration structure */
    const cy_stc_sysint_t canfd_irq_cfg =
    {
        /* .intrSrc */ canfd_interrupts0_0_IRQn, /* CAN FD interrupt number */
        /* .intrPriority */ CANFD_INTR_PRIORITY
    };
    /* Hook the interrupt service routine and enable the interrupt */
    (void) Cy_SysInt_Init(&canfd_irq_cfg, &canfd_isr);
    NVIC_EnableIRQ(canfd_interrupts0_0_IRQn);
}


/*******************************************************************************
* Function Name: canfd_isr()
********************************************************************************
* Summary:
* Wrapper function for enabling CANFD interrupts
*
*******************************************************************************/
/* CANFD interrupt handler */
static void canfd_isr(void)
{
    /* Just call the IRQ handler with the current channel number and context */
    Cy_CANFD_IrqHandler(CANFD0, 0, &canfd0_context);
}

/*******************************************************************************
* Function Name: sendTouch
********************************************************************************
* Summary:
* Function to gsend touch data over CANFD bus
*******************************************************************************/
static void sendTouch(void)
{
    /* Add board position to the first index */
    board_config_t board_conf1 = {
        .position = BOARD_POSITION,
        .segment = 1,
        .scan_time = scan_time,
    };
    board_config_t board_conf2 = {
        .position = BOARD_POSITION,
        .segment = 2,
        .scan_time = scan_time,
    };
    
    /* Copy board config to buffer */
    memcpy(&CANFD0_txBuffer_0.data_area_f[0], &board_conf1, sizeof(board_conf1));
    memcpy(&CANFD0_txBuffer_1.data_area_f[0], &board_conf2, sizeof(board_conf2));

    /* Save touch data to buffer */
    memcpy(&CANFD0_txBuffer_0.data_area_f[1], &touch1Data.u16_signal[0], 60);
    memcpy(&CANFD0_txBuffer_1.data_area_f[1], &touch1Data.u16_signal[30], 60);

    /* Sends the prepared data using tx buffer 1 and waits for 1000ms */
    Cy_CANFD_UpdateAndTransmitMsgBuffer(CANFD0, 0u, &CANFD0_txBuffer_0, 0u, &canfd0_context);
    Cy_CANFD_UpdateAndTransmitMsgBuffer(CANFD0, 0u, &CANFD0_txBuffer_1, 1u, &canfd0_context);
}

/*******************************************************************************
* Function Name: sendTouch
********************************************************************************
* Summary:
* Function to gsend touch data over CANFD bus
*******************************************************************************/

/* CANFD reception callback */
void CAN_RxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum,
                       cy_stc_canfd_rx_buffer_t* canfd_rx_buf)
{
    /* Get data from receive buffer */
    /* Checking whether the frame received is a data frame */
    if(CY_CANFD_RTR_DATA_FRAME == canfd_rx_buf->r0_f->rtr) 
    {
        // Get board config
        board_config_t conf = {0};

        // Copy board conf
        memcpy(&conf, &canfd_rx_buf->data_area_f[0], sizeof(conf));

        // Make sure board position is correct
        if (conf.position != 1) {
            if (conf.segment == 1) {
                /* Copy receive data to transfer buffer */
                CANFD0_txBuffer_2.data_area_f = canfd_rx_buf->data_area_f;

                /* Copy data to touch array */
                memcpy(&touch1Data.u16_signal[60], &canfd_rx_buf->data_area_f[1], 60);
            } else if (conf.segment == 2) {
                /* Copy receive data to transfer buffer */
                CANFD0_txBuffer_3.data_area_f = canfd_rx_buf->data_area_f;

                /* Copy data to touch array */
                memcpy(&touch1Data.u16_signal[90], &canfd_rx_buf->data_area_f[1], 60);
            }
        }


        // Acknowledge message
        if (bRxFifoMsg) {
            Cy_CANFD_AckRxFifo(CANFD0, 0UL, u8MsgBufOrRxFifoNum);
        } else {
            Cy_CANFD_AckRxBuf(CANFD0, 0UL, u8MsgBufOrRxFifoNum);
        }
    }
}


#if CY_CAPSENSE_BIST_EN
/*******************************************************************************
* Function Name: measure_sensor_cp
********************************************************************************
* Summary:
*  Measures the self capacitance of the sensor electrode (Cp) in Femto Farad and
*  stores its value in the variable button0_cp and button1_cp.
*
*******************************************************************************/
static void measure_sensor_cp(void)
{
    /* Measure the self capacitance of sensor 0 electrode */
    button0_cp_status = Cy_CapSense_MeasureCapacitanceSensorElectrode(CY_CAPSENSE_BUTTON0_WDGT_ID,
                                                  CY_CAPSENSE_BUTTON0_SNS0_ID, &cy_capsense_context);
    button0_cp = cy_capsense_context.ptrWdConfig[CY_CAPSENSE_BUTTON0_WDGT_ID].ptrEltdCapacitance[CY_CAPSENSE_BUTTON0_SNS0_ID];

    /* Measure the self capacitance of sensor 1 electrode */
    button1_cp_status = Cy_CapSense_MeasureCapacitanceSensorElectrode(CY_CAPSENSE_BUTTON1_WDGT_ID,
                                                  CY_CAPSENSE_BUTTON1_SNS0_ID, &cy_capsense_context);
    button1_cp = cy_capsense_context.ptrWdConfig[CY_CAPSENSE_BUTTON1_WDGT_ID].ptrEltdCapacitance[CY_CAPSENSE_BUTTON1_SNS0_ID];
}
#endif /* CY_CAPSENSE_BIST_EN */



/* [] END OF FILE */
