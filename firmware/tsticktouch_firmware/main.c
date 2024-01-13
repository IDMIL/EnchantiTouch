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
#include "cy_pdl.h"
#include "cy_scb_spi.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "cycfg_peripherals.h"
#include <stdint.h>


/*******************************************************************************
* Macros
*******************************************************************************/
#define CAPSENSE_MSC0_INTR_PRIORITY      (3u)
#define CAPSENSE_MSC1_INTR_PRIORITY      (3u)
#define CY_ASSERT_FAILED                 (0u)
#define MSC_CAPSENSE_WIDGET_INACTIVE     (0u)

/* EZI2C interrupt priority must be higher than CapSense interrupt. */
#define EZI2C_INTR_PRIORITY              (2u)
#define SPI_INTR_PRIORITY                (2u)
#define SPI_AUX_INTR_PRIORITY            (2u)

/* Define Board Mode Macros*/
#define TOUCHSIZE (60u)
#define MAIN_TOUCH_BOARD (0u)
#define AUX_TOUCH_BOARD (1u)
#define SPI_TIMEOUT (100)
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

struct touchBuffer touch1Data; // Main board touch data
struct touchBuffer touch2Data; // Auxillary board touch data

/* Allocate context for SPI operation */
cy_stc_scb_spi_context_t spiContext;
cy_stc_scb_spi_context_t spi_aux_Context;
unsigned long transfer_status;
uint32_t bytes_transferred;
uint32_t spi_main_count;
uint32_t spi_aux_count;
uint16_t txBuffer[SPI_BUFFERSIZE];

/* Define SPI Mode
    0 = Main board
    1 = Aux Board
*/
uint32_t BOARD_MODE = 0;
uint32_t ENABLE_SPI = 0;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void initialize_capsense(void);
static void capsense_msc0_isr(void);
static void capsense_msc1_isr(void);
static void saveTouchData(void);
static void ezi2c_isr(void);
static void spi_isr(void);
static void spi_aux_isr(void);
static void initialize_i2c(void);
static void initialize_spi(void);
static void getTouch(void);
static void sendTouch(void);

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

    /* touch buffer Initialization */
    uint16_t i;  

    touch1Data.u8_reserve = 0x00u; 
    touch1Data.u8_boardmode = 0x00u;
    touch1Data.u8_numboards = 0x01u;
    touch1Data.u8_touchmode = 0x00u;   
    touch2Data.u8_reserve = 0x00u;   
    touch2Data.u8_boardmode = 0x00u;
    touch2Data.u8_numboards = 0x01u;
    touch2Data.u8_touchmode = 0x00u; 
    for(i=0; i<TOUCHSIZE; i++)
    {   
        // Main touch buffer
        touch1Data.u16_signal[i]   = 0x0000u;

        // Aux touch buffer
        touch2Data.u16_signal[i]   = 0x0000u;
    }

    /* Initialise SPI buffer*/
    for(i=0; i<SPI_BUFFERSIZE; i++)
    {   
        // Main touch buffer
        txBuffer[i]   = 0x0000u;
    }

    /* Initialize EZI2C */
    initialize_i2c();

    /* Initialize SPI*/
    initialize_spi();

    /* Initialize MSC CapSense */
    initialize_capsense();

#if CY_CAPSENSE_BIST_EN
    /* Measure the self capacitance of sensor electrode using BIST */
    measure_sensor_cp();
#endif /* CY_CAPSENSE_BIST_EN */

    /* Start the first scan */
    Cy_CapSense_ScanAllSlots(&cy_capsense_context);

    for (;;)
    {
        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* */
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            // /* Save touch data to touch buffers */
            saveTouchData();
            
            /* Start the next scan */
            Cy_CapSense_ScanAllSlots(&cy_capsense_context);

            /* If there are more than one touchboards get SPI data from them */
            if (touch1Data.u8_numboards > 1){
                getTouch();
            }

            /* Send data to host MCU */
            sendTouch();

            // /* Toggles GPIO for refresh rate measurement. Probe at P3.4. */
            Cy_GPIO_Inv(CYBSP_SENSE_SCAN_RATE_PORT, CYBSP_SENSE_SCAN_RATE_NUM);

            // /* Sends interrupt if there is touch data */
            if (newData) {
                Cy_GPIO_Inv(CYBSP_EVT_PORT, CYBSP_EVT_NUM);
                newData = 0;
            }                
        }
    }
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
    for(i=0;i<TOUCHSIZE;i++)
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
    Cy_SCB_EZI2C_SetBuffer2(CYBSP_EZI2C_HW, (uint8_t *)&touch2Data,
                            sizeof(touch2Data), sizeof(touch2Data),
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
* Function Name: initialize_spi
********************************************************************************
* Summary:
* Initialise the SPI comms in master/slave mode
*
*******************************************************************************/
static void initialize_spi(void)
{
    cy_en_scb_spi_status_t status = CY_SCB_SPI_SUCCESS;

    /* Init SPI */
    // Initialise SPI Slave device
    status = Cy_SCB_SPI_Init(CYBSP_SPI_HW, &CYBSP_SPI_config, &spiContext);

    // Initialise SPI Master device
    status = Cy_SCB_SPI_Init(CYBSP_SPI_AUX_HW, &CYBSP_SPI_AUX_config, &spi_aux_Context);

    if(status != CY_SCB_SPI_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Populate configuration structure */
    const cy_stc_sysint_t spi_main_IntrConfig =
    {
        .intrSrc      = CYBSP_SPI_IRQ,
        .intrPriority = SPI_INTR_PRIORITY,
    };

    /* Populate configuration structure */
    const cy_stc_sysint_t spi_aux_IntrConfig =
    {
        .intrSrc      = CYBSP_SPI_AUX_IRQ,
        .intrPriority = SPI_AUX_INTR_PRIORITY,
    };

    /* Enable Interrupts*/
    Cy_SysInt_Init(&spi_main_IntrConfig, &spi_isr);
    Cy_SysInt_Init(&spi_aux_IntrConfig, &spi_aux_isr);

    NVIC_EnableIRQ(CYBSP_SPI_IRQ);
    NVIC_EnableIRQ(CYBSP_SPI_AUX_IRQ);

    /* Enable SPI to operate */
    Cy_SCB_SPI_Enable(CYBSP_SPI_HW);
    Cy_SCB_SPI_Enable(CYBSP_SPI_AUX_HW);
}

/*******************************************************************************
* Function Name: spi_isr
********************************************************************************
* Summary:
* Wrapper function for handling interrupts from SPI block.
*
*******************************************************************************/
static void spi_isr(void)
{
    Cy_SCB_SPI_Interrupt(CYBSP_SPI_HW, &spiContext);
}

/*******************************************************************************
* Function Name: spi_aux_isr
********************************************************************************
* Summary:
* Wrapper function for handling interrupts from SPI block.
*
*******************************************************************************/
static void spi_aux_isr(void)
{
    Cy_SCB_SPI_Interrupt(CYBSP_SPI_AUX_HW, &spi_aux_Context);
}

/*******************************************************************************
* Function Name: getTouch
********************************************************************************
* Summary:
* Function to get touch data over SPI
*
*******************************************************************************/
static void getTouch(void)
{
    // Create empty buffers for SPI (we will use the same structure as the I2C buffers)
    cy_en_scb_spi_status_t status;
    uint16_t rxBuffer[SPI_BUFFERSIZE];

    for(int i=0; i<TOUCHSIZE; i++)
    {   
        rxBuffer[i] = 0;
        rxBuffer[i+60] = 0;
    }

    /* Save touch data to buffer */
    int i = 0;

    /* Master: start a transfer. Slave: prepare for a transfer. */
    status = Cy_SCB_SPI_Transfer(CYBSP_SPI_AUX_HW, NULL, (uint8_t *)&rxBuffer, sizeof(rxBuffer), &spi_aux_Context);

    if (status == CY_SCB_SPI_SUCCESS) {
        while (0UL != (CY_SCB_SPI_TRANSFER_ACTIVE & Cy_SCB_SPI_GetTransferStatus(CYBSP_SPI_AUX_HW, &spi_aux_Context)))
        {
        }

        /* Save data to touch buffer*/
        for(i=0; i<TOUCHSIZE; i++)
        {   
            touch2Data.u16_signal[i] = rxBuffer[i];
        }
    }
}

/*******************************************************************************
* Function Name: sendTouch
********************************************************************************
* Summary:
* Function to gsend touch data over SPI to auxillary touch board
*******************************************************************************/
static void sendTouch(void)
{
    /* Master: start a transfer. Slave: prepare for a transfer. */
    // if (spi_main_count > SPI_TIMEOUT) {
    //     Cy_SCB_SPI_AbortTransfer(CYBSP_SPI_HW, &spiContext);
    //     spi_main_count = 0;
    // }

    /* Save touch data to buffer */
    for(int i=0; i<TOUCHSIZE; i++)
    {   
        txBuffer[i] = touch1Data.u16_signal[i];
        txBuffer[i+60] = touch2Data.u16_signal[i];
    }

    /* Master: start a transfer. Slave: prepare for a transfer. */
    Cy_SCB_SPI_Transfer(CYBSP_SPI_HW, (uint8_t *)&txBuffer, NULL, sizeof(txBuffer), &spiContext);
    while ((0UL != (CY_SCB_SPI_TRANSFER_ACTIVE & Cy_SCB_SPI_GetTransferStatus(CYBSP_SPI_HW, &spiContext)))) 
    {
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
