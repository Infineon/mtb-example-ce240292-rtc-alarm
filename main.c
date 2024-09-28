/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RTC alarm interrupt Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/******************************************************************************
* Macros
*******************************************************************************/
#define SECONDS_PER_MIN          (60u)  /* used to keep values in range */
#define TICK_INTERVAL            (3u)   /* seconds or minutes interval. The range should be 1-59 */
#define RTC_INITIAL_DATE_SEC      0u    /* Initial seconds value */
#define RTC_INITIAL_DATE_MIN      0u    /* Initial minutes value */
#define RTC_INITIAL_DATE_HOUR     0u    /* Initial hours value */
#define RTC_INITIAL_DATE_DOW      1u    /* Initial day of the week */
#define RTC_INITIAL_DATE_DOM      1u    /* Initial day of the month */
#define RTC_INITIAL_DATE_MONTH    1u    /* Initial month */
#define RTC_INTERRUPT_PRIORITY    7u    /* Interrupt priority level */

/*******************************************************************************
* Global Variables
*******************************************************************************/
volatile bool alarm_flag = 0;    /*Flag to indicate alarm triggering */

/* Alarm configuration structure */
cy_stc_rtc_alarm_t alarm_config =
{
    .sec            = RTC_INITIAL_DATE_SEC,
    .secEn          = CY_RTC_ALARM_DISABLE,
    .min            = RTC_INITIAL_DATE_MIN,
    .minEn          = CY_RTC_ALARM_DISABLE,
    .hour           = RTC_INITIAL_DATE_HOUR,
    .hourEn         = CY_RTC_ALARM_DISABLE,
    .dayOfWeek      = RTC_INITIAL_DATE_DOW,
    .dayOfWeekEn    = CY_RTC_ALARM_DISABLE,
    .date           = RTC_INITIAL_DATE_DOM,
    .dateEn         = CY_RTC_ALARM_DISABLE,
    .month          = RTC_INITIAL_DATE_MONTH,
    .monthEn        = CY_RTC_ALARM_DISABLE,
    .almEn          = CY_RTC_ALARM_ENABLE
};

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void rtc_interrupt_handler(void);
void rtc_step_alarm(void);

/*******************************************************************************
* Function Definitions
*******************************************************************************/


/******************************************************************************
* Function Name: rtc_interrupt_handler
*******************************************************************************
*
* Summary:
*  This is the general RTC interrupt handler in CPU NVIC.
*  It calls the Alarm2 interrupt handler if that is the interrupt that occurs.
*
* Parameters:
*  None
*
* Return:
*  None
*
******************************************************************************/
void rtc_interrupt_handler(void)
{
    /* No DST parameters are required for the custom tick. */
    Cy_RTC_Interrupt(NULL, false);

}

/******************************************************************************
* Function Name: Cy_RTC_Alarm2Interrupt
*******************************************************************************
*
* Summary:
*  The function sets the alarm flag for led toggle.
*
* Parameters:
*  None
*
* Return:
*  None
*
******************************************************************************/
void Cy_RTC_Alarm2Interrupt(void)
{
    /* Interrupt has occurred and the alarm_flag is set to 1 */
    alarm_flag = 1u;
}

/******************************************************************************
* Function Name: rtc_step_alarm
*******************************************************************************
*
* Summary:
*  This function sets the time for CY_RTC_ALARM_2, and configures the interrupt.
*  It retrieves the current time and adds the alarm for 3 seconds
* Parameters:
*  None
*
* Return:
*  None
*
******************************************************************************/
void rtc_step_alarm(void)
{
    cy_en_rtc_status_t rtc_result;
    cy_stc_rtc_config_t current_rtc_time;

    /* Get current Date and Time from RTC and update alarm_config */
    Cy_RTC_GetDateAndTime(&current_rtc_time);
    alarm_config.secEn = CY_RTC_ALARM_ENABLE;
    /* advance the second by the specified interval */
    alarm_config.sec = current_rtc_time.sec + TICK_INTERVAL;
    /* keep it in range, 0-59 */
    alarm_config.sec = alarm_config.sec % SECONDS_PER_MIN;

    do
    {
        /* set the new alarm based on updated values of date and time  */
        rtc_result = Cy_RTC_SetAlarmDateAndTime((cy_stc_rtc_alarm_t *)&alarm_config, CY_RTC_ALARM_2);
    } while (rtc_result != CY_RTC_SUCCESS);
    if(rtc_result != CY_RTC_SUCCESS)
    {
        CY_ASSERT(0u);
    }
}

/*******************************************************************************
* Function Name: main
*********************************************************************************
* Summary:
* System entrance point. This function performs
*    1. Initializes and configures the device and board peripherals.
*    2. Initialize the RTC and Alarm configuration.
*    3. RTC Interrupt configuration and enabling the Interrupts
*    4. The forever loop monitors the alarm_flag; upon activation, it toggles the LED,
*        schedules the next alarm, and enters deep sleep mode.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_en_rtc_status_t rtc_result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Setting the initial date and time */
    rtc_result = Cy_RTC_Init(&RTC_config);
    if (rtc_result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Set the alarm  */
    rtc_result = Cy_RTC_SetAlarmDateAndTime((cy_stc_rtc_alarm_t *)&alarm_config, CY_RTC_ALARM_2);
    if (rtc_result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure RTC interrupt by defining the interrupt source and priority level.  */
    cy_stc_sysint_t rtc_intr_config = {
            .intrSrc = srss_interrupt_backup_IRQn,
            .intrPriority = RTC_INTERRUPT_PRIORITY
            };

    /* Enable RTC interrupt */
    Cy_RTC_SetInterruptMask(CY_RTC_INTR_ALARM2);

    /* Configure RTC interrupt ISR */
    Cy_SysInt_Init(&rtc_intr_config, rtc_interrupt_handler);
    /* Clear pending interrupt */
    NVIC_ClearPendingIRQ(rtc_intr_config.intrSrc);
    /* Enable RTC interrupt */
    NVIC_EnableIRQ(rtc_intr_config.intrSrc);
    /* setup the alarm config */
    rtc_step_alarm();

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        /* The flag is set, meaning time has expired */
        if(alarm_flag)
        {
            alarm_flag = 0u;

            /* Toggle the LED */
            Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM);

            /* Set up the next alarm after 3 seconds */
            rtc_step_alarm();
        }

        /* Go to Deep Sleep mode until the next interrupt */
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }
}

/* [] END OF FILE */
