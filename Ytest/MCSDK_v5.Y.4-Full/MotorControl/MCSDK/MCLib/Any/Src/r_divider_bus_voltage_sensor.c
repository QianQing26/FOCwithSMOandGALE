/**
  ******************************************************************************
  * @file    r_divider_bus_voltage_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the  features
  *          of the Resistor Divider Bus Voltage Sensor component of the Motor
  *          Control SDK:
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "r_divider_bus_voltage_sensor.h"
#include "regular_conversion_manager.h"


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */

/** @defgroup RDividerBusVoltageSensor Resistor Divider Bus Voltage Sensor
  * @brief Resistor Divider Bus Voltage Sensor implementation
  *
  * @todo Document the Resistor Divider Bus Voltage Sensor "module".
  *
  * @{
  */

/**
  * @brief  It initializes bus voltage conversion (ADC, ADC channel, conversion time.
    It must be called only after PWMC_Init.
  * @param  pHandle related RDivider_Handle_t
  * @retval none
  */
__weak void RVBS_Init(RDivider_Handle_t *pHandle)
{
#ifdef NULL_PTR_RDIV_BUS_VLT_SNS
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    /* Need to be register with RegularConvManager */
    pHandle->convHandle = RCM_RegisterRegConv(&pHandle->VbusRegConv);
    /* Check */
    RVBS_Clear(pHandle);
#ifdef NULL_PTR_RDIV_BUS_VLT_SNS
  }
#endif
}


/**
  * @brief  It clears bus voltage FW variable containing average bus voltage
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval none
  */
__weak void RVBS_Clear(RDivider_Handle_t *pHandle)
{
#ifdef NULL_PTR_RDIV_BUS_VLT_SNS
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint16_t aux;
    uint16_t index;

    aux = (pHandle->OverVoltageThreshold + pHandle->UnderVoltageThreshold) / 2U;
    for (index = 0U; index < pHandle->LowPassFilterBW; index++)
    {
      pHandle->aBuffer[index] = aux;
    }
    pHandle->_Super.LatestConv = aux;
    pHandle->_Super.AvBusVoltage_d = aux;
    pHandle->index = 0U;
#ifdef NULL_PTR_RDIV_BUS_VLT_SNS
  }
#endif
}

static uint16_t RVBS_ConvertVbusFiltrered(RDivider_Handle_t *pHandle)
{


  uint32_t tot = 0U;
  uint16_t hAux;
  uint16_t max = 0U;
  uint16_t min = 0U;
  uint16_t tempValue;
  uint8_t vindex;

  for (vindex = 0U; vindex < pHandle->LowPassFilterBW; vindex++)
  {
    hAux = RCM_ExecRegularConv(pHandle->convHandle);
    while (0xFFFFU == hAux)
    {
      hAux = RCM_ExecRegularConv(pHandle->convHandle);
    }

    if (0U == vindex)
    {
      min = hAux;
      max = hAux;
    }
    else
    {
      if (hAux < min)
      {
        min = hAux;
      }
      else
      {
        /* Nothing to do */
      }
      if (hAux > max)
      {
        max = hAux;
      }
      else
      {
        /* Nothing to do */
      }
    }

    tot += hAux;
  }

  tot -= max;
  tot -= min;
  tempValue = (uint16_t)(tot / (uint16_t)(pHandle->LowPassFilterBW - 2U));
  return (tempValue);
}

/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
__weak uint16_t RVBS_CalcAvVbusFilt(RDivider_Handle_t *pHandle)
{
  uint16_t tempValue;
#ifdef NULL_PTR_RDIV_BUS_VLT_SNS
  if (MC_NULL == pHandle)
  {
    tempValue = 0U;
  }
  else
  {
#endif
    uint32_t wtemp;
    uint16_t hAux;
    uint8_t i;

    hAux = RVBS_ConvertVbusFiltrered(pHandle);

    if (0xFFFFU == hAux)
    {
      /* Nothing to do */
    }
    else
    {
      pHandle->aBuffer[pHandle->index] = hAux;
      wtemp = 0u;
      for (i = 0U; i < pHandle->LowPassFilterBW; i++)
      {
        wtemp += pHandle->aBuffer[i];
      }
      wtemp /= pHandle->LowPassFilterBW;
      pHandle->_Super.AvBusVoltage_d = (uint16_t)wtemp;
      pHandle->_Super.LatestConv = hAux;

      if (pHandle->index < (pHandle->LowPassFilterBW - 1U))
      {
        pHandle->index++;
      }
      else
      {
        pHandle->index = 0U;
      }
    }

    pHandle->_Super.FaultState = RVBS_CheckFaultState(pHandle);
    tempValue = pHandle->_Super.FaultState;
#ifdef NULL_PTR_RDIV_BUS_VLT_SNS
  }
#endif
  return (tempValue);
}

/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
__weak uint16_t RVBS_CalcAvVbus(RDivider_Handle_t *pHandle)
{
  uint16_t tempValue;
#ifdef NULL_PTR_RDIV_BUS_VLT_SNS
  if (MC_NULL == pHandle)
  {
    tempValue = 0U;
  }
  else
  {
#endif
    uint32_t wtemp;
    uint16_t hAux;
    uint8_t i;

    hAux = RCM_ExecRegularConv(pHandle->convHandle);

    if (0xFFFFU == hAux)
    {
      /* Nothing to do */
    }
    else
    {
      pHandle->aBuffer[pHandle->index] = hAux;
      wtemp = 0u;
      for (i = 0U; i < (uint8_t)pHandle->LowPassFilterBW; i++)
      {
        wtemp += pHandle->aBuffer[i];
      }
      wtemp /= pHandle->LowPassFilterBW;
      pHandle->_Super.AvBusVoltage_d = (uint16_t)wtemp;
      pHandle->_Super.LatestConv = hAux;

      if ((uint16_t)pHandle->index < (pHandle->LowPassFilterBW - 1U))
      {
        pHandle->index++;
      }
      else
      {
        pHandle->index = 0U;
      }
    }

    pHandle->_Super.FaultState = RVBS_CheckFaultState(pHandle);
    tempValue = pHandle->_Super.FaultState;
#ifdef NULL_PTR_RDIV_BUS_VLT_SNS
  }
#endif
  return (tempValue);
}

/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage and protection threshold values
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
__weak uint16_t RVBS_CheckFaultState(RDivider_Handle_t *pHandle)
{
  uint16_t fault;
#ifdef NULL_PTR_RDIV_BUS_VLT_SNS
  if (MC_NULL == pHandle)
  {
    fault = MC_SW_ERROR;
  }
  else
  {
#endif
    if (pHandle->_Super.AvBusVoltage_d > pHandle->OverVoltageThreshold)
    {
      fault = MC_OVER_VOLT;
    }
    else if (pHandle->_Super.AvBusVoltage_d < pHandle->UnderVoltageThreshold)
    {
      fault = MC_UNDER_VOLT;
    }
    else
    {
      fault = MC_NO_ERROR;
    }
#ifdef NULL_PTR_RDIV_BUS_VLT_SNS
  }
#endif
  return (fault);
}


/**
  * @}
  */

/**
  * @}
  */

/** @} */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

