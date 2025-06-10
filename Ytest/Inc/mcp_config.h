
/**
  ******************************************************************************
  * @file    mcp_config.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides configuration definition of the MCP protocol
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#ifndef MCP_CONFIG_H
#define MCP_CONFIG_H

#include "mcp.h"
#include "aspep.h"
#include "mcpa.h"

#define USARTA USART2
#define DMA_RX_A DMA2
#define DMA_TX_A DMA2
#define DMACH_RX_A LL_DMA_CHANNEL_2
#define DMACH_TX_A LL_DMA_CHANNEL_3
#define USARTA_IRQHandler USART2_IRQHandler
#define MCP_RX_IRQHandler_A DMA2_Channel2_IRQHandler
#define MCP_USER_CALLBACK_MAX 2

#define MCPA_OVER_UARTA_STREAM 10

extern ASPEP_Handle_t aspepOverUartA;
extern MCP_Handle_t MCP_Over_UartA;
extern MCPA_Handle_t MCPA_UART_A;
extern MCP_user_cb_t MCP_UserCallBack[MCP_USER_CALLBACK_MAX];
#endif /* MCP_CONFIG_H */

/************************ (C) COPYRIGHT 2021 STMicroelectronics *****END OF FILE****/
