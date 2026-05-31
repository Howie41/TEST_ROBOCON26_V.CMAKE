/**
 * @file robot_task.h
 * @author 大帅将军 ，Keten (2863861004@qq.com)
 * @brief 任务管理和任务间通讯
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
/* RTOS层及mcu main接口 */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "task.h"

/* bsp 层接口头文件 */
#include "bsp_dwt.h"
#include "chassis_task.h"
#include "com_config.h"
#include "control_task.h"
#include "debug_task.h"
#include "NavProtocol.hpp"
#include "lift_task.h"
#include "tail_claw_task.hpp"
/* module层接口头文件 */

/* Definitions for TaskHand */
extern osThreadId_t CAN1_Send_TaskHandle;
extern osThreadId_t CAN2_Send_TaskHandle;
extern osThreadId_t CAN3_Send_TaskHandle;
extern osThreadId_t uart2ProcessTaskHandle;
extern osThreadId_t uart3ProcessTaskHandle;
extern osThreadId_t Debug_TaskHandle;
extern osThreadId_t ChassisTaskHandle;
extern osThreadId_t ControlTaskHandle;
extern osThreadId_t usbcdcProcessTaskHandle;
extern osThreadId_t tail_claw_TaskHandle;
extern osThreadId_t NavControlTaskHandle;
extern osThreadId_t LiftTaskHandle;
extern osThreadId_t PcComTaskHandle;


void osTaskInit(void) {
  const osThreadAttr_t CAN1_SendTaskHandle_attributes = {
      .name = "CAN1_Send_TaskHandle",
      .stack_size = 256 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  CAN1_Send_TaskHandle =
      osThreadNew(can1SendTask, NULL, &CAN1_SendTaskHandle_attributes);

  const osThreadAttr_t CAN2_SendTaskHandle_attributes = {
      .name = "CAN2_Send_TaskHandle",
      .stack_size = 256 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  CAN2_Send_TaskHandle =
      osThreadNew(can2SendTask, NULL, &CAN2_SendTaskHandle_attributes);

  const osThreadAttr_t CAN3_SendTaskHandle_attributes = {
      .name = "CAN3_Send_TaskHandle",
      .stack_size = 256 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  CAN3_Send_TaskHandle =
      osThreadNew(can3SendTask, NULL, &CAN3_SendTaskHandle_attributes);

  const osThreadAttr_t DebugTaskHandle_attributes = {
      .name = "Debug_TaskHandle",
      .stack_size = 512 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  Debug_TaskHandle = osThreadNew(debugTask, NULL, &DebugTaskHandle_attributes);

  const osThreadAttr_t ChassisTaskHandle_attributes = {
      .name = "Chassis_TaskHandle",
      .stack_size = 512 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  ChassisTaskHandle =
      osThreadNew(chassisTask, NULL, &ChassisTaskHandle_attributes);

  const osThreadAttr_t ControlTaskHandle_attributes = {
      .name = "Control_TaskHandle",
      .stack_size = 512 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  ControlTaskHandle =
      osThreadNew(controlTask, NULL, &ControlTaskHandle_attributes);

  const osThreadAttr_t Uart2ProcessTaskHandle_attributes = {
      .name = "Uart2Process_TaskHandle",
      .stack_size = 512 * 4,
      .priority = (osPriority_t)osPriorityNormal1,
  };
  uart2ProcessTaskHandle =
      osThreadNew(uart2RxProcessTask, NULL, &Uart2ProcessTaskHandle_attributes);

  const osThreadAttr_t Uart3ProcessTaskHandle_attributes = {
      .name = "Uart3Process_TaskHandle",
      .stack_size = 512 * 4,
      .priority = (osPriority_t)osPriorityNormal1,
  };
  uart3ProcessTaskHandle =
      osThreadNew(uart3RxProcessTask, NULL, &Uart3ProcessTaskHandle_attributes);

 /* const osThreadAttr_t UsbcdcProcessTaskHandle_attributes = {
      .name = "UsbcdcProcess_TaskHandle",
      .stack_size = 512 * 4,
      .priority = (osPriority_t)osPriorityNormal1,
  };
  usbcdcProcessTaskHandle =
      osThreadNew(usbCdcProcessTask, NULL, &UsbcdcProcessTaskHandle_attributes);
*/
  const osThreadAttr_t tail_claw_TaskHandle_attributes = {
      .name = "tail_claw_TaskHandle",
      .stack_size = 128 * 4,
      .priority = (osPriority_t)osPriorityNormal1,
  };
  tail_claw_TaskHandle =
      osThreadNew(tail_claw_task, NULL, &tail_claw_TaskHandle_attributes);

      
//用于定位
  const osThreadAttr_t NavControlTaskHandle_attributes = {
      .name = "NavControl_TaskHandle",
      .stack_size = 512 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  NavControlTaskHandle =
      osThreadNew(NavControlTask, NULL, &NavControlTaskHandle_attributes);
//用于抬升
  const osThreadAttr_t LiftTaskHandle_attributes = {
      .name = "Lift_TaskHandle",
      .stack_size = 512 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  LiftTaskHandle =
      osThreadNew(liftTask, NULL, &LiftTaskHandle_attributes);

//用于上位机和下位机通讯的协议解析和封装
  const osThreadAttr_t PcComTaskHandle_attributes = {
      .name = "PcCom_TaskHandle",
      .stack_size = 512 * 4,
      .priority = (osPriority_t)osPriorityNormal,
  };
  PcComTaskHandle =
      osThreadNew(PcComTask, NULL, &PcComTaskHandle_attributes);
}
