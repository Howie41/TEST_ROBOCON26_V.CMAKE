
#include "mc_mission.hpp"
#include "blackboard.hpp"
#include "bt_core.hpp"
#include "driver.hpp"
#include"cmsis_os.h"
//在main中统一调度每个区域的任务，main_mission.cpp中是主任务，负责调度每个区域的任务
//到时候黑板上会有每个区域的状态，主任务根据这些状态来决定执行哪个区域的任务，
//就是有一个主任务，负责调度每个区域任务，比如是在武馆就调度武馆的任务，在梅林就调度梅林的任务等等
void Task_R2Mission(void* argument)
{
    R2MCMission r2_mc_mission(
        chassis,
        gripper,
        vision,
        comm
    );

    while (1) {
        BB.now_ms = HAL_GetTick();

        r2_mc_mission.tick();

        osDelay(10);
    }
}