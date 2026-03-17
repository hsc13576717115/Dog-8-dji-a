#include "process_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern uint8_t flag_Standup;
extern osThreadId_t ProcessHandle;

void Process_Task(void *argument)
{
    while (1)
    {
        if (flag_Standup)
        {
            Change_NowState(STOP);
            osThreadSuspend(ProcessHandle); //暂停不运行了 直至重启
        }
    }
}
