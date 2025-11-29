#ifndef _COMMAND_PROCESSOR_H_
#define _COMMAND_PROCESSOR_H_

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "neopixel_motorpin.h"

// Forward declare the task function
void commandQueueProcessorTask(void *param);

class CommandProcessor
{
private:
    QueueHandle_t m_command_queue_handle;

public:
    CommandProcessor();
    void queueCommand(uint16_t commandIndex, float best_score);
    void processCommand(uint16_t commandIndex);

    // Grant our task function access to private members
    friend void commandQueueProcessorTask(void *param);
};

extern int flag;

#endif
