#include <Arduino.h>
#include "CommandProcessor.h"
#include "neopixel_motorpin.h"
#include "ble_commands.h"
// #include "obstacle.h"

extern BleCommands BLECmd;

const char *words[] = {
    "forward",
    "backward",
    "left",
    "right",
    "_nonsense",
};

void commandQueueProcessorTask(void *param)
{
    CommandProcessor *commandProcessor = (CommandProcessor *)param;
    while (true)
    {
        uint16_t commandIndex = 0;
        if (xQueueReceive(commandProcessor->m_command_queue_handle, &commandIndex, portMAX_DELAY) == pdTRUE)
        {
            commandProcessor->processCommand(commandIndex);
        }
    }
}

// This function is no longer needed here as motor control is commented out
// but left for context if you uncomment motor logic.
int calcDuty(int ms)
{
    // 50Hz = 20ms period
    return (65536 * ms) / 20000;
}

const int leftForward = 1600;
const int leftBackward = 1400;
const int leftStop = 1500;
const int rightBackward = 1600;
const int rightForward = 1445;
const int rightStop = 1500;

void CommandProcessor::processCommand(uint16_t commandIndex)
{
    switch (commandIndex)
    {
    case 0: // forward
        if(flag){
            Serial.println("Forward");
            BLECmd.notify("FORWARD");
            pinMode(4, OUTPUT);
            pinMode(5, INPUT);
            digitalWrite(4, LOW);
            delayMicroseconds(2);
            digitalWrite(4, HIGH);
            delayMicroseconds(10);
            digitalWrite(4, LOW);
            long duration = pulseIn(5, HIGH, 20000);
            float d = 0;
            if(duration == 0){
                d = -1;
            }
            else{
                d = duration * 0.034 / 2.0;
            }
            Serial.print("Distance: ");
            Serial.print(d, 2);
            Serial.println("cm");
            if(d > 0 && d < 6.0){
                Serial.println("Obstacle detected! Stopping.");
                BLECmd.notify("OBSTACLE");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
            }
            else{
                digitalWrite(DRV_EEP, HIGH);
                BLECmd.notify("NO_OBSTACLE");
            }
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            for (int i = 0; i < 47; i++) {
                digitalWrite(4, LOW);
                delayMicroseconds(2);
                digitalWrite(4, HIGH);
                delayMicroseconds(10);
                digitalWrite(4, LOW);
                long duration = pulseIn(5, HIGH, 20000);
                float d = 0;
                if(duration == 0){
                    d = -1;
                }
                else{
                    d = duration * 0.034 / 2.0;
                }
                if(d > 0 && d < 6.0){
                    Serial.println("Obstacle detected! Stopping.");
                    BLECmd.notify("OBSTACLE");
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
        break;
    case 1: // backward
        if(flag){
            digitalWrite(DRV_EEP, HIGH);
            Serial.println("Backward");
            BLECmd.notify("BACKWARD");
            BLECmd.notify("NO_OBSTACLE");
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        break;
    case 2: // left
        if(flag){
            digitalWrite(DRV_EEP, HIGH);
            Serial.println("Left");
            BLECmd.notify("LEFT");
            BLECmd.notify("NO_OBSTACLE");
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        break;
    case 3: // right
        if(flag){
            digitalWrite(DRV_EEP, HIGH);
            Serial.println("Right");
            BLECmd.notify("RIGHT");
            BLECmd.notify("NO_OBSTACLE");
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
        break;
    }
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    BLECmd.notify("IDLE");
    digitalWrite(DRV_EEP, LOW);
}

CommandProcessor::CommandProcessor()
{
    // allow up to 5 commands to be in flight at once
    m_command_queue_handle = xQueueCreate(5, sizeof(uint16_t));
    if (!m_command_queue_handle)
    {
        Serial.println("Failed to create command queue");
    }
    // kick off the command processor task
    TaskHandle_t command_queue_task_handle;
    xTaskCreate(commandQueueProcessorTask, "Command Queue Processor", 4096, this, 1, &command_queue_task_handle);
}

void CommandProcessor::queueCommand(uint16_t commandIndex, float best_score)
{
    if (commandIndex != 5 && commandIndex != -1)
    {
        Serial.printf("***** %ld Detected command %s(%f)\n", millis(), words[commandIndex], best_score);

        if (xQueueSendToBack(m_command_queue_handle, &commandIndex, 0) != pdTRUE)
        {
            Serial.println("No more space for command");
        }
    }
}
