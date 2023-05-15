#include <FreeRTOS_SAMD21.h>
#include "Sampling.h"
#include "KalmanFilter.h"
#include "Communication.h"

//**************************************************************************
// Type Defines and Constants
//**************************************************************************

//**************************************************************************
// global variables
//**************************************************************************
size_t samplingPeriod { 50 };
void SamplingTask( void *pvParameters );
void EstimationTask( void *pvParameters );
void TransmitterTask( void *pvParameters );
TaskHandle_t Handle_SamplingTask;
TaskHandle_t Handle_EstimationTask;
TaskHandle_t Handle_TransmitterTask;
Sampling _Sampling { samplingPeriod };
KalmanFilter _KalmanFilter { samplingPeriod };
Communication _Communication;

void setup() 
{
  Serial.begin(115200);
  while (!Serial);

  _Sampling.Begin();
  _Communication.Begin();

  // Create the threads that will be managed by the rtos
  // Sets the stack size and priority of each task
  // Also initializes a handler pointer to each task, which are important to communicate with and retrieve info from tasks
  xTaskCreate(SamplingTask,     "Sampling",       256, NULL, 3, &Handle_SamplingTask);
  xTaskCreate(EstimationTask,   "Estimation",     256, NULL, 2, &Handle_EstimationTask);
  xTaskCreate(TransmitterTask,  "Transmission",   256, NULL, 1, &Handle_TransmitterTask);

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();
}

void loop() 
{

}

//**************************************************************************
// Tasks
//**************************************************************************
void SamplingTask( void *pvParameters ) 
{ 
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = samplingPeriod;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    _Sampling.Sample();
  }
}

void EstimationTask( void *pvParameters ) 
{
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = samplingPeriod;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    _KalmanFilter.Estimate(_Sampling.GetAcceleration(), _Sampling.GetDistance());
  }
}

void TransmitterTask(void *pvParameters)
{   
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = samplingPeriod;
  xLastWakeTime = xTaskGetTickCount();
  while(1)
  {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    _Communication.Transmit(_KalmanFilter.GetEstimate());
  }
}