#include <stdio.h>
#include <stdlib.h>
#include "speedcontroller.h"

void main(void)
{
    setup();
    INIT_controller();
    xTaskTorqueController
    xCreateTask(loop, "Loop", 1000, NULL, 1, NULL);
}