#pragma once
#include "../../common/LaneInfo.h"
bool initializeLanePipeWriter();

// sends the latest steering error result to the controller
// takes in a lane info struct which contains valid flag and steering error and returns true if sent succesfully
bool sendLaneInput(LaneInfo info);