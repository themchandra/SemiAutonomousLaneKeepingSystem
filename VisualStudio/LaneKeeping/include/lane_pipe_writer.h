#pragma once

struct LaneInput {
    bool valid;
    float steering_error;
};

bool initializeLanePipeWriter();
bool sendLaneInput(bool valid, float steering_error);