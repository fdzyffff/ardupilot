#include "Copter.h"

#define USERENGINE_THR_POS0 1000
#define USERENGINE_THR_POS1 900
#define USERENGINE_THR_POS2 1950

void UserEngine::Init(AP_SerialManager::SerialProtocol in_protocol)
{
    if (_uart == nullptr) {
        _uart = new FD_UART(in_protocol);
    }
}

void UserEngine::update()
{
    tnow = millis();
    switch (thrPos) {
        default:
        case 0:
            _output = USERENGINE_THR_POS0; // normal low, 1000
            break;
        case 1:
            _output = USERENGINE_THR_POS1; // lowest, 900
            if (tnow - _last_ms > 2000) {
                thrPos = 2;
            }
            break;
        case 2:
            _output = USERENGINE_THR_POS2; // highest, 1950
            if (tnow - _last_ms > 2000) {
                thrPos = 0;
            }
            break;
    }
}

void UserEngine::Boost()
{
    thrPos = 1;
    _last_ms = millis();
}

bool UserEngine::IsState(EngineState in_state)
{
    return (_state == in_state);
}

int16_t UserEngine::get_output_min()
{
    return USERENGINE_THR_POS1;
}