#ifndef CONTROLLER_ENUMCLASS_H
#define CONTROLLER_ENUMCLASS_H

// Command values carried on controller/msg/Inputs.command - see
// that message file for the authoritative list.
enum class ControlCommand
{
    NONE = 0,
    STAND = 1,
    SIT = 2,
    ESTOP = 9
};

enum class FSMStateName
{
    INVALID,
    PASSIVE,
    STAND,
    SIT
};

enum class FSMMode
{
    NORMAL,
    CHANGE
};

#endif //CONTROLLER_ENUMCLASS_H
