#pragma once

#include <map>

#include "BoardIO.h"

namespace sim {

struct AxisConfig
{
    // this must coincide with the NmToAmps scale in the XML config
    // where you will find the scale computed as 1/torque_constant
    double torque_constant = 0.1;
    // this must coincide with the BitsToPosSI scale in the XML config
    // where you will find the scale computed as 360/counts_per_turn
    double counts_per_turn = 100000.0;
    // Thermal model parameters [C]
    double ambient_temp_c = 25.0;
    // Reduce heating effect (less aggressive temperature rise) [C/s per A^2]
    double thermal_heating_coeff = 0.05;
    // Increase cooling (stronger dissipation) [1/s]
    double thermal_cooling_coeff = 0.02;
};

struct AxisState
{
    // Command torque seen by the backend after hardware-level DAC/amp emulation.
    double CommandTorque = 0.0;

    // ENC_MIDRANGE
    int32_t EncoderPos = 0x800000;

    double EncoderVel = 0.0;
    double SimPosition = 0.0;
    double SimVelocity = 0.0;

    int32_t EncoderQtr1 = 0;
    int32_t EncoderQtr5 = 0;
    int32_t EncoderRun = 0;

    // ENC_MIDRANGE
    int32_t EncoderPreload = 0x800000;
    int32_t EncoderOffset = 0;

    // zero current
    uint32_t MotorCurrent = 32768;

    // Default to OFF
    uint32_t MotorStatus = 0x00000000;

    // Simple temperature state [C]
    double TemperatureC = 25.0;

    // Per-axis parameters shared by all backends.
    AxisConfig config;
};

struct BoardState
{
    uint32_t Timestamp = 0;
    uint32_t Status = 0;
    uint32_t Temperature = 0;
    uint8_t DigitalOutput = 0;

    AxisState Axes[4];

    // Simulation behavior: require a power-off after startup before granting amp enable
    // When true, motors won't transition to STATUS until a power-off event is seen once.
    bool RequirePowerCycleLatch = true;
    bool HasSeenPowerOff = false;
};

using StateMap = std::map<nodeid_t, BoardState>;

} // namespace sim