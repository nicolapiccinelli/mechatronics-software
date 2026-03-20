// system
#include <algorithm>
#include <array>
#include <cmath>
#include <map>

// firewire
#include "SimulationPortBackend.h"

class DefaultBackend : public sim::Backend
{
public:
    struct AxisDynamicsParams
    {
        // 2.0e-5 kg m^2 (approx Maxon RE40 rotor + gearhead)
        double motor_inertia = 0.001;
        // 1.0e-4 Nm/(rad/s)
        double viscous_damping = 0.1;
        // let's use multi-step integration for better stability
        unsigned int integration_steps = 5;
    };

    using BoardDynamicsParams = std::array<AxisDynamicsParams, 4>;

    const char *Name() const override
    {
        return "default";
    }

    void Step(SimulationPort &, sim::StateMap &boardStates, double dt) override
    {
        for (auto &kv : boardStates)
        {
            UpdateAxisDynamics(kv.second, mDynamicsParams[kv.first], dt);
        }
    }

private:
    static void UpdateAxisDynamics(sim::BoardState &state, const BoardDynamicsParams &params, double dt)
    {
        for (int i = 0; i < 4; ++i)
        {
            auto &ax = state.Axes[i];
            const AxisDynamicsParams &axisParams = params[i];

            const unsigned int steps = std::max(1u, axisParams.integration_steps);
            const double h = dt / static_cast<double>(steps);

            auto accel = [&](double pos, double vel)
            {
                return (ax.CommandTorque - axisParams.viscous_damping * vel) / axisParams.motor_inertia;
            };

            for (unsigned int s = 0; s < steps; ++s)
            {
                double k1_pos = ax.SimVelocity;
                double k1_vel = accel(ax.SimPosition, ax.SimVelocity);

                double k2_pos = ax.SimVelocity + 0.5 * h * k1_vel;
                double k2_vel = accel(ax.SimPosition + 0.5 * h * k1_pos,
                                      ax.SimVelocity + 0.5 * h * k1_vel);

                double k3_pos = ax.SimVelocity + 0.5 * h * k2_vel;
                double k3_vel = accel(ax.SimPosition + 0.5 * h * k2_pos,
                                      ax.SimVelocity + 0.5 * h * k2_vel);

                double k4_pos = ax.SimVelocity + h * k3_vel;
                double k4_vel = accel(ax.SimPosition + h * k3_pos,
                                      ax.SimVelocity + h * k3_vel);

                ax.SimPosition += (h / 6.0) * (k1_pos + 2.0 * k2_pos + 2.0 * k3_pos + k4_pos);
                ax.SimVelocity += (h / 6.0) * (k1_vel + 2.0 * k2_vel + 2.0 * k3_vel + k4_vel);
            }
        }
    }

    std::map<nodeid_t, BoardDynamicsParams> mDynamicsParams;
};

std::unique_ptr<sim::Backend> sim::MakeDefaultBackend()
{
    return std::unique_ptr<Backend>(new DefaultBackend());
}