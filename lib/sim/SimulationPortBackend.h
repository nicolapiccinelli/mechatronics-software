#pragma once

#include <memory>
#include <string>

#include "SimulationPortTypes.h"

class SimulationPort;

namespace sim {

class Backend
{
public:
    virtual ~Backend() = default;
    virtual const char *Name() const = 0;
    virtual void Initialize(::SimulationPort &port) {}
    virtual void Shutdown(::SimulationPort &port) {}
    virtual void Step(::SimulationPort &port, StateMap &boardStates, double dt) = 0;
};

std::unique_ptr<Backend> MakeDefaultBackend();
std::unique_ptr<Backend> MakeSocketBackend(const std::string &socketPath, int replyTimeoutMs = 50, int connectRetryMs = 500);

// Environment variables:
// AMP1394_SIM_BACKEND=socket
// AMP1394_SIM_SOCKET_PATH
// AMP1394_SIM_SOCKET_TIMEOUT_MS
// AMP1394_SIM_SOCKET_CONNECT_RETRY_MS
std::unique_ptr<Backend> MakeSocketBackendFromEnvironment();

} // namespace sim