// system
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

// firewire
#include "SimulationPortBackend.h"

class SimulationPortSocketBackend : public sim::Backend
{
public:
    struct FrameHeader
    {
        uint32_t Magic;
        uint32_t Version;
        uint32_t PayloadSize;
        uint32_t NodeCount;
        uint64_t Sequence;
        double Dt;
    };

    struct SocketAxisState
    {
        double CommandTorque;
        double SimPosition;
        double SimVelocity;
        double DesiredTs;
    };

    static constexpr uint32_t SocketFrameMagic = 0x53494D50u;
    static constexpr uint32_t SocketFrameVersion = 1u;
    static constexpr const char *DefaultSocketPath = "/tmp/amp1394-sim.sock";

    SimulationPortSocketBackend(const std::string &socketPath,
                                std::chrono::milliseconds replyTimeout,
                                std::chrono::milliseconds connectRetry)
        : mSocketPath(socketPath),
          mReplyTimeout(replyTimeout),
          mConnectRetry(connectRetry)
    {
    }

    ~SimulationPortSocketBackend() override
    {
        ShutdownImpl();
    }

    const char *Name() const override
    {
        return "unix-socket";
    }

    void Initialize(SimulationPort &) override
    {
    }

    void Shutdown(SimulationPort &) override
    {
        ShutdownImpl();
    }

    void Step(SimulationPort &, sim::StateMap &boardStates, double dt) override
    {
        std::lock_guard<std::mutex> lock(mMutex);

        if (!EnsureConnected())
        {
            return;
        }

        const std::vector<uint8_t> frame = SerializeStateMap(boardStates, ++mSequence, dt);
        if (!SendAll(mSocketFd, frame.data(), frame.size()))
        {
            CloseSocket(mSocketFd);
            return;
        }

        sim::StateMap inboundStates;
        if (!ReceiveFrame(mSocketFd, inboundStates))
        {
            CloseSocket(mSocketFd);
            return;
        }

        ApplyBackendState(inboundStates, boardStates);
    }

    static int ReadEnvInt(const char *name, int defaultValue)
    {
        const char *value = std::getenv(name);
        if (!value || !value[0])
        {
            return defaultValue;
        }

        char *end = nullptr;
        const long parsed = std::strtol(value, &end, 10);
        if (!end || *end != '\0' || parsed <= 0)
        {
            return defaultValue;
        }

        return static_cast<int>(parsed);
    }

    static bool IsRequestedFromEnvironment()
    {
        const char *backend = std::getenv("AMP1394_SIM_BACKEND");

        if (!backend || !backend[0])
        {
            return false;
        }

        return (std::strcmp(backend, "socket") == 0) ||
               (std::strcmp(backend, "unix") == 0) ||
               (std::strcmp(backend, "unix-socket") == 0);
    }

private:
    template <class TValue>
    static void AppendValue(std::vector<uint8_t> &buffer, const TValue &value)
    {
        static_assert(std::is_trivially_copyable<TValue>::value, "wire value must be trivially copyable");

        const size_t offset = buffer.size();
        buffer.resize(offset + sizeof(TValue));
        std::memcpy(buffer.data() + offset, &value, sizeof(TValue));
    }

    template <class TValue>
    static bool ReadValue(const std::vector<uint8_t> &buffer, size_t &offset, TValue &value)
    {
        static_assert(std::is_trivially_copyable<TValue>::value, "wire value must be trivially copyable");

        if (offset + sizeof(TValue) > buffer.size())
        {
            return false;
        }

        std::memcpy(&value, buffer.data() + offset, sizeof(TValue));
        offset += sizeof(TValue);
        return true;
    }

    static void SerializeAxisState(std::vector<uint8_t> &buffer, const sim::AxisState &axis, double desiredTs)
    {
        const SocketAxisState socketAxis{
            axis.CommandTorque,
            axis.SimPosition,
            axis.SimVelocity,
            desiredTs
        };
        AppendValue(buffer, socketAxis);
    }

    static bool DeserializeAxisState(const std::vector<uint8_t> &buffer, size_t &offset, sim::AxisState &axis)
    {
        SocketAxisState socketAxis{};
        if (!ReadValue(buffer, offset, socketAxis))
        {
            return false;
        }

        axis.CommandTorque = socketAxis.CommandTorque;
        axis.SimPosition = socketAxis.SimPosition;
        axis.SimVelocity = socketAxis.SimVelocity;
        return true;
    }

    static void SerializeBoardState(std::vector<uint8_t> &buffer, const sim::BoardState &state, double desiredTs)
    {
        AppendValue(buffer, state.DigitalOutput);

        for (int axisIndex = 0; axisIndex < 4; ++axisIndex)
        {
            SerializeAxisState(buffer, state.Axes[axisIndex], desiredTs);
        }
    }

    static bool DeserializeBoardState(const std::vector<uint8_t> &buffer, size_t &offset, sim::BoardState &state)
    {
        if (!ReadValue(buffer, offset, state.DigitalOutput))
        {
            return false;
        }

        for (int axisIndex = 0; axisIndex < 4; ++axisIndex)
        {
            if (!DeserializeAxisState(buffer, offset, state.Axes[axisIndex]))
            {
                return false;
            }
        }

        return true;
    }

    static void ApplyBackendState(const sim::StateMap &source, sim::StateMap &target)
    {
        for (const auto &entry : source)
        {
            const auto found = target.find(entry.first);
            if (found == target.end())
            {
                continue;
            }

            sim::BoardState &targetBoard = found->second;
            const sim::BoardState &sourceBoard = entry.second;
            for (int axisIndex = 0; axisIndex < 4; ++axisIndex)
            {
                targetBoard.Axes[axisIndex].SimPosition = sourceBoard.Axes[axisIndex].SimPosition;
                targetBoard.Axes[axisIndex].SimVelocity = sourceBoard.Axes[axisIndex].SimVelocity;
            }
        }
    }

    static std::vector<uint8_t> SerializeStateMap(const sim::StateMap &boardStates,
                                                  uint64_t sequence,
                                                  double dt)
    {
        std::vector<uint8_t> payload;
        payload.reserve(boardStates.size() * 256);

        for (const auto &entry : boardStates)
        {
            const uint32_t node = static_cast<uint32_t>(entry.first);
            AppendValue(payload, node);
            SerializeBoardState(payload, entry.second, dt);
        }

        FrameHeader header;
        header.Magic = SocketFrameMagic;
        header.Version = SocketFrameVersion;
        header.PayloadSize = static_cast<uint32_t>(payload.size());
        header.NodeCount = static_cast<uint32_t>(boardStates.size());
        header.Sequence = sequence;
        header.Dt = dt;

        std::vector<uint8_t> frame;
        frame.reserve(sizeof(FrameHeader) + payload.size());
        AppendValue(frame, header);
        frame.insert(frame.end(), payload.begin(), payload.end());
        return frame;
    }

    static bool DeserializeStateMap(const std::vector<uint8_t> &payload,
                                    uint32_t nodeCount,
                                    sim::StateMap &boardStates)
    {
        sim::StateMap decoded;
        size_t offset = 0;

        for (uint32_t index = 0; index < nodeCount; ++index)
        {
            uint32_t node = 0;
            sim::BoardState state;

            if (!ReadValue(payload, offset, node) || !DeserializeBoardState(payload, offset, state))
            {
                return false;
            }

            decoded[static_cast<nodeid_t>(node)] = state;
        }

        if (offset != payload.size())
        {
            return false;
        }

        boardStates = std::move(decoded);
        return true;
    }

    static bool SendAll(int fd, const uint8_t *data, size_t size)
    {
        size_t totalSent = 0;
        while (totalSent < size)
        {
            const ssize_t sent = send(fd,
                                      data + totalSent,
                                      size - totalSent,
                                      MSG_NOSIGNAL);
            if (sent <= 0)
            {
                return false;
            }

            totalSent += static_cast<size_t>(sent);
        }

        return true;
    }

    static bool RecvAll(int fd, uint8_t *data, size_t size)
    {
        size_t totalRead = 0;
        while (totalRead < size)
        {
            const ssize_t received = recv(fd, data + totalRead, size - totalRead, 0);
            if (received <= 0)
            {
                return false;
            }

            totalRead += static_cast<size_t>(received);
        }

        return true;
    }

    void ShutdownImpl()
    {
        std::lock_guard<std::mutex> lock(mMutex);
        CloseSocket(mSocketFd);
    }

    bool EnsureConnected()
    {
        if (mSocketFd >= 0)
        {
            return true;
        }

        const auto now = std::chrono::steady_clock::now();
        if ((mLastConnectAttempt.time_since_epoch().count() != 0) &&
            ((now - mLastConnectAttempt) < mConnectRetry))
        {
            return false;
        }

        mLastConnectAttempt = now;
        mSocketFd = ConnectSocket();
        return (mSocketFd >= 0);
    }

    int ConnectSocket() const
    {
        int fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd < 0)
        {
            return -1;
        }

        timeval timeout;
        timeout.tv_sec = static_cast<time_t>(mReplyTimeout.count() / 1000);
        timeout.tv_usec = static_cast<suseconds_t>((mReplyTimeout.count() % 1000) * 1000);

        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        sockaddr_un address;
        std::memset(&address, 0, sizeof(address));
        address.sun_family = AF_UNIX;
        std::snprintf(address.sun_path, sizeof(address.sun_path), "%s", mSocketPath.c_str());

        if (connect(fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0)
        {
            close(fd);
            return -1;
        }

        return fd;
    }

    bool ReceiveFrame(int fd, sim::StateMap &boardStates) const
    {
        FrameHeader header;
        if (!RecvAll(fd, reinterpret_cast<uint8_t *>(&header), sizeof(header)))
        {
            return false;
        }

        if ((header.Magic != SocketFrameMagic) || (header.Version != SocketFrameVersion))
        {
            return false;
        }

        std::vector<uint8_t> payload(header.PayloadSize, 0);
        if (!payload.empty() && !RecvAll(fd, payload.data(), payload.size()))
        {
            return false;
        }

        return DeserializeStateMap(payload, header.NodeCount, boardStates);
    }

    static void CloseSocket(int &fd)
    {
        if (fd >= 0)
        {
            close(fd);
            fd = -1;
        }
    }

    std::string mSocketPath;
    std::chrono::milliseconds mReplyTimeout;
    std::chrono::milliseconds mConnectRetry;

    mutable std::mutex mMutex;
    int mSocketFd = -1;
    std::chrono::steady_clock::time_point mLastConnectAttempt{};
    uint64_t mSequence = 0;
};

std::unique_ptr<sim::Backend> sim::MakeSocketBackend(const std::string &socketPath,
                                                     int replyTimeoutMs,
                                                     int connectRetryMs)
{
    return std::unique_ptr<Backend>(
        new SimulationPortSocketBackend(socketPath,
                                        std::chrono::milliseconds(replyTimeoutMs),
                                        std::chrono::milliseconds(connectRetryMs)));
}

std::unique_ptr<sim::Backend> sim::MakeSocketBackendFromEnvironment()
{
    if (!SimulationPortSocketBackend::IsRequestedFromEnvironment())
    {
        return std::unique_ptr<Backend>();
    }

    const char *socketPath = std::getenv("AMP1394_SIM_SOCKET_PATH");
    const std::string path = (socketPath && socketPath[0]) ? socketPath : SimulationPortSocketBackend::DefaultSocketPath;

    return MakeSocketBackend(path,
                             SimulationPortSocketBackend::ReadEnvInt("AMP1394_SIM_SOCKET_TIMEOUT_MS", 50),
                             SimulationPortSocketBackend::ReadEnvInt("AMP1394_SIM_SOCKET_CONNECT_RETRY_MS", 500));
}