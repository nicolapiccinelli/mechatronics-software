#pragma once

#include <vector>
#include <queue>
#include <map>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <ostream>

#include "SimulationPortTypes.h"
#include "SimulationPortBackend.h"
#include "BoardIO.h"
#include "BasePort.h"

class SimulationPort : public BasePort
{
public:
    using AxisState = sim::AxisState;
    using BoardState = sim::BoardState;
    using StateMap = sim::StateMap;
    using Backend = sim::Backend;

    SimulationPort(int portNum, std::ostream &ostr = std::cerr);
    ~SimulationPort();

    PortType GetPortType(void) const { return PORT_SIMULATION; }
    int NumberOfUsers(void);
    bool IsOK(void) { return true; }
    unsigned int GetBusGeneration(void) const;
    void UpdateBusGeneration(unsigned int gen) {}

    unsigned int GetPrefixOffset(MsgType msg) const { return 0; }
    unsigned int GetWritePostfixSize(void) const { return 0; }
    unsigned int GetReadPostfixSize(void) const { return 0; }
    unsigned int GetWriteQuadAlign(void) const { return 0; }
    unsigned int GetReadQuadAlign(void) const { return 0; }
    unsigned int GetMaxReadDataSize(void) const { return 2048; }
    unsigned int GetMaxWriteDataSize(void) const { return 2048; }

    bool WriteBroadcastOutput(quadlet_t *buffer, unsigned int size);
    bool WriteBroadcastReadRequest(unsigned int seq);
    void WaitBroadcastRead(void);

    void PromDelay(void) const {}

    // Adds board(s)
    bool AddBoard(BoardIO *board);
    // Removes board
    bool RemoveBoard(unsigned char boardId);

protected:
    bool Init(void);
    void Cleanup(void);
    nodeid_t InitNodes(void);

    bool ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char flags = 0);
    bool WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char flags = 0);
    bool WriteBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes, unsigned char flags = 0);
    bool ReadBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *rdata, unsigned int nbytes, unsigned char flags = 0);

private:
    class WriteRequest
    {
    public:
        nodeid_t node;
        nodeaddr_t addr;
        quadlet_t data;
        unsigned char flags;

        WriteRequest(nodeid_t n, nodeaddr_t a, quadlet_t d, unsigned char f)
            : node(n), addr(a), data(d), flags(f) {}
    };

    const uint32_t PERIOD_MASK = 0x03FFFFFFu;
    const uint32_t DIR_BIT = 0x40000000u;
    const uint32_t OVF_BIT = 0x80000000u;

    const uint32_t DOUT_MASK = 0x0Fu;
    const uint32_t HOME_SHIFT = 0;
    const uint32_t POS_LIMIT_SHIFT = 4;
    const uint32_t NEG_LIMIT_SHIFT = 8;
    const uint32_t DOUT_FB_SHIFT = 12;
    const uint32_t ENC_INDEX_SHIFT = 16;
    const uint32_t ENC_B_SHIFT = 20;
    const uint32_t ENC_A_SHIFT = 24;

    const double DAC_BITS_PER_AMP = 5242.88;
    const uint32_t AMP_STATUS_BIT = 0x20000000u;
    const uint32_t ENC_MIDRANGE = 0x00800000u;
    const int32_t ENC_MASK_24 = 0x00FFFFFF;
    const int32_t ENC_MODULUS_24 = 0x01000000;

    const char kSimQLASN[12] = "QLA 1234-56";
    const std::string SimFPGASerialString = "FPGA 1234-56";

    // i want a queue to store request for every board id or node id separately
    std::map<nodeid_t, std::queue<WriteRequest>> WriteRequestQueues;
    StateMap mBoardStates;
    
    // Simulated FPGA PROM state
    uint32_t SimPromCurrentAddr;

    // Dynamics simulation thread state
    std::thread dynamicsThread;
    std::mutex stateMutex;
    std::atomic<bool> dynamicsRun{false};
    bool backendActive = false;
    std::unique_ptr<Backend> backend;

    // 1 ms default timestep for dynamics simulation
    // this should be enough for simulating the dynamics of the arm
    double dynamics_dt_sec = 0.001;

    // Helper to fetch a simulated PROM byte at absolute 24-bit address
    uint8_t GetSimPromByte(uint32_t abs_addr) const;
    quadlet_t ProcessPROM(nodeid_t node);
    
    int32_t WrapEncoder24(int32_t value);
    int32_t ShortestEncoderDelta24(int32_t from, int32_t to);
    uint8_t PackAxisTemperature(double temperatureC);

    // Prepare the BoardState for the backend by applying any necessary transformations
    void PrepareBackendState(BoardState &state);

    // Update the BoardState based on the backend's output, applying any necessary transformations
    void UpdateEmulatedFeedback(BoardState &state, double dt);

    void StartDynamicsThread();
    void StopDynamicsThread();
    void DynamicsThreadFunc();
};