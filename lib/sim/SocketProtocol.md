# Socket Backend Protocol

The optional socket backend uses an AF_UNIX stream socket.

Environment selection:

- `AMP1394_SIM_BACKEND=socket`
- `AMP1394_SIM_SOCKET_PATH=/tmp/amp1394-sim.sock`
- `AMP1394_SIM_SOCKET_TIMEOUT_MS=50`
- `AMP1394_SIM_SOCKET_CONNECT_RETRY_MS=500`

The socket backend is selected only when `AMP1394_SIM_BACKEND` explicitly requests it.
`AMP1394_SIM_SOCKET_PATH` only configures the socket path once the socket backend is selected.
Otherwise the simulation remains self-contained.

The simulation port acts as a client.
An external simulator process must create and listen on the Unix domain socket path.

The backend contract is physical, not register-level:

- SimulationPort computes commanded torque from DAC bits and amplifier state.
- SimulationPort regenerates encoder feedback, thermal state, and packed temperature registers locally.
- The backend only evolves physical state and receives command torques and digital outputs.

Reference test server:

- `python3 lib/sim/python/socket_backend_server.py --socket /tmp/amp1394-sim.sock --mode echo`
- `python3 lib/sim/python/socket_backend_server.py --socket /tmp/amp1394-sim.sock --mode integrate --verbose`

Protocol model:

1. The simulation port sends one frame per backend step.
2. The external simulator replies with one frame containing the updated board states.
3. The reply is applied immediately as the internal simulation state snapshot for the same backend step.

Frame header layout:

- `uint32_t Magic` = `0x53494D50` (`SIMP`)
- `uint32_t Version` = `1`
- `uint32_t PayloadSize`
- `uint32_t NodeCount`
- `uint64_t Sequence`
- `double Dt`

Payload layout:

For each node:

- `uint32_t node`
- `sim::BoardState`

`sim::BoardState` serialized fields:

- `uint8_t DigitalOutput`
- `sim::AxisState Axes[4]`

Socket axis payload fields:

- `double CommandTorque`
- `double SimPosition`
- `double SimVelocity`
- `double DesiredTs`

Notes:

- The wire format is binary and uses native host endianness and native `double` representation.
- This backend is intended for processes on the same machine or identical architectures.
- The reply updates only the physical feedback fields (`SimPosition`, `SimVelocity`).
- `DesiredTs` is the desired simulation time step requested by the simulation port for that axis update.
- Encoder counts, velocity registers, quarter-cycle tracking, thermal state, and packed temperature registers are generated locally by `SimulationPort` for all backends.