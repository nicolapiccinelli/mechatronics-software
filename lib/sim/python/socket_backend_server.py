#!/usr/bin/env python3

import argparse
import signal
import socket
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable


MAGIC = 0x53494D50
VERSION = 1
HEADER_STRUCT = struct.Struct("=IIIIQd")


@dataclass
class AxisState:
    command_torque: float
    sim_position: float
    sim_velocity: float
    desired_ts: float


@dataclass
class BoardState:
    digital_output: int
    axes: list[AxisState] = field(default_factory=list)


AXIS_STRUCT = struct.Struct("=dddd")
BOARD_PREFIX_STRUCT = struct.Struct("=B")
NODE_STRUCT = struct.Struct("=I")

DEFAULT_MOTOR_INERTIA = 0.001
DEFAULT_VISCOUS_DAMPING = 0.1
DEFAULT_INTEGRATION_STEPS = 5


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("socket closed while reading frame")
        chunks.extend(chunk)
    return bytes(chunks)


def recv_frame(sock: socket.socket) -> tuple[int, float, Dict[int, BoardState]]:
    header = recv_exact(sock, HEADER_STRUCT.size)
    magic, version, payload_size, node_count, sequence, dt = HEADER_STRUCT.unpack(header)
    if magic != MAGIC:
        raise ValueError(f"unexpected magic 0x{magic:08x}")
    if version != VERSION:
        raise ValueError(f"unexpected version {version}")

    payload = memoryview(recv_exact(sock, payload_size))
    offset = 0
    states: Dict[int, BoardState] = {}

    for _ in range(node_count):
        node, = NODE_STRUCT.unpack_from(payload, offset)
        offset += NODE_STRUCT.size
        board, offset = unpack_board_state(payload, offset)
        states[node] = board

    if offset != payload_size:
        raise ValueError("payload not fully consumed")

    return sequence, dt, states


def send_frame(sock: socket.socket, sequence: int, dt: float, states: Dict[int, BoardState]) -> None:
    payload = bytearray()
    for node, board in states.items():
        payload.extend(NODE_STRUCT.pack(node))
        payload.extend(pack_board_state(board))

    header = HEADER_STRUCT.pack(MAGIC, VERSION, len(payload), len(states), sequence, dt)
    sock.sendall(header)
    sock.sendall(payload)


def unpack_axis_state(payload: memoryview, offset: int) -> tuple[AxisState, int]:
    values = AXIS_STRUCT.unpack_from(payload, offset)
    offset += AXIS_STRUCT.size
    return AxisState(*values), offset


def pack_axis_state(axis: AxisState) -> bytes:
    return AXIS_STRUCT.pack(
        axis.command_torque,
        axis.sim_position,
        axis.sim_velocity,
        axis.desired_ts,
    )


def unpack_board_state(payload: memoryview, offset: int) -> tuple[BoardState, int]:
    digital_output, = BOARD_PREFIX_STRUCT.unpack_from(payload, offset)
    offset += BOARD_PREFIX_STRUCT.size

    axes = []
    for _ in range(4):
        axis, offset = unpack_axis_state(payload, offset)
        axes.append(axis)

    return BoardState(
        digital_output=digital_output,
        axes=axes,
    ), offset


def pack_board_state(board: BoardState) -> bytes:
    packed = bytearray(
        BOARD_PREFIX_STRUCT.pack(
            board.digital_output,
        )
    )
    for axis in board.axes:
        packed.extend(pack_axis_state(axis))
    return bytes(packed)


def echo_mode(states: Dict[int, BoardState], _: float) -> Dict[int, BoardState]:
    return states


def integrate_mode(states: Dict[int, BoardState], dt: float) -> Dict[int, BoardState]:
    for board in states.values():
        for axis in board.axes:
            step_dt = axis.desired_ts if axis.desired_ts > 0.0 else dt
            steps = DEFAULT_INTEGRATION_STEPS
            h = step_dt / float(steps)

            def accel(position: float, velocity: float) -> float:
                return (axis.command_torque - DEFAULT_VISCOUS_DAMPING * velocity) / DEFAULT_MOTOR_INERTIA

            for _ in range(steps):
                k1_pos = axis.sim_velocity
                k1_vel = accel(axis.sim_position, axis.sim_velocity)

                k2_pos = axis.sim_velocity + 0.5 * h * k1_vel
                k2_vel = accel(axis.sim_position + 0.5 * h * k1_pos,
                               axis.sim_velocity + 0.5 * h * k1_vel)

                k3_pos = axis.sim_velocity + 0.5 * h * k2_vel
                k3_vel = accel(axis.sim_position + 0.5 * h * k2_pos,
                               axis.sim_velocity + 0.5 * h * k2_vel)

                k4_pos = axis.sim_velocity + h * k3_vel
                k4_vel = accel(axis.sim_position + h * k3_pos,
                               axis.sim_velocity + h * k3_vel)

                axis.sim_position += (h / 6.0) * (k1_pos + 2.0 * k2_pos + 2.0 * k3_pos + k4_pos)
                axis.sim_velocity += (h / 6.0) * (k1_vel + 2.0 * k2_vel + 2.0 * k3_vel + k4_vel)

    return states


def print_summary(sequence: int, states: Dict[int, BoardState]) -> None:
    summary = []
    for node, board in states.items():
        torques = ", ".join(f"{axis.command_torque:.3f}" for axis in board.axes)
        positions = ", ".join(f"{axis.sim_position:.4f}" for axis in board.axes)
        summary.append(f"node={node} dout=0x{board.digital_output:02x} tau=[{torques}] q=[{positions}]")
    print(f"frame {sequence}: {'; '.join(summary)}")


def serve(socket_path: Path, mode: str, verbose: bool) -> int:
    if socket_path.exists():
        socket_path.unlink()

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(socket_path))
    server.listen(1)

    def cleanup(*_args: object) -> None:
        server.close()
        if socket_path.exists():
            socket_path.unlink()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    handler = echo_mode if mode == "echo" else integrate_mode

    print(f"listening on {socket_path} in {mode} mode")
    try:
        while True:
            conn, _ = server.accept()
            print("client connected")
            with conn:
                while True:
                    try:
                        sequence, dt, states = recv_frame(conn)
                    except ConnectionError:
                        print("client disconnected")
                        break

                    if verbose:
                        print_summary(sequence, states)

                    states = handler(states, dt)
                    send_frame(conn, sequence, dt, states)
    finally:
        server.close()
        if socket_path.exists():
            socket_path.unlink()


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test server for the Amp1394 Unix socket simulation backend")
    parser.add_argument("--socket", default="/tmp/amp1394-sim.sock", help="Unix socket path")
    parser.add_argument("--mode", choices=("echo", "integrate"), default="echo",
                        help="echo returns the received state unchanged; integrate applies a simple test dynamics")
    parser.add_argument("--verbose", action="store_true", help="print one summary line per received frame")
    return parser.parse_args(list(argv))


def main(argv: Iterable[str]) -> int:
    args = parse_args(argv)
    socket_path = Path(args.socket)
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    return serve(socket_path, args.mode, args.verbose)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))