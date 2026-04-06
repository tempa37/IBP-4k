#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, Sequence

try:
    from gs_usb.gs_usb import GsUsb
    from gs_usb.gs_usb_frame import GsUsbFrame
except ImportError:
    print("Error: gs_usb is not installed.")
    print("Install it with: pip install gs_usb")
    sys.exit(1)


FW_BLOCK_SIZE_PACKETS = 10
FW_PACKET_DATA_SIZE = 6
FW_BLOCK_SIZE_BYTES = FW_BLOCK_SIZE_PACKETS * FW_PACKET_DATA_SIZE

CAN_FLAG_SLAVE_OS = 0x4E1
CAN_SLAVE_OS = 0x4E2
CAN_ACK_BLOCK = 0x4E3
CAN_FLAG_MASTER_OS = 0x4E4
CAN_MASTER_OS = 0x4E5

CAN_CMD_GET_REGS = 0x40F
CAN_SLAVE_UPDATE_START = 0x4FA

CAN_BLOCK_BASE_IDS = {
    0: 0x400,
    1: 0x420,
    2: 0x440,
    3: 0x460,
}

MODBUS_REGISTER_COUNT = 0x27
MODBUS_REGISTER_BYTE_COUNT = MODBUS_REGISTER_COUNT * 2
CAN_PACKETS_PER_BLOCK = (MODBUS_REGISTER_BYTE_COUNT + 7) // 8

TIMEOUT_INIT = 10.0
TIMEOUT_ACK = 1.5
MAX_RETRIES = 3

DEFAULT_COLLECT_TIMEOUT = 2.0
DEFAULT_GAP_TIMEOUT = 0.40
DEFAULT_ATTEMPTS = 2


@dataclass(frozen=True)
class RegisterDef:
    index: int
    name: str
    signed: bool = False


REGISTER_LAYOUT = [
    RegisterDef(0, "ina_config"),
    RegisterDef(1, "voltage_mv"),
    RegisterDef(2, "current_ma", signed=True),
    RegisterDef(3, "power_mw"),
    RegisterDef(4, "bq_status_state_sys"),
    RegisterDef(5, "balancing_status"),
    RegisterDef(6, "cell_voltage_mv[0]"),
    RegisterDef(7, "cell_voltage_mv[1]"),
    RegisterDef(8, "cell_voltage_mv[2]"),
    RegisterDef(9, "cell_voltage_mv[3]"),
    RegisterDef(10, "cell_voltage_mv[4]"),
    RegisterDef(11, "cell_voltage_mv[5]"),
    RegisterDef(12, "cell_voltage_mv[6]"),
    RegisterDef(13, "cell_voltage_mv[7]"),
    RegisterDef(14, "cell_voltage_mv[8]"),
    RegisterDef(15, "cell_voltage_mv[9]"),
    RegisterDef(16, "pack_voltage_mv"),
    RegisterDef(17, "pack_current_ma", signed=True),
    RegisterDef(18, "ship_mode"),
    RegisterDef(19, "capacity_mah"),
    RegisterDef(20, "soc_percent"),
    RegisterDef(21, "calibration_active_flag"),
    RegisterDef(22, "error_flags"),
    RegisterDef(23, "firmware_version"),
    RegisterDef(24, "pack_current_raw_ma", signed=True),
    RegisterDef(25, "bq_curr_cal_x[0]", signed=True),
    RegisterDef(26, "bq_curr_cal_x[1]", signed=True),
    RegisterDef(27, "bq_curr_cal_y[0]", signed=True),
    RegisterDef(28, "bq_curr_cal_y[1]", signed=True),
    RegisterDef(29, "ina_curr_cal_x[0]", signed=True),
    RegisterDef(30, "ina_curr_cal_x[1]", signed=True),
    RegisterDef(31, "ina_curr_cal_y[0]", signed=True),
    RegisterDef(32, "ina_curr_cal_y[1]", signed=True),
    RegisterDef(33, "ina_volt_cal_x[0]"),
    RegisterDef(34, "ina_volt_cal_x[1]"),
    RegisterDef(35, "ina_volt_cal_y[0]"),
    RegisterDef(36, "ina_volt_cal_y[1]"),
    RegisterDef(37, "nominal_capacity_mah"),
    RegisterDef(38, "bq_coulomb_count_mah"),
]


@dataclass
class Frame:
    can_id: int
    data: bytes


@dataclass
class BlockCapture:
    block_index: int
    packets: Dict[int, bytes] = field(default_factory=dict)

    @property
    def base_id(self) -> int:
        return CAN_BLOCK_BASE_IDS[self.block_index]

    def missing_packets(self) -> list[int]:
        return [idx for idx in range(CAN_PACKETS_PER_BLOCK) if idx not in self.packets]

    def is_complete(self) -> bool:
        return not self.missing_packets()

    def assemble_payload(self) -> bytes:
        missing = self.missing_packets()
        if missing:
            missing_ids = ", ".join(f"0x{self.base_id + idx:03X}" for idx in missing)
            raise ValueError(f"missing packet IDs: {missing_ids}")

        payload = bytearray()
        for packet_index in range(CAN_PACKETS_PER_BLOCK):
            payload.extend(self.packets[packet_index])

        if len(payload) < MODBUS_REGISTER_BYTE_COUNT:
            raise ValueError(
                f"received only {len(payload)} payload bytes, expected {MODBUS_REGISTER_BYTE_COUNT}"
            )

        return bytes(payload[:MODBUS_REGISTER_BYTE_COUNT])


def to_signed16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def decode_registers(payload: bytes) -> list[int]:
    if len(payload) < MODBUS_REGISTER_BYTE_COUNT:
        raise ValueError(
            f"payload is too short: {len(payload)} bytes, expected {MODBUS_REGISTER_BYTE_COUNT}"
        )

    registers = []
    for offset in range(0, MODBUS_REGISTER_BYTE_COUNT, 2):
        registers.append(struct.unpack(">H", payload[offset : offset + 2])[0])
    return registers


def resolve_block_packet(can_id: int) -> tuple[Optional[int], Optional[int]]:
    for block_index, base_id in CAN_BLOCK_BASE_IDS.items():
        packet_index = can_id - base_id
        if 0 <= packet_index < CAN_PACKETS_PER_BLOCK:
            return block_index, packet_index
    return None, None


class CandleLightCanClient:
    def __init__(self, bitrate: int = 125000, verbose: bool = False):
        self.bitrate = bitrate
        self.verbose = verbose
        self.dev = None

    def connect(self) -> bool:
        try:
            print(f"Searching CandleLight USB (bitrate={self.bitrate})...")
            devices = GsUsb.scan()
            if not devices:
                print("No CandleLight/gs_usb device found.")
                return False

            self.dev = devices[0]
            print(f"Using device: {self.dev}")

            if not self.dev.set_bitrate(self.bitrate):
                print("Failed to set CAN bitrate.")
                return False

            self.dev.start()
            print("CAN interface is up.")
            return True
        except Exception as exc:
            print(f"Connect failed: {exc}")
            return False

    def disconnect(self) -> None:
        if self.dev is not None:
            try:
                self.dev.stop()
            except Exception:
                pass
        print("Disconnected.")

    def _log_frame(self, direction: str, can_id: int, data: bytes) -> None:
        if not self.verbose:
            return
        hex_data = " ".join(f"{byte:02X}" for byte in data)
        print(f"{direction} ID=0x{can_id:03X} DLC={len(data)} DATA=[{hex_data}]")

    def send_frame(self, can_id: int, data: bytes = b"") -> bool:
        if self.dev is None:
            print("CAN device is not connected.")
            return False

        try:
            payload = bytes(data)
            self._log_frame("TX", can_id, payload)
            frame = GsUsbFrame(can_id=can_id, data=payload)
            return bool(self.dev.send(frame))
        except Exception as exc:
            print(f"TX failed for 0x{can_id:03X}: {exc}")
            return False

    def read_frame(self, timeout_ms: int = 100) -> Optional[Frame]:
        if self.dev is None:
            return None

        frame = GsUsbFrame()
        if not self.dev.read(frame, timeout_ms=timeout_ms):
            return None

        data = bytes(frame.data[: frame.can_dlc])
        self._log_frame("RX", frame.can_id, data)
        return Frame(can_id=frame.can_id, data=data)

    def wait_for_id(self, expected_id: int, timeout: float) -> Optional[bytes]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.read_frame(timeout_ms=100)
            if frame is None:
                continue
            if frame.can_id == expected_id:
                return frame.data
        return None

    def drain_rx(self, window_s: float = 0.15) -> None:
        deadline = time.monotonic() + window_s
        while time.monotonic() < deadline:
            frame = self.read_frame(timeout_ms=10)
            if frame is None:
                return


class FirmwareUploader:
    def __init__(self, client: CandleLightCanClient, verbose: bool = True, slave_num: int = 0):
        self.client = client
        self.verbose = verbose
        self.slave_num = slave_num
        self.firmware: Optional[bytes] = None
        self.is_master = False
        self.total_blocks = 0
        self.sent_blocks = 0

    def load_firmware(self, path: Path, is_master: bool) -> bool:
        try:
            self.firmware = path.read_bytes()
        except Exception as exc:
            print(f"Failed to read firmware: {exc}")
            return False

        self.is_master = is_master
        self.total_blocks = (len(self.firmware) + FW_BLOCK_SIZE_BYTES - 1) // FW_BLOCK_SIZE_BYTES
        target = "master" if self.is_master else "slave"
        print(f"Firmware: {path}")
        print(f"Size: {len(self.firmware)} bytes")
        print(f"Blocks: {self.total_blocks}")
        print(f"Target: {target}")
        return True

    def init_update(self) -> bool:
        init_id = CAN_FLAG_MASTER_OS if self.is_master else CAN_FLAG_SLAVE_OS
        request = struct.pack(">H", self.total_blocks)

        print(f"Init update: send 0x{init_id:03X}, total_blocks={self.total_blocks}")
        self.client.drain_rx()
        if not self.client.send_frame(init_id, request):
            return False

        deadline = time.monotonic() + TIMEOUT_INIT
        while time.monotonic() < deadline:
            response = self.client.wait_for_id(CAN_ACK_BLOCK, timeout=0.5)
            if response is None or not response:
                continue

            if response[0] == 0xFF:
                print("Target flash is erased and ready.")
                return True

            if self.verbose:
                print(f"Waiting for erase completion, status=0x{response[0]:02X}")
            time.sleep(0.2)

        print(f"Timeout waiting for init ACK on 0x{CAN_ACK_BLOCK:03X}.")
        return False

    def send_block(self, block_num: int) -> bool:
        if self.firmware is None:
            return False

        block_offset = block_num * FW_BLOCK_SIZE_BYTES
        block_len = min(FW_BLOCK_SIZE_BYTES, len(self.firmware) - block_offset)
        block_data = self.firmware[block_offset : block_offset + block_len]
        can_id = CAN_MASTER_OS if self.is_master else CAN_SLAVE_OS

        if self.verbose:
            print(f"Sending block #{block_num}: offset={block_offset}, len={block_len}")

        for packet_num in range(FW_BLOCK_SIZE_PACKETS):
            global_idx = block_num * FW_BLOCK_SIZE_PACKETS + packet_num
            packet_offset = packet_num * FW_PACKET_DATA_SIZE

            if packet_offset < block_len:
                packet = block_data[packet_offset : packet_offset + FW_PACKET_DATA_SIZE]
                if len(packet) < FW_PACKET_DATA_SIZE:
                    packet += b"\xFF" * (FW_PACKET_DATA_SIZE - len(packet))
            else:
                packet = b"\xFF" * FW_PACKET_DATA_SIZE

            frame_data = struct.pack(">H", global_idx) + packet[:FW_PACKET_DATA_SIZE]

            for attempt in range(1, MAX_RETRIES + 1):
                if self.client.send_frame(can_id, frame_data):
                    break
                if attempt == MAX_RETRIES:
                    print(f"Failed to send packet {packet_num} of block {block_num}.")
                    return False
                time.sleep(0.02)

            time.sleep(0.02)

        response = self.client.wait_for_id(CAN_ACK_BLOCK, TIMEOUT_ACK)
        if response is None or len(response) < 2:
            print(f"ACK timeout for block #{block_num}.")
            return False

        acked_block = struct.unpack(">H", response[:2])[0]
        if acked_block != block_num:
            print(
                f"Block ACK mismatch: expected #{block_num}, got #{acked_block}. "
                "STM32 block indexing is out of sync."
            )
            return False

        return True

    def post_upload(self) -> None:
        if self.is_master:
            return

        print(
            f"Sending slave restart command: ID=0x{CAN_SLAVE_UPDATE_START:03X}, "
            f"slave_num={self.slave_num}"
        )
        self.client.send_frame(CAN_SLAVE_UPDATE_START, bytes([self.slave_num & 0xFF]))

    def upload(self) -> bool:
        if self.firmware is None:
            print("Firmware is not loaded.")
            return False

        if not self.init_update():
            return False

        print(f"Uploading {self.total_blocks} block(s)...")
        for block_num in range(self.total_blocks):
            if not self.send_block(block_num):
                return False
            self.sent_blocks += 1

            if not self.verbose:
                percent = (self.sent_blocks / self.total_blocks) * 100.0
                print(
                    f"\rProgress: {percent:6.2f}% "
                    f"({self.sent_blocks}/{self.total_blocks})",
                    end="",
                    flush=True,
                )

        if not self.verbose:
            print()

        print("Upload completed successfully.")
        self.post_upload()
        return True


class RegisterDumper:
    def __init__(
        self,
        client: CandleLightCanClient,
        total_timeout: float,
        gap_timeout: float,
        attempts: int,
    ):
        self.client = client
        self.total_timeout = total_timeout
        self.gap_timeout = gap_timeout
        self.attempts = attempts

    def request_once(self, captures: Dict[int, BlockCapture]) -> bool:
        self.client.drain_rx()
        if not self.client.send_frame(CAN_CMD_GET_REGS, b""):
            return False

        start = time.monotonic()
        last_relevant_rx: Optional[float] = None
        saw_relevant = False

        while (time.monotonic() - start) < self.total_timeout:
            frame = self.client.read_frame(timeout_ms=100)
            now = time.monotonic()

            if frame is None:
                if saw_relevant and last_relevant_rx is not None:
                    if (now - last_relevant_rx) >= self.gap_timeout:
                        break
                continue

            block_index, packet_index = resolve_block_packet(frame.can_id)
            if block_index is None or packet_index is None:
                continue

            captures[block_index].packets[packet_index] = frame.data
            saw_relevant = True
            last_relevant_rx = now

            if all(capture.is_complete() for capture in captures.values()):
                break

        return saw_relevant

    def read_all(self) -> Dict[int, BlockCapture]:
        captures = {
            block_index: BlockCapture(block_index=block_index)
            for block_index in CAN_BLOCK_BASE_IDS
        }

        for attempt in range(1, self.attempts + 1):
            print(f"Requesting registers over CAN (attempt {attempt}/{self.attempts})...")
            saw_relevant = self.request_once(captures)
            if all(capture.is_complete() for capture in captures.values()):
                break
            if not saw_relevant:
                print("No register packets received on this attempt.")

        return captures

    def print_report(self, captures: Dict[int, BlockCapture]) -> bool:
        any_complete = False

        for block_index in sorted(CAN_BLOCK_BASE_IDS):
            capture = captures[block_index]
            base_id = capture.base_id
            print("=" * 72)
            print(
                f"Battery block {block_index + 1} "
                f"(IDs 0x{base_id:03X}-0x{base_id + CAN_PACKETS_PER_BLOCK - 1:03X})"
            )

            if not capture.packets:
                print("  No data received.")
                continue

            missing = capture.missing_packets()
            if missing:
                missing_ids = ", ".join(f"0x{base_id + idx:03X}" for idx in missing)
                print(
                    f"  Incomplete block: received {len(capture.packets)}/"
                    f"{CAN_PACKETS_PER_BLOCK} packets"
                )
                print(f"  Missing packet IDs: {missing_ids}")
                continue

            try:
                payload = capture.assemble_payload()
                registers = decode_registers(payload)
            except ValueError as exc:
                print(f"  Decode error: {exc}")
                continue

            any_complete = True
            for definition, raw_value in zip(REGISTER_LAYOUT, registers):
                value = to_signed16(raw_value) if definition.signed else raw_value
                print(
                    f"  reg[{definition.index:02d}] "
                    f"{definition.name:<24} = {value:7d} (0x{raw_value:04X})"
                )

        print("=" * 72)
        return any_complete


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="CandleLight CAN tool: register dump and firmware upload."
    )
    subparsers = parser.add_subparsers(dest="command")

    dump_parser = subparsers.add_parser(
        "dump",
        help="Request all Modbus registers over CAN and print them.",
    )
    dump_parser.add_argument("--bitrate", type=int, default=125000, help="CAN bitrate in bps.")
    dump_parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_COLLECT_TIMEOUT,
        help="Max collection time per request in seconds.",
    )
    dump_parser.add_argument(
        "--gap-timeout",
        type=float,
        default=DEFAULT_GAP_TIMEOUT,
        help="Stop reading after this idle gap once relevant frames started to arrive.",
    )
    dump_parser.add_argument(
        "--attempts",
        type=int,
        default=DEFAULT_ATTEMPTS,
        help="How many GET_REGS requests to send before giving up.",
    )
    dump_parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print every CAN TX/RX frame.",
    )

    upload_parser = subparsers.add_parser(
        "upload",
        help="Upload firmware over CAN using the existing STM32 boot/update protocol.",
    )
    upload_parser.add_argument("firmware", type=Path, help="Path to firmware image.")
    upload_parser.add_argument("--master", action="store_true", help="Upload master firmware.")
    upload_parser.add_argument("--bitrate", type=int, default=125000, help="CAN bitrate in bps.")
    upload_parser.add_argument("--quiet", action="store_true", help="Minimal upload output.")
    upload_parser.add_argument(
        "--slave-num",
        type=int,
        default=0,
        help="Slave number for the post-upload restart command.",
    )

    return parser


def normalize_argv(argv: Sequence[str]) -> list[str]:
    if not argv:
        return ["dump"]
    if argv[0] in {"dump", "upload", "-h", "--help"}:
        return list(argv)
    if argv[0].startswith("-"):
        return ["dump", *argv]
    return ["upload", *argv]


def run_dump(args: argparse.Namespace) -> int:
    client = CandleLightCanClient(bitrate=args.bitrate, verbose=args.verbose)
    try:
        if not client.connect():
            return 1

        dumper = RegisterDumper(
            client=client,
            total_timeout=args.timeout,
            gap_timeout=args.gap_timeout,
            attempts=max(1, args.attempts),
        )
        captures = dumper.read_all()
        return 0 if dumper.print_report(captures) else 2
    finally:
        client.disconnect()


def run_upload(args: argparse.Namespace) -> int:
    if not args.firmware.exists():
        print(f"Firmware file does not exist: {args.firmware}")
        return 1

    verbose = not args.quiet
    client = CandleLightCanClient(bitrate=args.bitrate, verbose=verbose)
    try:
        if not client.connect():
            return 1

        uploader = FirmwareUploader(
            client=client,
            verbose=verbose,
            slave_num=args.slave_num,
        )
        if not uploader.load_firmware(args.firmware, is_master=args.master):
            return 1
        return 0 if uploader.upload() else 2
    finally:
        client.disconnect()


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    normalized_argv = normalize_argv(sys.argv[1:] if argv is None else argv)
    args = parser.parse_args(normalized_argv)

    try:
        if args.command == "dump":
            return run_dump(args)
        if args.command == "upload":
            return run_upload(args)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        return 130

    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
