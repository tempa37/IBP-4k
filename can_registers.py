#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

try:
    from gs_usb.constants import CAN_EFF_FLAG, CAN_EFF_MASK, CAN_SFF_MASK
    from gs_usb.gs_usb import GsUsb
    from gs_usb.gs_usb_frame import GsUsbFrame
except ImportError:
    print("Error: gs_usb is not installed.")
    print("Install it with: pip install gs_usb")
    sys.exit(1)


FW_BLOCK_SIZE_PACKETS = 10
FW_PACKET_DATA_SIZE = 6
FW_BLOCK_SIZE_BYTES = FW_BLOCK_SIZE_PACKETS * FW_PACKET_DATA_SIZE

CAN_PROTOCOL_ADDR_KOU = 1
CAN_PROTOCOL_ADDR_IBP4K = 20

CAN_PROTOCOL_MSG_INA260_VOLTAGE = 20
CAN_PROTOCOL_MSG_INA260_CURRENT = 21
CAN_PROTOCOL_MSG_BAT_SOC = 22
CAN_PROTOCOL_MSG_BQ_STATUS = 23
CAN_PROTOCOL_MSG_ERROR_FLAGS = 24
CAN_PROTOCOL_MSG_UART_VERSION = 25
CAN_PROTOCOL_MSG_BAT_CAPACITY_MAH = 26
CAN_PROTOCOL_MSG_BAT_NOMINAL_CAP = 27
CAN_PROTOCOL_MSG_BQ_COULOMB_COUNT_MAH = 28
CAN_PROTOCOL_MSG_CAN_BUS_OFF_COUNTER = 30
CAN_PROTOCOL_MSG_WDG_RESET_COUNTER = 31
CAN_PROTOCOL_MSG_LAST_ERROR_TIMESTAMP = 32
CAN_PROTOCOL_MSG_LAST_ERROR_CODE = 33
CAN_PROTOCOL_MSG_FW_SLAVE_BEGIN = 40
CAN_PROTOCOL_MSG_FW_SLAVE_DATA = 41
CAN_PROTOCOL_MSG_FW_ACK = 42
CAN_PROTOCOL_MSG_FW_MASTER_BEGIN = 43
CAN_PROTOCOL_MSG_FW_MASTER_DATA = 44
CAN_PROTOCOL_MSG_FW_SLAVE_UPDATE_START = 45

CAN_PROTOCOL_PRIORITY_DEFAULT = 0
CAN_PROTOCOL_PRIORITY_DIAGNOSTIC = 3

TIMEOUT_INIT = 10.0
TIMEOUT_ACK = 1.5
MAX_RETRIES = 3

DEFAULT_REQUEST_TIMEOUT = 0.75
DEFAULT_REQUEST_ATTEMPTS = 3
DEFAULT_REQUEST_PAUSE_MS = 40

ERROR_FLAG_BITS = (
    (0, "BQ_ERROR"),
    (1, "INA_ERROR"),
    (2, "BALANCE_WARNING"),
    (3, "BALANCE_CRITICAL"),
    (4, "COMM_ERROR"),
)

LAST_ERROR_CODE_DESCRIPTIONS = {
    0x00: "Ошибок не зарегистрировано",
    0x01: "Переход CAN-контроллера в состояние Bus Off",
    0x02: "Сброс по Independent Watchdog (IWDG)",
    0x03: "Ошибка контрольной суммы диагностических данных в Flash",
    0x04: "Ошибка записи в Flash",
}


@dataclass(frozen=True)
class RegisterRequestDef:
    msg_id: int
    name: str
    description: str
    signed: bool = False
    diagnostic: bool = False
    dlc: int = 8

    @property
    def request_priority(self) -> int:
        return CAN_PROTOCOL_PRIORITY_DEFAULT

    @property
    def response_priority(self) -> int:
        return (
            CAN_PROTOCOL_PRIORITY_DIAGNOSTIC
            if self.diagnostic
            else CAN_PROTOCOL_PRIORITY_DEFAULT
        )


REQUEST_LAYOUT = [
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_INA260_VOLTAGE,
        "INA260_VOLTAGE",
        "Напряжение на нагрузке (мВ).",
    ),
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_INA260_CURRENT,
        "INA260_CURRENT",
        "Ток через нагрузку (мА). Знаковое.",
        signed=True,
    ),
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_BAT_SOC,
        "BAT_SOC",
        "Текущий процент заряда (SoC), 0-100%.",
    ),
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_BQ_STATUS,
        "BQ_STATUS",
        "Статус BQ76930. Ст. байт = состояние (state), мл. байт = системный статус (sys_stat).",
    ),
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_ERROR_FLAGS,
        "ERROR_FLAGS",
        "Битовая маска ошибок.",
    ),
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_UART_VERSION,
        "UART_VERSION",
        "Версия протокола/прошивки UART.",
    ),
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_BAT_CAPACITY_MAH,
        "BAT_CAPACITY_MAH",
        "Текущая абсолютная емкость (мАч).",
    ),
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_BAT_NOMINAL_CAP,
        "BAT_NOMINAL_CAP",
        "Номинальная емкость батареи из EEPROM (мАч).",
    ),
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_BQ_COULOMB_COUNT_MAH,
        "BQ_COULOMB_COUNT_MAH",
        "Значение кулоновского счетчика (мАч).",
    ),
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_CAN_BUS_OFF_COUNTER,
        "CAN_BusOff_Counter",
        "Счетчик входов в состояние Bus Off.",
        diagnostic=True,
    ),
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_WDG_RESET_COUNTER,
        "WDG_Reset_Counter",
        "Счетчик сбросов по Independent Watchdog.",
        diagnostic=True,
    ),
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_LAST_ERROR_TIMESTAMP,
        "Last_Error_Timestamp",
        "Время последней ошибки (часы наработки).",
        diagnostic=True,
    ),
    RegisterRequestDef(
        CAN_PROTOCOL_MSG_LAST_ERROR_CODE,
        "Last_Error_Code",
        "Код последней ошибки.",
        diagnostic=True,
        dlc=4,
    ),
]


@dataclass
class Frame:
    raw_can_id: int
    arbitration_id: int
    is_extended_id: bool
    data: bytes


@dataclass
class RegisterReadResult:
    definition: RegisterRequestDef
    request_id: int
    response_id: int
    values: Optional[list[int]] = None
    data: bytes = b""
    error: Optional[str] = None


def build_protocol_can_id(src: int, dst: int, msg_id: int, priority: int = 0) -> int:
    if not (0 <= src <= 0x7F):
        raise ValueError(f"SRC out of range: {src}")
    if not (0 <= dst <= 0x7F):
        raise ValueError(f"DST out of range: {dst}")
    if not (0 <= msg_id <= 0x3FF):
        raise ValueError(f"MSG_ID out of range: {msg_id}")
    if not (0 <= priority <= 0x03):
        raise ValueError(f"Priority out of range: {priority}")

    return src | (dst << 7) | (msg_id << 14) | (priority << 27)


def build_update_can_id(src: int, dst: int, msg_id: int) -> int:
    return build_protocol_can_id(
        src=src,
        dst=dst,
        msg_id=msg_id,
        priority=CAN_PROTOCOL_PRIORITY_DEFAULT,
    )


def decode_u16_values_le(payload: bytes, signed: bool) -> list[int]:
    if len(payload) != 8:
        raise ValueError(f"expected 8 payload bytes, got {len(payload)}")

    values: list[int] = []
    for offset in range(0, 8, 2):
        values.append(int.from_bytes(payload[offset : offset + 2], "little", signed=signed))
    return values


def decode_last_error_codes(payload: bytes) -> list[int]:
    if len(payload) != 4:
        raise ValueError(f"expected 4 payload bytes, got {len(payload)}")
    return list(payload[:4])


def channel_label(index: int) -> str:
    return f"ПСИП {index} (лоток АКБ {index + 1})"


def format_error_flags(value: int) -> str:
    active = [name for bit, name in ERROR_FLAG_BITS if value & (1 << bit)]
    if not active:
        return f"{value} (0x{value & 0xFFFF:04X}, no active bits)"
    return f"{value} (0x{value & 0xFFFF:04X}, {', '.join(active)})"


def format_bq_status(value: int) -> str:
    state = (value >> 8) & 0xFF
    sys_stat = value & 0xFF
    return (
        f"{value} (0x{value & 0xFFFF:04X}, "
        f"state=0x{state:02X}, sys_stat=0x{sys_stat:02X})"
    )


def format_last_error_code(value: int) -> str:
    description = LAST_ERROR_CODE_DESCRIPTIONS.get(value, "Зарезервировано")
    return f"0x{value:02X} ({description})"


def format_register_value(definition: RegisterRequestDef, value: int) -> str:
    if definition.name == "BQ_STATUS":
        return format_bq_status(value & 0xFFFF)
    if definition.name == "ERROR_FLAGS":
        return format_error_flags(value & 0xFFFF)
    if definition.name == "Last_Error_Code":
        return format_last_error_code(value & 0xFF)
    return f"{value} (0x{value & 0xFFFF:04X})"


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

    @staticmethod
    def _decode_can_id(raw_can_id: int) -> tuple[int, bool]:
        is_extended_id = bool(raw_can_id & CAN_EFF_FLAG)
        mask = CAN_EFF_MASK if is_extended_id else CAN_SFF_MASK
        return raw_can_id & mask, is_extended_id

    def _log_frame(self, direction: str, raw_can_id: int, data: bytes) -> None:
        if not self.verbose:
            return

        arbitration_id, is_extended_id = self._decode_can_id(raw_can_id)
        kind = "EXT" if is_extended_id else "STD"
        id_width = 8 if is_extended_id else 3
        hex_data = " ".join(f"{byte:02X}" for byte in data)
        print(
            f"{direction} {kind} ID=0x{arbitration_id:0{id_width}X} "
            f"DLC={len(data)} DATA=[{hex_data}]"
        )

    def send_frame(self, can_id: int, data: bytes = b"", is_extended_id: bool = False) -> bool:
        if self.dev is None:
            print("CAN device is not connected.")
            return False

        try:
            payload = bytes(data)
            raw_can_id = can_id | CAN_EFF_FLAG if is_extended_id else can_id
            self._log_frame("TX", raw_can_id, payload)
            frame = GsUsbFrame(can_id=raw_can_id, data=payload)
            return bool(self.dev.send(frame))
        except Exception as exc:
            id_width = 8 if is_extended_id else 3
            print(f"TX failed for 0x{can_id:0{id_width}X}: {exc}")
            return False

    def read_frame(self, timeout_ms: int = 100) -> Optional[Frame]:
        if self.dev is None:
            return None

        frame = GsUsbFrame()
        if not self.dev.read(frame, timeout_ms=timeout_ms):
            return None

        data = bytes(frame.data[: frame.can_dlc])
        self._log_frame("RX", frame.can_id, data)
        arbitration_id, is_extended_id = self._decode_can_id(frame.can_id)
        return Frame(
            raw_can_id=frame.can_id,
            arbitration_id=arbitration_id,
            is_extended_id=is_extended_id,
            data=data,
        )

    def wait_for_id(
        self,
        expected_id: int,
        timeout: float,
        is_extended_id: Optional[bool] = None,
    ) -> Optional[bytes]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.read_frame(timeout_ms=100)
            if frame is None:
                continue
            if is_extended_id is not None and frame.is_extended_id != is_extended_id:
                continue
            if frame.arbitration_id == expected_id:
                return frame.data
        return None

    def drain_rx(self, window_s: float = 0.15) -> None:
        deadline = time.monotonic() + window_s
        while time.monotonic() < deadline:
            frame = self.read_frame(timeout_ms=10)
            if frame is None:
                return


class FirmwareUploader:
    def __init__(
        self,
        client: CandleLightCanClient,
        verbose: bool = True,
        slave_num: int = 0,
        src_addr: int = CAN_PROTOCOL_ADDR_KOU,
        dst_addr: int = CAN_PROTOCOL_ADDR_IBP4K,
    ):
        self.client = client
        self.verbose = verbose
        self.slave_num = slave_num
        self.src_addr = src_addr
        self.dst_addr = dst_addr
        self.firmware: Optional[bytes] = None
        self.is_master = False
        self.total_blocks = 0
        self.sent_blocks = 0

    def _command_id(self, msg_id: int) -> int:
        return build_update_can_id(self.src_addr, self.dst_addr, msg_id)

    def _ack_id(self) -> int:
        return build_update_can_id(
            self.dst_addr,
            self.src_addr,
            CAN_PROTOCOL_MSG_FW_ACK,
        )

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
        init_id = self._command_id(
            CAN_PROTOCOL_MSG_FW_MASTER_BEGIN
            if self.is_master
            else CAN_PROTOCOL_MSG_FW_SLAVE_BEGIN
        )
        ack_id = self._ack_id()
        request = struct.pack(">H", self.total_blocks)

        print(f"Init update: send 0x{init_id:08X}, total_blocks={self.total_blocks}")
        self.client.drain_rx()
        if not self.client.send_frame(init_id, request, is_extended_id=True):
            return False

        deadline = time.monotonic() + TIMEOUT_INIT
        while time.monotonic() < deadline:
            response = self.client.wait_for_id(ack_id, timeout=0.5, is_extended_id=True)
            if response is None or not response:
                continue

            if response[0] == 0xFF:
                print("Target flash is erased and ready.")
                return True

            if self.verbose:
                print(f"Waiting for erase completion, status=0x{response[0]:02X}")
            time.sleep(0.2)

        print(f"Timeout waiting for init ACK on 0x{ack_id:08X}.")
        return False

    def send_block(self, block_num: int) -> bool:
        if self.firmware is None:
            return False

        block_offset = block_num * FW_BLOCK_SIZE_BYTES
        block_len = min(FW_BLOCK_SIZE_BYTES, len(self.firmware) - block_offset)
        block_data = self.firmware[block_offset : block_offset + block_len]
        can_id = self._command_id(
            CAN_PROTOCOL_MSG_FW_MASTER_DATA
            if self.is_master
            else CAN_PROTOCOL_MSG_FW_SLAVE_DATA
        )
        ack_id = self._ack_id()

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
                if self.client.send_frame(can_id, frame_data, is_extended_id=True):
                    break
                if attempt == MAX_RETRIES:
                    print(f"Failed to send packet {packet_num} of block {block_num}.")
                    return False
                time.sleep(0.02)

            time.sleep(0.02)

        response = self.client.wait_for_id(ack_id, TIMEOUT_ACK, is_extended_id=True)
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

        restart_id = self._command_id(CAN_PROTOCOL_MSG_FW_SLAVE_UPDATE_START)
        print(
            f"Sending slave restart command: ID=0x{restart_id:08X}, "
            f"slave_num={self.slave_num}"
        )
        self.client.send_frame(
            restart_id,
            bytes([self.slave_num & 0xFF]),
            is_extended_id=True,
        )

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
        request_timeout: float,
        attempts: int,
        pause_ms: int,
        src_addr: int,
        dst_addr: int,
    ):
        self.client = client
        self.request_timeout = request_timeout
        self.attempts = attempts
        self.pause_s = max(0.0, pause_ms / 1000.0)
        self.src_addr = src_addr
        self.dst_addr = dst_addr

    def _decode_response(
        self,
        definition: RegisterRequestDef,
        payload: bytes,
    ) -> list[int]:
        if definition.msg_id == CAN_PROTOCOL_MSG_LAST_ERROR_CODE:
            return decode_last_error_codes(payload)
        return decode_u16_values_le(payload, signed=definition.signed)

    def request_register(self, definition: RegisterRequestDef) -> RegisterReadResult:
        request_id = build_protocol_can_id(
            self.src_addr,
            self.dst_addr,
            definition.msg_id,
            definition.request_priority,
        )
        response_id = build_protocol_can_id(
            self.dst_addr,
            self.src_addr,
            definition.msg_id,
            definition.response_priority,
        )

        last_error = "no response received"
        for attempt in range(1, self.attempts + 1):
            print(
                f"Requesting {definition.name} "
                f"(MSG_ID={definition.msg_id}, attempt {attempt}/{self.attempts})..."
            )
            self.client.drain_rx()

            if not self.client.send_frame(request_id, b"", is_extended_id=True):
                last_error = "request transmit failed"
                time.sleep(self.pause_s)
                continue

            payload = self.client.wait_for_id(
                response_id,
                timeout=self.request_timeout,
                is_extended_id=True,
            )
            if payload is None:
                last_error = "response timeout"
                time.sleep(self.pause_s)
                continue

            try:
                values = self._decode_response(definition, payload)
            except ValueError as exc:
                last_error = str(exc)
                time.sleep(self.pause_s)
                continue

            time.sleep(self.pause_s)
            return RegisterReadResult(
                definition=definition,
                request_id=request_id,
                response_id=response_id,
                values=values,
                data=payload,
            )

        return RegisterReadResult(
            definition=definition,
            request_id=request_id,
            response_id=response_id,
            error=last_error,
        )

    def read_all(self) -> list[RegisterReadResult]:
        return [self.request_register(definition) for definition in REQUEST_LAYOUT]

    def print_report(self, results: list[RegisterReadResult]) -> bool:
        all_ok = True
        for result in results:
            definition = result.definition
            print("=" * 88)
            print(definition.name)
            print(f"  MSG_ID:      {definition.msg_id}")
            print(f"  Request ID:  0x{result.request_id:X}")
            print(f"  Response ID: 0x{result.response_id:X}")
            print(f"  Description: {definition.description}")

            if result.error is not None or result.values is None:
                print(f"  Error:       {result.error or 'unknown error'}")
                all_ok = False
                continue

            print(f"  Raw payload: {' '.join(f'{byte:02X}' for byte in result.data)}")
            for index, value in enumerate(result.values):
                print(
                    f"  {channel_label(index):<24} = "
                    f"{format_register_value(definition, value)}"
                )

        print("=" * 88)
        return all_ok


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="CandleLight CAN tool: TЗ register dump and firmware upload."
    )
    subparsers = parser.add_subparsers(dest="command")

    dump_parser = subparsers.add_parser(
        "dump",
        help="Request all CAN registers from the TЗ protocol and print them separately.",
    )
    dump_parser.add_argument("--bitrate", type=int, default=125000, help="CAN bitrate in bps.")
    dump_parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_REQUEST_TIMEOUT,
        help="Max wait time for one register response in seconds.",
    )
    dump_parser.add_argument(
        "--attempts",
        type=int,
        default=DEFAULT_REQUEST_ATTEMPTS,
        help="How many times to retry each register request before giving up.",
    )
    dump_parser.add_argument(
        "--pause-ms",
        type=int,
        default=DEFAULT_REQUEST_PAUSE_MS,
        help="Delay between register requests in milliseconds.",
    )
    dump_parser.add_argument(
        "--src",
        type=int,
        default=CAN_PROTOCOL_ADDR_KOU,
        help="SRC address for requests.",
    )
    dump_parser.add_argument(
        "--dst",
        type=int,
        default=CAN_PROTOCOL_ADDR_IBP4K,
        help="DST address for requests.",
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
        "--src",
        type=int,
        default=CAN_PROTOCOL_ADDR_KOU,
        help="SRC address for update commands.",
    )
    upload_parser.add_argument(
        "--dst",
        type=int,
        default=CAN_PROTOCOL_ADDR_IBP4K,
        help="DST address for update commands.",
    )
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
            request_timeout=args.timeout,
            attempts=max(1, args.attempts),
            pause_ms=max(0, args.pause_ms),
            src_addr=args.src,
            dst_addr=args.dst,
        )
        results = dumper.read_all()
        return 0 if dumper.print_report(results) else 2
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
            src_addr=args.src,
            dst_addr=args.dst,
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
