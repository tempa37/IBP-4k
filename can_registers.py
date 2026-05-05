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
CAN_PROTOCOL_MSG_DIAG_LOG_INFO = 34
CAN_PROTOCOL_MSG_DIAG_LOG_READ = 35
CAN_PROTOCOL_MSG_FW_SLAVE_BEGIN = 40
CAN_PROTOCOL_MSG_FW_SLAVE_DATA = 41
CAN_PROTOCOL_MSG_FW_ACK = 42
CAN_PROTOCOL_MSG_FW_MASTER_BEGIN = 43
CAN_PROTOCOL_MSG_FW_MASTER_DATA = 44
CAN_PROTOCOL_MSG_FW_SLAVE_UPDATE_START = 45

CAN_PROTOCOL_PRIORITY_DEFAULT = 0
CAN_PROTOCOL_PRIORITY_DIAGNOSTIC = 3

DEFAULT_CAN_BITRATE = 250000

TIMEOUT_INIT = 10.0
TIMEOUT_ACK = 1.5
MAX_RETRIES = 3

DEFAULT_REQUEST_TIMEOUT = 2.0
DEFAULT_REQUEST_ATTEMPTS = 5
DEFAULT_REQUEST_PAUSE_MS = 80
DEFAULT_LOG_REQUEST_TIMEOUT = 5.0
DEFAULT_LOG_REQUEST_ATTEMPTS = 5
DEFAULT_LOG_REQUEST_PAUSE_MS = 100

# Параметры формата диагностического журнала должны совпадать с diagnostic_log.h.
DIAG_LOG_MAGIC = 0x474F4C44
DIAG_LOG_RECORD_SIZE = 32
DIAG_LOG_RECORD_CHUNK_SIZE = 8
DIAG_LOG_INVALID_INDEX = 0xFFFF
DIAG_LOG_RECORD_STRUCT = struct.Struct("<IIIIHHHHBBBBI")
DIAG_LOG_ERROR_WDG_RESET = 0x02
DIAG_LOG_ERROR_STARTUP_RESET = 0x05
DIAG_LOG_SOURCE_WDG = 0x02
DIAG_LOG_SOURCE_RESET = 0x04

DIAG_LOG_RESET_FLAG_BITS = (
    (0, "BORRST: brown-out/POR/PDR"),
    (1, "PINRST: NRST/external reset pin"),
    (2, "PORRST: power-on/power-down reset"),
    (3, "SFTRST: software reset"),
    (4, "IWDGRST: Independent Watchdog reset"),
    (5, "WWDGRST: Window Watchdog reset"),
    (6, "LPWRRST: low-power reset"),
)

DIAG_LOG_LEGACY_WDG_DETAIL_BITS = (
    (0, "IWDGRST: Independent Watchdog reset"),
    (1, "WWDGRST: Window Watchdog reset"),
)

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
    0x02: "Сброс по watchdog (IWDG/WWDG)",
    0x03: "Ошибка контрольной суммы диагностических данных в Flash",
    0x04: "Ошибка записи в Flash",
    0x05: "Сброс/перезапуск MCU, зафиксированный на старте",
}

DIAG_LOG_EVENT_DESCRIPTIONS = {
    0x01: "диагностическая ошибка",
}

DIAG_LOG_SOURCE_DESCRIPTIONS = {
    0x01: "CAN",
    0x02: "Watchdog",
    0x03: "Flash",
    0x04: "Reset/RCC",
}

DIAG_LOG_RAW_STATUS_DESCRIPTIONS = {
    "valid": "валидная запись",
    "erased": "пустой слот / много 0xFF",
    "bad_magic": "нет magic DLOG",
    "bad_header": "битый заголовок",
    "bad_crc": "CRC не сошелся",
    "timeout": "таймаут ответа",
    "bad_response": "битый CAN-ответ",
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


@dataclass(frozen=True)
class DiagnosticLogInfo:
    record_capacity: int
    record_size: int
    chunks_per_record: int
    valid_count: int
    last_record_index: int


@dataclass(frozen=True)
class DiagnosticLogRecord:
    physical_index: int
    magic: int
    sequence: int
    flags: int
    detail: int
    size: int
    timestamp_hours: int
    can_busoff_counter: int
    wdg_reset_counter: int
    error_code: int
    event_type: int
    channel: int
    source: int
    crc32: int
    raw: bytes
    crc_ok: bool = True
    calculated_crc32: int = 0


@dataclass(frozen=True)
class DiagnosticLogRawRecord:
    physical_index: int
    raw: bytes
    chunks_received: int
    complete: bool
    status: str
    parsed: Optional[DiagnosticLogRecord] = None


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


def diagnostic_log_crc32(data: bytes) -> int:
    # CRC журнала повторяет алгоритм прошивки: reflected CRC32 с финальным xor.
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
            crc &= 0xFFFFFFFF
    return crc ^ 0xFFFFFFFF


def parse_diagnostic_log_info(payload: bytes) -> DiagnosticLogInfo:
    if len(payload) != 8:
        raise ValueError(f"expected 8 log-info bytes, got {len(payload)}")

    return DiagnosticLogInfo(
        record_capacity=int.from_bytes(payload[0:2], "little"),
        record_size=payload[2],
        chunks_per_record=payload[3],
        valid_count=int.from_bytes(payload[4:6], "little"),
        last_record_index=int.from_bytes(payload[6:8], "little"),
    )


def parse_diagnostic_log_record(index: int, raw: bytes) -> Optional[DiagnosticLogRecord]:
    if len(raw) != DIAG_LOG_RECORD_SIZE:
        return None

    (
        magic,
        sequence,
        flags,
        detail,
        size,
        timestamp_hours,
        can_busoff_counter,
        wdg_reset_counter,
        error_code,
        event_type,
        channel,
        source,
        crc32,
    ) = DIAG_LOG_RECORD_STRUCT.unpack(raw)

    if magic != DIAG_LOG_MAGIC or size != DIAG_LOG_RECORD_SIZE:
        return None

    calculated_crc = diagnostic_log_crc32(raw[: DIAG_LOG_RECORD_STRUCT.size - 4])

    return DiagnosticLogRecord(
        physical_index=index,
        magic=magic,
        sequence=sequence,
        flags=flags,
        detail=detail,
        size=size,
        timestamp_hours=timestamp_hours,
        can_busoff_counter=can_busoff_counter,
        wdg_reset_counter=wdg_reset_counter,
        error_code=error_code,
        event_type=event_type,
        channel=channel,
        source=source,
        crc32=crc32,
        raw=raw,
        crc_ok=(calculated_crc == crc32),
        calculated_crc32=calculated_crc,
    )


def format_diagnostic_log_channel(channel: int) -> str:
    if channel == 0xFF:
        return "глобальная ошибка"
    if 0 <= channel <= 3:
        return f"ПСИП {channel} (лоток АКБ {channel + 1})"
    return f"неизвестный канал 0x{channel:02X}"


def format_named_bitmask(value: int, bit_descriptions: Sequence[tuple[int, str]]) -> str:
    active = [name for bit, name in bit_descriptions if value & (1 << bit)]
    if not active:
        return f"0x{value:08X} (нет активных известных битов)"
    return f"0x{value:08X} ({'; '.join(active)})"


def format_reset_summary(reset_flags: int) -> str:
    reasons: list[str] = []

    if reset_flags & ((1 << 4) | (1 << 5)):
        reasons.append("watchdog reset")
    if reset_flags & (1 << 3):
        reasons.append("software reset")
    if reset_flags & ((1 << 0) | (1 << 2)):
        reasons.append("power/brown-out reset")
    if reset_flags & (1 << 1):
        reasons.append("NRST/external reset pin flag")
    if reset_flags & (1 << 6):
        reasons.append("low-power reset")

    if not reasons:
        return "неизвестная причина по нормализованным флагам"
    return "; ".join(reasons)


def format_diagnostic_log_reset_details(record: DiagnosticLogRecord) -> list[str]:
    if record.source == DIAG_LOG_SOURCE_RESET:
        return [
            f"Reset flags decoded:       {format_named_bitmask(record.flags, DIAG_LOG_RESET_FLAG_BITS)}",
            f"Reset reason summary:      {format_reset_summary(record.flags)}",
            f"Reset detail source:       raw RCC->CSR before __HAL_RCC_CLEAR_RESET_FLAGS()",
        ]

    if (
        record.source == DIAG_LOG_SOURCE_WDG
        and record.error_code == DIAG_LOG_ERROR_WDG_RESET
        and record.flags == 0
    ):
        return [
            f"Legacy WDG detail decoded: {format_named_bitmask(record.detail, DIAG_LOG_LEGACY_WDG_DETAIL_BITS)}",
        ]

    return []


def format_diagnostic_log_record(record: DiagnosticLogRecord) -> list[str]:
    error_description = LAST_ERROR_CODE_DESCRIPTIONS.get(
        record.error_code,
        "зарезервировано/неизвестно",
    )
    event_description = DIAG_LOG_EVENT_DESCRIPTIONS.get(
        record.event_type,
        f"неизвестный тип 0x{record.event_type:02X}",
    )
    source_description = DIAG_LOG_SOURCE_DESCRIPTIONS.get(
        record.source,
        f"неизвестный источник 0x{record.source:02X}",
    )

    lines = [
        f"Физический индекс:         {record.physical_index}",
        f"Sequence:                  {record.sequence}",
        f"Код ошибки:                0x{record.error_code:02X} ({error_description})",
        f"Тип события:               0x{record.event_type:02X} ({event_description})",
        f"Канал:                     {format_diagnostic_log_channel(record.channel)}",
        f"Источник:                  {source_description}",
        f"Время ошибки, часов:       {record.timestamp_hours}",
        f"CAN BusOff Counter:        {record.can_busoff_counter}",
        f"WDG Reset Counter:         {record.wdg_reset_counter}",
        f"Flags:                     0x{record.flags:08X}",
        f"Detail:                    0x{record.detail:08X}",
    ]
    lines.extend(format_diagnostic_log_reset_details(record))
    lines.extend([
        f"CRC32:                     0x{record.crc32:08X}",
        f"CRC32 calculated:          0x{record.calculated_crc32:08X}",
        f"CRC32 check:               {'OK' if record.crc_ok else 'FAIL (STM32 запись отдала, но PC-проверка не сошлась)'}",
        f"Raw:                       {' '.join(f'{byte:02X}' for byte in record.raw)}",
    ])
    return lines


def format_hex_bytes(data: bytes) -> str:
    if not data:
        return "<нет байтов>"

    return " ".join(f"{byte:02X}" for byte in data)


def format_diagnostic_log_raw_record(
    raw_record: DiagnosticLogRawRecord,
    chunks_per_record: int,
) -> list[str]:
    status = DIAG_LOG_RAW_STATUS_DESCRIPTIONS.get(raw_record.status, raw_record.status)
    parsed = raw_record.parsed
    crc_text = "нет"
    sequence_text = "нет"

    if parsed is not None:
        sequence_text = str(parsed.sequence)
        crc_text = (
            "OK"
            if parsed.crc_ok
            else f"FAIL (expected 0x{parsed.crc32:08X}, calculated 0x{parsed.calculated_crc32:08X})"
        )

    return [
        f"Физический индекс:         {raw_record.physical_index}",
        f"Статус:                    {status}",
        f"Получено CAN-чанков:       {raw_record.chunks_received}/{chunks_per_record}",
        f"Полная запись:             {'да' if raw_record.complete else 'нет'}",
        f"Sequence:                  {sequence_text}",
        f"CRC32 check:               {crc_text}",
        f"Raw:                       {format_hex_bytes(raw_record.raw)}",
    ]


def build_diagnostic_log_text(
    info: DiagnosticLogInfo,
    records: list[DiagnosticLogRecord],
    raw_records: list[DiagnosticLogRawRecord],
) -> str:
    last_index = (
        "нет"
        if info.last_record_index == DIAG_LOG_INVALID_INDEX
        else str(info.last_record_index)
    )
    lines = [
        "Выгрузка диагностического журнала IBP-4k",
        f"Дата выгрузки:             {time.strftime('%Y-%m-%d %H:%M:%S')}",
        "",
        "Сводка журнала:",
        f"  Физических слотов:       {info.record_capacity}",
        f"  Валидных записей:        {info.valid_count}",
        f"  Размер записи:           {info.record_size} байт",
        f"  CAN-чанков на запись:    {info.chunks_per_record}",
        f"  Индекс последней записи: {last_index}",
        f"  Получено сырых записей:  {len(raw_records)}",
        f"  Расшифровано валидных:   {len(records)}",
        "",
        "Сырые CAN-данные:",
    ]

    if not raw_records:
        lines.append("  Сырые записи не получены.")
    else:
        for number, raw_record in enumerate(raw_records, start=1):
            lines.extend(["", "-" * 88, f"Raw запись #{number}"])
            lines.extend(format_diagnostic_log_raw_record(raw_record, info.chunks_per_record))

    lines.extend(["", "Расшифровка валидных записей:"])

    if not records:
        lines.append("  Валидных записей для выгрузки нет.")
        return "\n".join(lines) + "\n"

    for number, record in enumerate(records, start=1):
        lines.extend(["", "=" * 88, f"Запись #{number}"])
        lines.extend(format_diagnostic_log_record(record))

    return "\n".join(lines) + "\n"


class CandleLightCanClient:
    def __init__(self, bitrate: int = DEFAULT_CAN_BITRATE, verbose: bool = False):
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
        echo_id: Optional[int] = None,
        require_echo_before_response: bool = False,
    ) -> Optional[bytes]:
        saw_echo = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.read_frame(timeout_ms=100)
            if frame is None:
                continue
            if is_extended_id is not None and frame.is_extended_id != is_extended_id:
                continue
            if frame.arbitration_id == expected_id:
                if require_echo_before_response and echo_id is not None and not saw_echo:
                    if self.verbose:
                        print(
                            f"Ignoring stale pre-echo response 0x{expected_id:08X} "
                            f"while waiting for TX echo 0x{echo_id:08X}."
                        )
                    continue
                return frame.data
            if echo_id is not None and frame.arbitration_id == echo_id:
                saw_echo = True
        if saw_echo:
            print(
                f"Only local TX echo was seen for 0x{echo_id:08X}; "
                f"no target response 0x{expected_id:08X}."
            )
        return None

    def drain_rx(self, window_s: float = 0.15, max_s: float = 0.5) -> None:
        now = time.monotonic()
        hard_deadline = now + max_s
        quiet_deadline = now + window_s

        while now < quiet_deadline and now < hard_deadline:
            frame = self.read_frame(timeout_ms=10)
            now = time.monotonic()
            if frame is not None:
                quiet_deadline = min(now + window_s, hard_deadline)


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
        if self.firmware is None:
            print("Firmware is not loaded.")
            return False

        image_size = len(self.firmware)
        init_id = self._command_id(
            CAN_PROTOCOL_MSG_FW_MASTER_BEGIN
            if self.is_master
            else CAN_PROTOCOL_MSG_FW_SLAVE_BEGIN
        )
        ack_id = self._ack_id()
        request = struct.pack(">HI", self.total_blocks, image_size)

        print(
            f"Init update: send 0x{init_id:08X}, "
            f"total_blocks={self.total_blocks}, size={image_size}"
        )
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


class DiagnosticLogDumper:
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

    def _request_payload(self, msg_id: int, payload: bytes = b"") -> Optional[bytes]:
        request_id = build_protocol_can_id(
            self.src_addr,
            self.dst_addr,
            msg_id,
            CAN_PROTOCOL_PRIORITY_DEFAULT,
        )
        response_id = build_protocol_can_id(
            self.dst_addr,
            self.src_addr,
            msg_id,
            CAN_PROTOCOL_PRIORITY_DIAGNOSTIC,
        )

        # Лог-запросы отправляем тем же рабочим способом, что и dump: drain -> TX EXT -> wait RX EXT.
        for attempt in range(1, self.attempts + 1):
            print(f"Requesting log MSG_ID={msg_id}, attempt {attempt}/{self.attempts}...")
            self.client.drain_rx()

            if not self.client.send_frame(request_id, payload, is_extended_id=True):
                time.sleep(self.pause_s)
                continue

            response = self.client.wait_for_id(
                response_id,
                timeout=self.request_timeout,
                is_extended_id=True,
                echo_id=request_id,
                require_echo_before_response=True,
            )
            if response is not None:
                time.sleep(self.pause_s)
                return response

            time.sleep(self.pause_s)

        return None

    def read_info(self) -> Optional[DiagnosticLogInfo]:
        payload = self._request_payload(CAN_PROTOCOL_MSG_DIAG_LOG_INFO)
        if payload is None:
            print("Log info timeout.")
            return None

        try:
            return parse_diagnostic_log_info(payload)
        except ValueError as exc:
            print(f"Bad log info response: {exc}")
            return None

    def read_record_raw(self, index: int, chunks_per_record: int) -> DiagnosticLogRawRecord:
        raw = bytearray()
        chunks_received = 0

        # Одна запись во Flash больше одного CAN-кадра, поэтому читаем ее частями по 8 байт.
        for chunk in range(chunks_per_record):
            request = struct.pack("<HB", index & 0xFFFF, chunk & 0xFF)
            payload = self._request_payload(CAN_PROTOCOL_MSG_DIAG_LOG_READ, request)
            if payload is None or len(payload) != DIAG_LOG_RECORD_CHUNK_SIZE:
                if payload is not None:
                    raw.extend(payload)
                    chunks_received += 1
                return DiagnosticLogRawRecord(
                    physical_index=index,
                    raw=bytes(raw),
                    chunks_received=chunks_received,
                    complete=False,
                    status="timeout" if payload is None else "bad_response",
                )

            raw.extend(payload)
            chunks_received += 1

            # Первый чанк содержит magic. Если слот пустой или чужой, дальше CAN не долбим.
            if chunk == 0:
                if all(byte == 0xFF for byte in payload):
                    return DiagnosticLogRawRecord(
                        physical_index=index,
                        raw=bytes(raw),
                        chunks_received=chunks_received,
                        complete=False,
                        status="erased",
                    )
                if int.from_bytes(payload[:4], "little") != DIAG_LOG_MAGIC:
                    return DiagnosticLogRawRecord(
                        physical_index=index,
                        raw=bytes(raw),
                        chunks_received=chunks_received,
                        complete=False,
                        status="bad_magic",
                    )

        parsed = parse_diagnostic_log_record(index, bytes(raw))
        if parsed is None:
            status = "bad_header"
        elif not parsed.crc_ok:
            status = "bad_crc"
        else:
            status = "valid"

        return DiagnosticLogRawRecord(
            physical_index=index,
            raw=bytes(raw),
            chunks_received=chunks_received,
            complete=(chunks_received == chunks_per_record),
            status=status,
            parsed=parsed,
        )

    def read_record(self, index: int, chunks_per_record: int) -> Optional[DiagnosticLogRecord]:
        raw_record = self.read_record_raw(index, chunks_per_record)
        if raw_record.status == "valid" and raw_record.parsed is not None:
            return raw_record.parsed
        return None

    def read_records(
        self,
        info: DiagnosticLogInfo,
        *,
        index: Optional[int],
        latest: bool,
        limit: Optional[int],
    ) -> tuple[list[DiagnosticLogRecord], list[DiagnosticLogRawRecord]]:
        records: list[DiagnosticLogRecord] = []
        raw_records: list[DiagnosticLogRawRecord] = []

        if index is not None:
            raw_record = self.read_record_raw(index, info.chunks_per_record)
            raw_records.append(raw_record)
            if raw_record.status == "valid" and raw_record.parsed is not None:
                records.append(raw_record.parsed)
            return records, raw_records

        if latest:
            if info.last_record_index == DIAG_LOG_INVALID_INDEX:
                return records, raw_records
            raw_record = self.read_record_raw(info.last_record_index, info.chunks_per_record)
            raw_records.append(raw_record)
            if raw_record.status == "valid" and raw_record.parsed is not None:
                records.append(raw_record.parsed)
            return records, raw_records

        if info.last_record_index == DIAG_LOG_INVALID_INDEX or info.record_capacity <= 0:
            return records, raw_records

        target_count = info.record_capacity
        if limit is not None:
            target_count = min(target_count, max(0, limit))
        if target_count == 0:
            return records, raw_records

        physical_index = info.last_record_index
        for scanned_count in range(target_count):
            if scanned_count == 0 or (scanned_count + 1) % 256 == 0:
                print(
                    "Reading log slots backward: "
                    f"{scanned_count + 1}/{target_count}, "
                    f"index {physical_index}, valid {len(records)}"
                )

            raw_record = self.read_record_raw(physical_index, info.chunks_per_record)
            raw_records.append(raw_record)
            if raw_record.status != "valid" or raw_record.parsed is None:
                print(
                    "Log scan stopped at index "
                    f"{physical_index}: "
                    f"{DIAG_LOG_RAW_STATUS_DESCRIPTIONS.get(raw_record.status, raw_record.status)}"
                )
                break

            records.append(raw_record.parsed)
            if len(records) >= target_count:
                break

            physical_index = (physical_index - 1) % info.record_capacity

        # В txt удобнее видеть журнал по времени записи, а не по физическим слотам кольца.
        records.sort(key=lambda item: item.sequence)
        return records, raw_records

    def dump_to_txt(
        self,
        output_path: Path,
        *,
        index: Optional[int],
        latest: bool,
        limit: Optional[int],
    ) -> bool:
        info = self.read_info()
        if info is None:
            return False

        if info.record_size != DIAG_LOG_RECORD_SIZE:
            print(
                f"Unsupported log record size: {info.record_size}, "
                f"expected {DIAG_LOG_RECORD_SIZE}."
            )
            return False
        if info.chunks_per_record * DIAG_LOG_RECORD_CHUNK_SIZE != info.record_size:
            print(
                f"Unsupported log chunk layout: {info.chunks_per_record} chunks "
                f"for {info.record_size} bytes."
            )
            return False

        records, raw_records = self.read_records(info, index=index, latest=latest, limit=limit)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            build_diagnostic_log_text(info, records, raw_records),
            encoding="utf-8",
        )
        print(f"Log txt written: {output_path}")
        print(f"Records written: {len(records)}")
        print(f"Raw records read: {len(raw_records)}")
        return True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="CandleLight CAN tool: TЗ register dump and firmware upload."
    )
    subparsers = parser.add_subparsers(dest="command")

    dump_parser = subparsers.add_parser(
        "dump",
        help="Request all CAN registers from the TЗ protocol and print them separately.",
    )
    dump_parser.add_argument("--bitrate", type=int, default=DEFAULT_CAN_BITRATE, help="CAN bitrate in bps.")
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

    log_parser = subparsers.add_parser(
        "log",
        help="Read diagnostic Flash log through CAN and save it to a TXT file.",
    )
    log_parser.add_argument("--bitrate", type=int, default=DEFAULT_CAN_BITRATE, help="CAN bitrate in bps.")
    log_parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_LOG_REQUEST_TIMEOUT,
        help="Max wait time for one log response in seconds.",
    )
    log_parser.add_argument(
        "--attempts",
        type=int,
        default=DEFAULT_LOG_REQUEST_ATTEMPTS,
        help="How many times to retry each log request before giving up.",
    )
    log_parser.add_argument(
        "--pause-ms",
        type=int,
        default=DEFAULT_LOG_REQUEST_PAUSE_MS,
        help="Delay between log chunk requests in milliseconds.",
    )
    log_parser.add_argument(
        "--src",
        type=int,
        default=CAN_PROTOCOL_ADDR_KOU,
        help="SRC address for log requests.",
    )
    log_parser.add_argument(
        "--dst",
        type=int,
        default=CAN_PROTOCOL_ADDR_IBP4K,
        help="DST address for log requests.",
    )
    log_parser.add_argument(
        "--out",
        type=Path,
        default=Path("diagnostic_log.txt"),
        help="TXT file for the exported diagnostic log.",
    )
    log_parser.add_argument(
        "--index",
        type=int,
        help="Read only one physical log record index.",
    )
    log_parser.add_argument(
        "--latest",
        action="store_true",
        help="Read only the latest valid log record.",
    )
    log_parser.add_argument(
        "--limit",
        type=int,
        help="Stop full scan after this many valid records.",
    )
    log_parser.add_argument(
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
    upload_parser.add_argument("--bitrate", type=int, default=DEFAULT_CAN_BITRATE, help="CAN bitrate in bps.")
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
    if argv[0] in {"dump", "log", "upload", "-h", "--help"}:
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


def run_log(args: argparse.Namespace) -> int:
    if args.index is not None and args.latest:
        print("Use either --index or --latest, not both.")
        return 1

    client = CandleLightCanClient(bitrate=args.bitrate, verbose=args.verbose)
    try:
        if not client.connect():
            return 1

        dumper = DiagnosticLogDumper(
            client=client,
            request_timeout=args.timeout,
            attempts=max(1, args.attempts),
            pause_ms=max(0, args.pause_ms),
            src_addr=args.src,
            dst_addr=args.dst,
        )
        return (
            0
            if dumper.dump_to_txt(
                args.out,
                index=args.index,
                latest=args.latest,
                limit=args.limit,
            )
            else 2
        )
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
        if args.command == "log":
            return run_log(args)
        if args.command == "upload":
            return run_upload(args)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        return 130

    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
