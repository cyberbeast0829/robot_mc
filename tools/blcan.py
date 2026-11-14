#!/usr/bin/env python3
"""CAN USB OpenBootloader test tool (python-can).

Features:
- Basic CAN raw send/receive
- Single OpenBootloader command tests
- Full firmware upgrade flow
- Detailed logs and clear error output for debugging

Supported interfaces:
- CANable (slcan, default, requires pyserial)
- PCAN (pcan)
- any other python-can backend via --interface / --channel

本仓库用法（本文件是 OpenBootloader 仓库同名工具的同步副本，代码保持一致）：
- 应用链接在 0x08008000，加载器是 0x08000000 处的 OpenBootloader（32 KB）。
- 先让设备停在 bootloader 里：`python tools/mc_ep.py --usb call action.enter_bootloader_mode`
  （或串口菜单按 `b`），再跑本工具的 `sync` / `upgrade`。
- 典型升级：
  `python tools/blcan.py upgrade --firmware build/mit_h473.bin --erase auto`
  `        --addr 0x08008000 --chunk-size 256 --verify --go-addr 0x08008000`
- 详细说明与故障排查见 tools/blcan_使用说明.md；升级契约见 docs/DEV.md。
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import can


ACK = 0x79
NACK = 0x1F
SYNC = 0x7F

CMD_GET = 0x00
CMD_GET_VERSION = 0x01
CMD_GET_ID = 0x02
CMD_READ_MEMORY = 0x11
CMD_GO = 0x21
CMD_WRITE_MEMORY = 0x31
CMD_ERASE_MEMORY = 0x44
CMD_WRITE_PROTECT = 0x63
CMD_WRITE_UNPROTECT = 0x73
CMD_READ_PROTECT = 0x82
CMD_READ_UNPROTECT = 0x92


class BootloaderError(RuntimeError):
    """Protocol/communication failure."""


@dataclass
class FirmwareSegment:
    address: int
    data: bytes


@dataclass(frozen=True)
class McuProfile:
    name: str
    flash_base: int
    flash_page_size: int
    sync_id: int


MCU_PROFILES = {
    "n32h473": McuProfile(
        name="n32h473",
        flash_base=0x08000000,
        flash_page_size=0x2000,
        sync_id=0x128,
    ),
    "hpm6e6y": McuProfile(
        name="hpm6e6y",
        flash_base=0x80000000,
        flash_page_size=0x1000,
        sync_id=0x128,
    ),
}


def parse_int(text: str) -> int:
    return int(text, 0)


def parse_hex_bytes(text: str) -> bytes:
    cleaned = text.replace(" ", "").replace("_", "")
    if cleaned.startswith("0x") or cleaned.startswith("0X"):
        cleaned = cleaned[2:]
    if len(cleaned) == 0:
        return b""
    if len(cleaned) % 2 != 0:
        raise ValueError("hex data length must be even")
    return bytes.fromhex(cleaned)


def chunked(data: bytes, size: int) -> Iterable[bytes]:
    for i in range(0, len(data), size):
        yield data[i : i + size]



def guess_default_channel() -> str:
    """Guess the CANable (slcan) serial port channel.

    Tries to find a connected USB serial port that looks like a CANable
    device; falls back to a platform-specific default otherwise.
    """
    try:
        import serial.tools.list_ports as list_ports

        ports = list(list_ports.comports())
        if not ports:
            raise OSError("no serial ports detected")
        for p in ports:
            desc = " ".join(filter(None, [p.description, p.product, p.manufacturer])).lower()
            if "canable" in desc or "candlelight" in desc or ("can" in desc and "usb" in desc):
                return p.device
        for p in ports:
            if p.device.lower().startswith(("com", "/dev/tty")):
                return p.device
    except Exception:
        pass
    if sys.platform.startswith("win"):
        return "COM3"
    if sys.platform.startswith("darwin"):
        return "/dev/tty.usbmodem0000"
    return "/dev/ttyACM0"


def resolve_channel(interface: str, channel: Optional[str]) -> str:
    """Resolve the CAN channel for the selected interface when not given."""
    if channel:
        return channel
    if interface == "pcan":
        return "PCAN_USBBUS1"
    return guess_default_channel()

def parse_int_list(text: str) -> List[int]:
    if not text.strip():
        return []
    return [parse_int(item.strip()) for item in text.split(",") if item.strip()]


def parse_intel_hex(file_path: str) -> List[FirmwareSegment]:
    upper = 0
    memory = {}

    with open(file_path, "r", encoding="ascii", errors="strict") as f:
        for lineno, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise ValueError(f"HEX line {lineno}: missing ':'")

            raw = bytes.fromhex(line[1:])
            if len(raw) < 5:
                raise ValueError(f"HEX line {lineno}: too short")

            byte_count = raw[0]
            address = (raw[1] << 8) | raw[2]
            rec_type = raw[3]
            data = raw[4 : 4 + byte_count]
            checksum = raw[4 + byte_count]

            if ((sum(raw[:-1]) + checksum) & 0xFF) != 0:
                raise ValueError(f"HEX line {lineno}: checksum error")

            if rec_type == 0x00:
                base = (upper << 16) + address
                for i, b in enumerate(data):
                    memory[base + i] = b
            elif rec_type == 0x01:
                break
            elif rec_type == 0x04:
                if byte_count != 2:
                    raise ValueError(f"HEX line {lineno}: invalid extended linear address")
                upper = (data[0] << 8) | data[1]
            else:
                # Ignore unsupported record types (02/03/05) for this tool.
                continue

    if not memory:
        return []

    addresses = sorted(memory.keys())
    segments: List[FirmwareSegment] = []
    start = addresses[0]
    last = addresses[0]
    buf = bytearray([memory[start]])

    for addr in addresses[1:]:
        if addr == last + 1:
            buf.append(memory[addr])
        else:
            segments.append(FirmwareSegment(start, bytes(buf)))
            start = addr
            buf = bytearray([memory[addr]])
        last = addr

    segments.append(FirmwareSegment(start, bytes(buf)))
    return segments


def load_firmware_segments(file_path: str, base_addr: Optional[int]) -> List[FirmwareSegment]:
    lower = file_path.lower()
    if lower.endswith(".bin"):
        if base_addr is None:
            raise ValueError("--addr is required when firmware is .bin")
        with open(file_path, "rb") as f:
            data = f.read()
        return [FirmwareSegment(base_addr, data)]

    if lower.endswith(".hex"):
        segments = parse_intel_hex(file_path)
        if not segments:
            raise ValueError("no data records found in HEX file")

        if base_addr is None:
            return segments

        merged = b"".join(seg.data for seg in segments)
        return [FirmwareSegment(base_addr, merged)]

    raise ValueError("firmware file must be .bin or .hex")


def apply_mcu_profile_defaults(args: argparse.Namespace) -> McuProfile:
    profile = MCU_PROFILES[args.mcu]

    if args.sync_id is None:
        args.sync_id = profile.sync_id

    if hasattr(args, "flash_base") and args.flash_base is None:
        args.flash_base = profile.flash_base

    if hasattr(args, "page_size") and args.page_size is None:
        args.page_size = profile.flash_page_size

    return profile


class BootloaderCanClient:
    def __init__(
        self,
        interface: str,
        channel: str,
        bitrate: int,
        timeout: float,
        sync_id: int,
        logger: logging.Logger,
    ) -> None:
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.timeout = timeout
        self.sync_id = sync_id
        self.log = logger
        self.bus: Optional[can.BusABC] = None

    def __enter__(self) -> "BootloaderCanClient":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        channel = resolve_channel(self.interface, self.channel)
        self.log.info("Open CAN bus: interface=%s channel=%s bitrate=%d", self.interface, channel, self.bitrate)
        self.bus = can.Bus(interface=self.interface, channel=channel, bitrate=self.bitrate)
        # Some drivers (e.g. PCAN) may keep old frames in the RX queue across short reconnects.
        self.flush_rx()

    def close(self) -> None:
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None
            self.log.info("CAN bus closed")

    def flush_rx(self) -> int:
        if self.bus is None:
            raise BootloaderError("CAN bus is not open")

        dropped = 0
        while True:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break
            dropped += 1
        if dropped:
            self.log.debug("Dropped %d stale RX frame(s)", dropped)
        return dropped

    def send_frame(self, can_id: int, data: bytes) -> None:
        if self.bus is None:
            raise BootloaderError("CAN bus is not open")
        if len(data) > 8:
            raise BootloaderError("classic CAN supports max 8 bytes per frame")

        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg)
        self.log.debug("TX ID=0x%03X DLC=%d DATA=%s", can_id, len(data), data.hex(" ").upper())

    def recv_frame(self, timeout: Optional[float] = None, expected_id: Optional[int] = None) -> can.Message:
        if self.bus is None:
            raise BootloaderError("CAN bus is not open")

        wait_s = self.timeout if timeout is None else timeout

        while True:
            msg = self.bus.recv(timeout=wait_s)
            if msg is None:
                raise BootloaderError(f"timeout waiting for CAN frame (expected_id={expected_id})")
            if msg.is_error_frame:
                self.log.warning("RX error frame")
                continue
            if msg.is_extended_id:
                self.log.debug("Ignore extended frame ID=0x%X", msg.arbitration_id)
                continue
            if expected_id is not None and msg.arbitration_id != expected_id:
                self.log.debug("Ignore frame ID=0x%03X while waiting ID=0x%03X", msg.arbitration_id, expected_id)
                continue

            self.log.debug(
                "RX ID=0x%03X DLC=%d DATA=%s",
                msg.arbitration_id,
                msg.dlc,
                bytes(msg.data).hex(" ").upper(),
            )
            return msg

    def recv_frame_any(self, expected_ids: Sequence[int], timeout: Optional[float] = None) -> can.Message:
        if self.bus is None:
            raise BootloaderError("CAN bus is not open")

        wait_s = self.timeout if timeout is None else timeout
        allowed = set(expected_ids)

        while True:
            msg = self.bus.recv(timeout=wait_s)
            if msg is None:
                raise BootloaderError(f"timeout waiting for CAN frame (expected_ids={sorted(allowed)})")
            if msg.is_error_frame:
                self.log.warning("RX error frame")
                continue
            if msg.is_extended_id:
                self.log.debug("Ignore extended frame ID=0x%X", msg.arbitration_id)
                continue
            if msg.arbitration_id not in allowed:
                self.log.debug("Ignore frame ID=0x%03X while waiting IDs=%s", msg.arbitration_id, sorted(allowed))
                continue

            self.log.debug(
                "RX ID=0x%03X DLC=%d DATA=%s",
                msg.arbitration_id,
                msg.dlc,
                bytes(msg.data).hex(" ").upper(),
            )
            return msg

    def recv_single_byte(self, expected_id: int, timeout: Optional[float] = None) -> int:
        msg = self.recv_frame(expected_id=expected_id, timeout=timeout)
        if msg.dlc < 1:
            raise BootloaderError("received frame has empty payload")
        return int(msg.data[0])

    def expect_ack(
        self,
        expected_id: int,
        stage: str,
        timeout: Optional[float] = None,
        allow_id0_fallback: bool = True,
    ) -> None:
        if allow_id0_fallback:
            msg = self.recv_frame_any((expected_id, 0x000), timeout=timeout)
            if msg.arbitration_id == 0x000:
                self.log.warning("ACK/NACK for %s received on CAN ID 0x000, using compatibility fallback", stage)
        else:
            msg = self.recv_frame(expected_id=expected_id, timeout=timeout)

        if msg.dlc < 1:
            raise BootloaderError(f"received frame has empty payload at {stage}")

        b = int(msg.data[0])
        if b == ACK:
            self.log.info("ACK at %s", stage)
            return
        if b == NACK:
            raise BootloaderError(f"NACK at {stage}")
        raise BootloaderError(f"unexpected byte 0x{b:02X} at {stage}, expected ACK/NACK")

    def sync(self) -> None:
        self.flush_rx()
        self.log.info("Send SYNC (0x7F) to ID 0x%03X", self.sync_id)
        self.send_frame(self.sync_id, bytes([SYNC]))
        # 老版本 bootloader 的握手 ACK 发在 CAN ID 0x000（TxHeader.ID 未按 SYNC 帧更新），
        # 新版本已改成回 0x128；这里允许 ID 0 兜底，两种固件都能握手。
        self.expect_ack(self.sync_id, "sync", allow_id0_fallback=True)

    def cmd_simple(self, opcode: int, payload: bytes = b"", auto_sync: bool = True) -> None:
        if auto_sync:
            self.sync()
        else:
            # Without SYNC, proactively drop stale frames from previous commands.
            self.flush_rx()
        self.send_frame(opcode, payload)

    def get(self, auto_sync: bool = True) -> Tuple[int, int, List[int]]:
        self.cmd_simple(CMD_GET, auto_sync=auto_sync)
        self.expect_ack(CMD_GET, "get:start")
        count = self.recv_single_byte(CMD_GET)
        version = self.recv_single_byte(CMD_GET)
        cmds = [self.recv_single_byte(CMD_GET) for _ in range(count)]
        self.expect_ack(CMD_GET, "get:end")
        return count, version, cmds

    def get_version(self, auto_sync: bool = True) -> Tuple[int, bytes]:
        self.cmd_simple(CMD_GET_VERSION, auto_sync=auto_sync)
        self.expect_ack(CMD_GET_VERSION, "get_version:start")
        version = self.recv_single_byte(CMD_GET_VERSION)
        dummy = bytes(self.recv_frame(expected_id=CMD_GET_VERSION).data)
        self.expect_ack(CMD_GET_VERSION, "get_version:end")
        return version, dummy

    def get_id(self, auto_sync: bool = True) -> int:
        self.cmd_simple(CMD_GET_ID, auto_sync=auto_sync)
        self.expect_ack(CMD_GET_ID, "get_id:start")
        msg = self.recv_frame(expected_id=CMD_GET_ID)
        if msg.dlc < 2:
            raise BootloaderError("GET ID response length < 2")
        dev_id = (int(msg.data[0]) << 8) | int(msg.data[1])
        self.expect_ack(CMD_GET_ID, "get_id:end")
        return dev_id

    def read_memory(self, address: int, size: int, auto_sync: bool = True) -> bytes:
        if size < 1 or size > 256:
            raise ValueError("read size must be 1..256")

        payload = address.to_bytes(4, "big") + bytes([size - 1])
        self.cmd_simple(CMD_READ_MEMORY, payload=payload, auto_sync=auto_sync)
        self.expect_ack(CMD_READ_MEMORY, "read_memory:start")

        frames = int(math.ceil(size / 8.0))
        out = bytearray()
        read_ids = (CMD_READ_MEMORY, 0x000)
        warned_id0 = False
        for _ in range(frames):
            msg = self.recv_frame_any(read_ids)
            if msg.arbitration_id == 0x000 and not warned_id0:
                self.log.warning("READ reply received on CAN ID 0x000, using compatibility fallback")
                warned_id0 = True
            out.extend(msg.data)

        end = self.recv_frame_any(read_ids)
        if end.dlc < 1:
            raise BootloaderError("received frame has empty payload at read_memory:end")
        b = int(end.data[0])
        if b == ACK:
            self.log.info("ACK at read_memory:end")
        elif b == NACK:
            raise BootloaderError("NACK at read_memory:end")
        else:
            raise BootloaderError(f"unexpected byte 0x{b:02X} at read_memory:end, expected ACK/NACK")

        return bytes(out[:size])

    def write_memory(self, address: int, data: bytes, auto_sync: bool = True) -> None:
        if len(data) < 1 or len(data) > 256:
            raise ValueError("write size must be 1..256")

        payload = address.to_bytes(4, "big") + bytes([len(data) - 1])
        self.cmd_simple(CMD_WRITE_MEMORY, payload=payload, auto_sync=auto_sync)
        self.expect_ack(CMD_WRITE_MEMORY, "write_memory:start")

        for part in chunked(data, 8):
            if len(part) < 8:
                part = part + (b"\xFF" * (8 - len(part)))
            self.send_frame(CMD_WRITE_MEMORY, part)

        self.expect_ack(CMD_WRITE_MEMORY, "write_memory:end")

    def erase_mass(self, mode: int, auto_sync: bool = True, erase_timeout: float = 120.0) -> None:
        # mode: 0xFFFF mass, 0xFFFE bank1, 0xFFFD bank2
        payload = mode.to_bytes(2, "big")
        self.cmd_simple(CMD_ERASE_MEMORY, payload=payload, auto_sync=auto_sync)
        self.expect_ack(CMD_ERASE_MEMORY, "erase_mass:start")
        self.log.info("Waiting for mass erase to complete (timeout=%ds)...", int(erase_timeout))
        self.expect_ack(CMD_ERASE_MEMORY, "erase_mass:end", timeout=erase_timeout)

    def erase_pages(self, pages: Sequence[int], auto_sync: bool = True, erase_timeout: float = 30.0) -> None:
        if len(pages) == 0:
            raise ValueError("erase pages list is empty")
        if len(pages) > 4:
            raise ValueError("max 4 pages per erase command (8-byte CAN frame = 4 x 2-byte page numbers)")

        cmd_payload = len(pages).to_bytes(2, "big")
        self.cmd_simple(CMD_ERASE_MEMORY, payload=cmd_payload, auto_sync=auto_sync)
        self.expect_ack(CMD_ERASE_MEMORY, "erase_pages:start")
        self.expect_ack(CMD_ERASE_MEMORY, "erase_pages:list_request")

        list_payload = bytearray()
        for p in pages:
            if p < 0 or p > 0xFFFF:
                raise ValueError(f"invalid page number: {p}")
            list_payload.extend(p.to_bytes(2, "big"))

        if len(list_payload) < 8:
            list_payload.extend(b"\x00" * (8 - len(list_payload)))

        self.send_frame(CMD_ERASE_MEMORY, bytes(list_payload))
        self.expect_ack(CMD_ERASE_MEMORY, "erase_pages:end", timeout=erase_timeout)

    def erase_pages_batched(self, pages: Sequence[int], auto_sync: bool = True, erase_timeout: float = 30.0) -> None:
        """Erase pages in batches of 4 (CAN frame limit). Sends multiple erase commands if needed."""
        if len(pages) == 0:
            raise ValueError("erase pages list is empty")

        page_list = list(pages)
        for i in range(0, len(page_list), 4):
            batch = page_list[i : i + 4]
            self.log.info("Erase batch %d/%d: pages %s", i // 4 + 1, math.ceil(len(page_list) / 4), batch)
            sync_this = auto_sync if i == 0 else False
            self.erase_pages(batch, auto_sync=sync_this, erase_timeout=erase_timeout)

    def go(self, address: int, auto_sync: bool = True) -> None:
        payload = address.to_bytes(4, "big")
        self.cmd_simple(CMD_GO, payload=payload, auto_sync=auto_sync)
        self.expect_ack(CMD_GO, "go:start")
        self.expect_ack(CMD_GO, "go:end")

    def write_protect(self, sectors: Sequence[int], auto_sync: bool = True) -> None:
        if len(sectors) > 7:
            raise ValueError("write protect sectors must be <= 7 (1 count + 7 bytes fits 8-byte frame)")
        payload = bytes([len(sectors)]) + bytes(int(x) & 0xFF for x in sectors)
        self.cmd_simple(CMD_WRITE_PROTECT, payload=payload, auto_sync=auto_sync)
        self.expect_ack(CMD_WRITE_PROTECT, "write_protect:start")
        self.expect_ack(CMD_WRITE_PROTECT, "write_protect:end")

    def write_unprotect(self, auto_sync: bool = True) -> None:
        self.cmd_simple(CMD_WRITE_UNPROTECT, auto_sync=auto_sync)
        self.expect_ack(CMD_WRITE_UNPROTECT, "write_unprotect:start")
        self.expect_ack(CMD_WRITE_UNPROTECT, "write_unprotect:end")

    def read_protect(self, auto_sync: bool = True) -> None:
        self.cmd_simple(CMD_READ_PROTECT, auto_sync=auto_sync)
        self.expect_ack(CMD_READ_PROTECT, "read_protect:start")
        self.expect_ack(CMD_READ_PROTECT, "read_protect:end")

    def read_unprotect(self, auto_sync: bool = True) -> None:
        self.cmd_simple(CMD_READ_UNPROTECT, auto_sync=auto_sync)
        self.expect_ack(CMD_READ_UNPROTECT, "read_unprotect:start")
        self.expect_ack(CMD_READ_UNPROTECT, "read_unprotect:end")


def make_logger(verbose: bool) -> logging.Logger:
    logger = logging.getLogger("pcan_bl")
    logger.setLevel(logging.DEBUG if verbose else logging.INFO)
    logger.handlers.clear()

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.DEBUG if verbose else logging.INFO)
    handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", "%H:%M:%S"))
    logger.addHandler(handler)
    return logger


def run_upgrade(client: BootloaderCanClient, args: argparse.Namespace, log: logging.Logger) -> None:
    segments = load_firmware_segments(args.firmware, args.addr)
    total_size = sum(len(seg.data) for seg in segments)
    log.info("Upgrade image: %s, segments=%d, bytes=%d", args.firmware, len(segments), total_size)

    if args.sync:
        client.sync()

    if args.erase == "mass":
        client.erase_mass(0xFFFF, auto_sync=False, erase_timeout=args.erase_timeout)
    elif args.erase == "bank1":
        client.erase_mass(0xFFFE, auto_sync=False, erase_timeout=args.erase_timeout)
    elif args.erase == "bank2":
        client.erase_mass(0xFFFD, auto_sync=False, erase_timeout=args.erase_timeout)
    elif args.erase == "pages":
        pages = parse_int_list(args.pages)
        if not pages:
            raise ValueError("--erase pages requires --pages")
        client.erase_pages_batched(pages, auto_sync=False, erase_timeout=args.erase_timeout)
    elif args.erase == "auto":
        if args.page_size <= 0:
            raise ValueError("--page-size must be > 0")

        all_pages = []
        for seg in segments:
            if seg.address < args.flash_base:
                raise ValueError(
                    f"segment address 0x{seg.address:08X} is below flash base 0x{args.flash_base:08X}"
                )
            start_page = (seg.address - args.flash_base) // args.page_size
            end_page = (seg.address + len(seg.data) - 1 - args.flash_base) // args.page_size
            all_pages.extend(range(start_page, end_page + 1))

        pages = sorted(set(all_pages))
        log.info("Auto erase: %d page(s) %s (flash_base=0x%08X, page_size=0x%X)", len(pages), pages, args.flash_base, args.page_size)
        client.erase_pages_batched(pages, auto_sync=False, erase_timeout=args.erase_timeout)
    elif args.erase == "none":
        log.info("Skip erase")
    else:
        raise ValueError(f"unknown erase mode: {args.erase}")

    written = 0
    for seg in segments:
        addr = seg.address
        for part in chunked(seg.data, args.chunk_size):
            client.write_memory(addr, part, auto_sync=False)
            written += len(part)
            addr += len(part)
            log.info("Write progress: %d/%d (%.1f%%)", written, total_size, 100.0 * written / total_size)

    if args.verify:
        log.info("Start readback verify")
        checked = 0
        for seg in segments:
            addr = seg.address
            for part in chunked(seg.data, args.chunk_size):
                rb = client.read_memory(addr, len(part), auto_sync=False)
                if rb != part:
                    for i, (a, b) in enumerate(zip(rb, part)):
                        if a != b:
                            raise BootloaderError(
                                f"verify mismatch at 0x{addr + i:08X}: read=0x{a:02X}, expected=0x{b:02X}"
                            )
                    raise BootloaderError(f"verify mismatch at block start 0x{addr:08X}")
                checked += len(part)
                addr += len(part)
                log.info("Verify progress: %d/%d (%.1f%%)", checked, total_size, 100.0 * checked / total_size)

    if args.go_addr is not None:
        client.go(args.go_addr, auto_sync=False)
        log.info("GO done: 0x%08X", args.go_addr)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="CAN OpenBootloader test tool (CANable/PCAN)")
    parser.add_argument(
        "--mcu",
        choices=sorted(MCU_PROFILES.keys()),
        default="n32h473",
        help="MCU profile used to fill default flash_base/page_size/sync_id",
    )
    parser.add_argument("--interface", default="slcan", help="python-can interface (default: slcan = CANable)")
    parser.add_argument(
        "--channel",
        default=None,
        help="CAN channel; default depends on --interface (CANable serial port e.g. COM3, or PCAN_USBBUS1)",
    )
    parser.add_argument("--bitrate", type=int, default=500000, help="CAN bitrate, default 500000")
    parser.add_argument("--timeout", type=float, default=1.0, help="receive timeout in seconds")
    parser.add_argument("--sync-id", type=parse_int, default=None, help="SYNC frame ID, defaults to selected MCU profile")
    parser.add_argument("--verbose", action="store_true", help="enable debug logs")

    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("sync", help="send SYNC and wait ACK")

    p_raw_send = sub.add_parser("raw-send", help="send one raw CAN frame")
    p_raw_send.add_argument("--id", required=True, type=parse_int, help="CAN ID")
    p_raw_send.add_argument("--data", required=True, help="hex bytes, e.g. 7F or 11 22 33")

    p_raw_recv = sub.add_parser("raw-recv", help="receive raw CAN frames")
    p_raw_recv.add_argument("--count", type=int, default=1, help="frame count to receive")
    p_raw_recv.add_argument("--expect-id", type=parse_int, default=None, help="optional expected CAN ID")

    p_get = sub.add_parser("get", help="GET (0x00)")
    p_get.add_argument("--sync", action="store_true", help="send SYNC before command")
    
    p_version = sub.add_parser("version", help="GET VERSION (0x01)")
    p_version.add_argument("--sync", action="store_true", help="send SYNC before command")
    
    p_id = sub.add_parser("id", help="GET ID (0x02)")
    p_id.add_argument("--sync", action="store_true", help="send SYNC before command")

    p_read = sub.add_parser("read", help="READ MEMORY (0x11)")
    p_read.add_argument("--addr", required=True, type=parse_int, help="read start address")
    p_read.add_argument("--len", required=True, type=int, help="read length 1..256")
    p_read.add_argument("--out", default=None, help="output file path")
    p_read.add_argument("--sync", action="store_true", help="send SYNC before command")

    p_write = sub.add_parser("write", help="WRITE MEMORY (0x31)")
    p_write.add_argument("--addr", required=True, type=parse_int, help="write address")
    src = p_write.add_mutually_exclusive_group(required=True)
    src.add_argument("--data", help="hex bytes, e.g. DEADBEEF")
    src.add_argument("--file", help="binary data file (raw bytes)")
    p_write.add_argument("--sync", action="store_true", help="send SYNC before command")

    p_erase = sub.add_parser("erase", help="ERASE MEMORY (0x44)")
    p_erase.add_argument(
        "--mode",
        required=True,
        choices=["mass", "bank1", "bank2", "pages"],
        help="erase mode",
    )
    p_erase.add_argument("--pages", default="", help="comma-separated page list, used by --mode pages")
    p_erase.add_argument("--erase-timeout", type=float, default=120.0, help="erase ACK timeout in seconds (default 120)")
    p_erase.add_argument("--sync", action="store_true", help="send SYNC before command")

    p_go = sub.add_parser("go", help="GO (0x21)")
    p_go.add_argument("--addr", required=True, type=parse_int, help="jump address")
    p_go.add_argument("--sync", action="store_true", help="send SYNC before command")

    p_wp = sub.add_parser("write-protect", help="WRITE PROTECT (0x63)")
    p_wp.add_argument("--sectors", default="", help="comma-separated sector list, max 7")
    p_wp.add_argument("--sync", action="store_true", help="send SYNC before command")

    p_wu = sub.add_parser("write-unprotect", help="WRITE UNPROTECT (0x73)")
    p_wu.add_argument("--sync", action="store_true", help="send SYNC before command")
    
    p_rp = sub.add_parser("read-protect", help="READ PROTECT (0x82)")
    p_rp.add_argument("--sync", action="store_true", help="send SYNC before command")
    
    p_ru = sub.add_parser("read-unprotect", help="READ UNPROTECT (0x92)")
    p_ru.add_argument("--sync", action="store_true", help="send SYNC before command")

    p_up = sub.add_parser("upgrade", help="full upgrade flow")
    p_up.add_argument("--firmware", required=True, help=".bin or .hex firmware file")
    p_up.add_argument("--addr", type=parse_int, default=None, help="base address for .bin, optional for .hex")
    p_up.add_argument("--chunk-size", type=int, default=256, help="write chunk size 1..256")
    p_up.add_argument(
        "--erase",
        choices=["none", "mass", "bank1", "bank2", "pages", "auto"],
        default="mass",
        help="erase strategy",
    )
    p_up.add_argument("--pages", default="", help="comma-separated pages when --erase pages")
    p_up.add_argument("--page-size", type=parse_int, default=None, help="flash page size in bytes, defaults to selected MCU profile")
    p_up.add_argument("--flash-base", type=parse_int, default=None, help="flash base address, defaults to selected MCU profile")
    p_up.add_argument("--erase-timeout", type=float, default=120.0, help="erase ACK timeout in seconds (default 120)")
    p_up.add_argument("--sync", action="store_true", help="send SYNC before upgrade flow")
    p_up.add_argument("--verify", action="store_true", help="readback verify after writing")
    p_up.add_argument("--go-addr", type=parse_int, default=None, help="optional GO address after upgrade")

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    log = make_logger(args.verbose)
    profile = apply_mcu_profile_defaults(args)

    log.info(
        "MCU profile: %s (sync_id=0x%03X, flash_base=0x%08X, page_size=0x%X)",
        profile.name,
        args.sync_id,
        args.flash_base if hasattr(args, "flash_base") and args.flash_base is not None else profile.flash_base,
        args.page_size if hasattr(args, "page_size") and args.page_size is not None else profile.flash_page_size,
    )

    if args.cmd == "upgrade":
        if args.chunk_size < 1 or args.chunk_size > 256:
            parser.error("--chunk-size must be 1..256")

    try:
        with BootloaderCanClient(
            interface=args.interface,
            channel=args.channel,
            bitrate=args.bitrate,
            timeout=args.timeout,
            sync_id=args.sync_id,
            logger=log,
        ) as client:
            if args.cmd == "sync":
                client.sync()
                log.info("SYNC OK")

            elif args.cmd == "raw-send":
                data = parse_hex_bytes(args.data)
                client.send_frame(args.id, data)
                log.info("raw-send OK")

            elif args.cmd == "raw-recv":
                for i in range(args.count):
                    msg = client.recv_frame(expected_id=args.expect_id)
                    log.info(
                        "[%d/%d] RX ID=0x%03X DLC=%d DATA=%s",
                        i + 1,
                        args.count,
                        msg.arbitration_id,
                        msg.dlc,
                        bytes(msg.data).hex(" ").upper(),
                    )

            elif args.cmd == "get":
                count, version, cmds = client.get(auto_sync=args.sync)
                log.info("GET: count=%d version=0x%02X cmds=%s", count, version, [f"0x{x:02X}" for x in cmds])

            elif args.cmd == "version":
                version, dummy = client.get_version(auto_sync=args.sync)
                log.info("VERSION: 0x%02X dummy=%s", version, dummy.hex(" ").upper())

            elif args.cmd == "id":
                dev_id = client.get_id(auto_sync=args.sync)
                log.info("DEVICE ID: 0x%04X", dev_id)

            elif args.cmd == "read":
                data = client.read_memory(args.addr, args.len, auto_sync=args.sync)
                log.info("READ %d bytes @ 0x%08X: %s", len(data), args.addr, data.hex(" ").upper())
                if args.out:
                    with open(args.out, "wb") as f:
                        f.write(data)
                    log.info("Saved to %s", args.out)

            elif args.cmd == "write":
                if args.data is not None:
                    payload = parse_hex_bytes(args.data)
                else:
                    with open(args.file, "rb") as f:
                        payload = f.read()
                client.write_memory(args.addr, payload, auto_sync=args.sync)
                log.info("WRITE OK: %d bytes @ 0x%08X", len(payload), args.addr)

            elif args.cmd == "erase":
                if args.mode == "mass":
                    client.erase_mass(0xFFFF, auto_sync=args.sync, erase_timeout=args.erase_timeout)
                elif args.mode == "bank1":
                    client.erase_mass(0xFFFE, auto_sync=args.sync, erase_timeout=args.erase_timeout)
                elif args.mode == "bank2":
                    client.erase_mass(0xFFFD, auto_sync=args.sync, erase_timeout=args.erase_timeout)
                elif args.mode == "pages":
                    pages = parse_int_list(args.pages)
                    client.erase_pages_batched(pages, auto_sync=args.sync, erase_timeout=args.erase_timeout)
                log.info("ERASE OK (%s)", args.mode)

            elif args.cmd == "go":
                client.go(args.addr, auto_sync=args.sync)
                log.info("GO OK: 0x%08X", args.addr)

            elif args.cmd == "write-protect":
                sectors = parse_int_list(args.sectors)
                client.write_protect(sectors, auto_sync=args.sync)
                log.info("WRITE PROTECT OK")

            elif args.cmd == "write-unprotect":
                client.write_unprotect(auto_sync=args.sync)
                log.info("WRITE UNPROTECT OK")

            elif args.cmd == "read-protect":
                client.read_protect(auto_sync=args.sync)
                log.info("READ PROTECT OK")

            elif args.cmd == "read-unprotect":
                client.read_unprotect(auto_sync=args.sync)
                log.info("READ UNPROTECT OK")

            elif args.cmd == "upgrade":
                run_upgrade(client, args, log)
                log.info("UPGRADE OK")

            else:
                parser.error(f"unsupported command: {args.cmd}")

        return 0

    except BootloaderError as e:
        log.error("Bootloader protocol error: %s", e)
        return 2
    except can.CanError as e:
        log.error("CAN driver error: %s", e)
        return 3
    except FileNotFoundError as e:
        log.error("File not found: %s", e)
        return 4
    except Exception as e:
        log.exception("Unexpected error: %s", e)
        return 1


if __name__ == "__main__":
    sys.exit(main())
