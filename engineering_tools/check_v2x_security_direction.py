#!/usr/bin/env python3
"""
Analyze classic PCAP captures for common SAE J2735 V2X messages and determine
whether they are carried in IEEE 1609.2 signedData or unsecuredData envelopes.

Supports:
  - Ethernet captures (DLT_EN10MB / linktype 1)
  - Linux cooked captures v1 (DLT_LINUX_SLL / linktype 113)
  - IPv4 and IPv6 UDP
  - MAP, SPaT, BSM, SDSM, PSM, TIM, SRM, and SSM

Direction handling for Linux cooked captures:
  0 = incoming to this host
  1 = incoming broadcast
  2 = incoming multicast
  3 = incoming for another host
  4 = outgoing from this host

Note:
  Detecting signedData does not cryptographically validate the signature.
"""

from __future__ import annotations

import argparse
import ipaddress
import struct
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional


DLT_EN10MB = 1
DLT_LINUX_SLL = 113

ETHERTYPE_IPV4 = 0x0800
ETHERTYPE_IPV6 = 0x86DD
ETHERTYPE_VLAN = 0x8100

J2735_MESSAGE_NAMES = {
    18: "MAP",
    19: "SPAT",
    20: "BSM",
    27: "SRM",
    28: "SSM",
    29: "TIM",
    30: "SSM",
    31: "TIM",
    32: "PSM",
    41: "SDSM",
    # CARMA Mobility messages (PSID 0xBFEE), per wave.json in v2x-ros-driver
    240: "MobilityRequest",
    241: "MobilityResponse",
    242: "MobilityPath",
    243: "MobilityOperation",
}


@dataclass
class PcapPacket:
    number: int
    data: bytes
    linktype: int


@dataclass
class UdpPacket:
    family: str
    source_ip: str
    destination_ip: str
    source_port: int
    destination_port: int
    payload: bytes
    direction: str


@dataclass
class V2xResult:
    security: str
    message_name: Optional[str]
    message_id: Optional[int]


def read_classic_pcap(path: Path) -> Iterator[PcapPacket]:
    with path.open("rb") as stream:
        global_header = stream.read(24)

        if len(global_header) != 24:
            raise ValueError("File is too short to be a valid classic PCAP.")

        magic = global_header[:4]

        if magic in (b"\xd4\xc3\xb2\xa1", b"\x4d\x3c\xb2\xa1"):
            endian = "<"
        elif magic in (b"\xa1\xb2\xc3\xd4", b"\xa1\xb2\x3c\x4d"):
            endian = ">"
        elif magic == b"\x0a\x0d\x0d\x0a":
            raise ValueError(
                "PCAPNG is not supported. Convert it first with:\n"
                "  editcap input.pcapng output.pcap"
            )
        else:
            raise ValueError(f"Unsupported PCAP magic number: {magic.hex()}")

        linktype = struct.unpack(endian + "I", global_header[20:24])[0]
        packet_number = 0

        while True:
            packet_header = stream.read(16)

            if not packet_header:
                break

            if len(packet_header) != 16:
                raise ValueError("Truncated PCAP packet header.")

            _, _, captured_length, _ = struct.unpack(
                endian + "IIII", packet_header
            )

            packet_data = stream.read(captured_length)

            if len(packet_data) != captured_length:
                raise ValueError(
                    f"Packet {packet_number + 1} is truncated."
                )

            packet_number += 1
            yield PcapPacket(packet_number, packet_data, linktype)


def parse_link_layer(packet: PcapPacket):
    data = packet.data

    if packet.linktype == DLT_EN10MB:
        if len(data) < 14:
            return None

        ethertype = struct.unpack("!H", data[12:14])[0]
        offset = 14
        direction = "unknown"

        if ethertype == ETHERTYPE_VLAN:
            if len(data) < 18:
                return None
            ethertype = struct.unpack("!H", data[16:18])[0]
            offset = 18

        return ethertype, offset, direction

    if packet.linktype == DLT_LINUX_SLL:
        if len(data) < 16:
            return None

        packet_type = struct.unpack("!H", data[0:2])[0]
        ethertype = struct.unpack("!H", data[14:16])[0]

        direction_names = {
            0: "incoming",
            1: "incoming-broadcast",
            2: "incoming-multicast",
            3: "incoming-otherhost",
            4: "outgoing",
        }

        return (
            ethertype,
            16,
            direction_names.get(packet_type, f"sll-type-{packet_type}"),
        )

    return None


def parse_udp(packet: PcapPacket) -> Optional[UdpPacket]:
    parsed = parse_link_layer(packet)

    if parsed is None:
        return None

    ethertype, offset, direction = parsed
    data = packet.data

    if ethertype == ETHERTYPE_IPV4:
        if len(data) < offset + 20:
            return None

        version_ihl = data[offset]
        version = version_ihl >> 4
        ihl = (version_ihl & 0x0F) * 4

        if version != 4 or ihl < 20 or len(data) < offset + ihl:
            return None

        if data[offset + 9] != 17:
            return None

        source_ip = str(ipaddress.IPv4Address(data[offset + 12:offset + 16]))
        destination_ip = str(
            ipaddress.IPv4Address(data[offset + 16:offset + 20])
        )

        udp_offset = offset + ihl
        family = "IPv4"

    elif ethertype == ETHERTYPE_IPV6:
        if len(data) < offset + 40:
            return None

        if data[offset] >> 4 != 6:
            return None

        next_header = data[offset + 6]
        udp_offset = offset + 40

        # Handle common IPv6 extension headers.
        while next_header in (0, 43, 44, 60):
            if len(data) < udp_offset + 8:
                return None

            if next_header == 44:
                next_header = data[udp_offset]
                udp_offset += 8
            else:
                next_header = data[udp_offset]
                header_length = (data[udp_offset + 1] + 1) * 8
                udp_offset += header_length

        if next_header != 17:
            return None

        source_ip = str(ipaddress.IPv6Address(data[offset + 8:offset + 24]))
        destination_ip = str(
            ipaddress.IPv6Address(data[offset + 24:offset + 40])
        )

        family = "IPv6"

    else:
        return None

    if len(data) < udp_offset + 8:
        return None

    source_port, destination_port, udp_length, _ = struct.unpack(
        "!HHHH", data[udp_offset:udp_offset + 8]
    )

    if udp_length < 8:
        return None

    payload_end = min(len(data), udp_offset + udp_length)
    payload = data[udp_offset + 8:payload_end]

    return UdpPacket(
        family=family,
        source_ip=source_ip,
        destination_ip=destination_ip,
        source_port=source_port,
        destination_port=destination_port,
        payload=payload,
        direction=direction,
    )


def decode_oer_length(data: bytes, offset: int):
    if offset >= len(data):
        return None

    first = data[offset]

    if first < 0x80:
        return first, offset + 1

    byte_count = first & 0x7F

    if byte_count == 0 or byte_count > 4:
        return None

    end = offset + 1 + byte_count

    if end > len(data):
        return None

    return int.from_bytes(data[offset + 1:end], "big"), end


def classify_v2x_payload(payload: bytes) -> V2xResult:
    """
    Search for a plausible IEEE 1609.2 unsecuredData container:

        03 80 <OER length> 00 <J2735 message ID>

    If an outer signedData marker (03 81) appears shortly before the nested
    unsecuredData container, classify the packet as signedData.
    """

    candidates = []

    for offset in range(max(0, len(payload) - 4)):
        if payload[offset:offset + 2] != b"\x03\x80":
            continue

        decoded = decode_oer_length(payload, offset + 2)

        if decoded is None:
            continue

        declared_length, content_offset = decoded

        if content_offset + 2 > len(payload):
            continue

        if payload[content_offset] != 0:
            continue

        message_id = payload[content_offset + 1]

        if message_id not in J2735_MESSAGE_NAMES:
            continue

        if declared_length > len(payload) - content_offset:
            continue

        candidates.append((offset, message_id))

    if not candidates:
        return V2xResult("unknown", None, None)

    security_offset, message_id = candidates[0]
    message_name = J2735_MESSAGE_NAMES[message_id]

    signed_marker_found = False
    window_start = max(0, security_offset - 16)

    for position in range(window_start, security_offset):
        if payload[position:position + 2] == b"\x03\x81":
            signed_marker_found = True
            break

    security = "signedData" if signed_marker_found else "unsecuredData"

    return V2xResult(
        security=security,
        message_name=message_name,
        message_id=message_id,
    )


def analyze_file(path: Path) -> int:
    total_packets = 0
    udp_packets = 0
    recognized_packets = 0

    detailed_counts = Counter()
    flow_counts = Counter()

    try:
        for packet in read_classic_pcap(path):
            total_packets += 1

            udp = parse_udp(packet)

            if udp is None:
                continue

            udp_packets += 1

            flow_key = (
                udp.direction,
                udp.family,
                udp.source_ip,
                udp.source_port,
                udp.destination_ip,
                udp.destination_port,
            )
            flow_counts[flow_key] += 1

            result = classify_v2x_payload(udp.payload)

            if result.message_name is None:
                continue

            recognized_packets += 1

            detailed_key = (
                udp.direction,
                result.message_name,
                result.security,
                udp.source_ip,
                udp.source_port,
                udp.destination_ip,
                udp.destination_port,
            )
            detailed_counts[detailed_key] += 1

    except (OSError, ValueError, struct.error) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1

    print(f"\nFile: {path}")
    print(f"Total packets: {total_packets}")
    print(f"UDP packets: {udp_packets}")
    print(f"Recognized J2735 packets: {recognized_packets}")

    if not detailed_counts:
        print("\nNo recognizable J2735 V2X messages were found.")
        return 0

    print("\nDetailed results:")
    print(
        f"{'Direction':<20} {'Message':<8} {'Security':<14} "
        f"{'Source':<28} {'Destination':<28} {'Count':>8}"
    )
    print("-" * 118)

    for key, count in sorted(detailed_counts.items()):
        (
            direction,
            message_name,
            security,
            source_ip,
            source_port,
            destination_ip,
            destination_port,
        ) = key

        source = f"{source_ip}:{source_port}"
        destination = f"{destination_ip}:{destination_port}"

        print(
            f"{direction:<20} {message_name:<8} {security:<14} "
            f"{source:<28} {destination:<28} {count:>8}"
        )

    totals_by_message = defaultdict(Counter)
    totals_by_direction = defaultdict(Counter)

    for key, count in detailed_counts.items():
        direction, message_name, security, *_ = key
        totals_by_message[message_name][security] += count
        totals_by_direction[direction][(message_name, security)] += count

    print("\nTotals by message type:")
    print(f"{'Message':<8} {'Signed':>10} {'Unsecured':>12} {'Total':>10}")
    print("-" * 44)

    for message_name in sorted(totals_by_message):
        signed = totals_by_message[message_name]["signedData"]
        unsecured = totals_by_message[message_name]["unsecuredData"]

        print(
            f"{message_name:<8} {signed:>10} {unsecured:>12} "
            f"{signed + unsecured:>10}"
        )

    print("\nTotals by direction:")
    for direction in sorted(totals_by_direction):
        print(f"  {direction}:")
        for (message_name, security), count in sorted(
            totals_by_direction[direction].items()
        ):
            print(f"    {message_name:<8} {security:<14} {count}")

    print("\nNotes:")
    print("  signedData means an IEEE 1609.2 signed envelope was detected.")
    print("  unsecuredData means an IEEE 1609.2 unsecured envelope was detected.")
    print("  Signatures are not cryptographically validated.")
    print("  Ethernet PCAP direction may appear as 'unknown' because classic")
    print("  Ethernet captures do not always store packet direction.")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Identify J2735 message types, IEEE 1609.2 security envelopes, "
            "and packet direction in classic PCAP files."
        )
    )
    parser.add_argument(
        "pcaps",
        nargs="+",
        type=Path,
        help="One or more classic PCAP files",
    )

    args = parser.parse_args()
    exit_code = 0

    for path in args.pcaps:
        if not path.exists():
            print(f"ERROR: File not found: {path}", file=sys.stderr)
            exit_code = 1
            continue

        exit_code = max(exit_code, analyze_file(path))

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
