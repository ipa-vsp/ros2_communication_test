#!/usr/bin/env python3
"""
Shared helpers for the ROS 2 QoS matrix benchmark tooling.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Dict, Iterable, List

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


RELIABILITY_MAP = {
    "reliable": ReliabilityPolicy.RELIABLE,
    "besteffort": ReliabilityPolicy.BEST_EFFORT,
}

RELIABILITY_CANONICAL = {
    "reliable": "reliable",
    "reliable_qos": "reliable",
    "best_effort": "besteffort",
    "besteffort": "besteffort",
}

DURABILITY_MAP = {
    "volatile": DurabilityPolicy.VOLATILE,
    "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
}

DURABILITY_CANONICAL = {
    "volatile": "volatile",
    "volatile_qos": "volatile",
    "transient_local": "transient_local",
    "transientlocal": "transient_local",
}


@dataclass(frozen=True)
class QoSConfig:
    name: str
    reliability: str
    durability: str
    depth: int = 10
    metadata: Dict[str, str] = field(default_factory=dict)

    def to_profile(self) -> QoSProfile:
        reliability = canonical_reliability(self.reliability)
        durability = canonical_durability(self.durability)
        return QoSProfile(
            depth=self.depth,
            history=HistoryPolicy.KEEP_LAST,
            reliability=RELIABILITY_MAP[reliability],
            durability=DURABILITY_MAP[durability],
        )

    def as_dict(self) -> Dict[str, str]:
        return {
            "name": self.name,
            "reliability": canonical_reliability(self.reliability),
            "durability": canonical_durability(self.durability),
            "depth": int(self.depth),
            "metadata": dict(self.metadata),
        }


def canonical_reliability(value: str) -> str:
    key = value.strip().lower()
    if key not in RELIABILITY_CANONICAL:
        raise ValueError(f"Unsupported reliability policy: {value}")
    return RELIABILITY_CANONICAL[key]


def canonical_durability(value: str) -> str:
    key = value.strip().lower()
    if key not in DURABILITY_CANONICAL:
        raise ValueError(f"Unsupported durability policy: {value}")
    return DURABILITY_CANONICAL[key]


def parse_profiles(raw: str) -> List[QoSConfig]:
    """
    Parse a comma separated list of profile descriptors.
    Each descriptor can have the form NAME or NAME=RELIABILITY:DURABILITY[:DEPTH].
    If only NAME is given, reliability and durability are inferred from the suffix
    (e.g. humble-fast-reliable-volatile).
    """
    profiles: List[QoSConfig] = []
    for item in (p.strip() for p in raw.split(",") if p.strip()):
        if "=" in item:
            name, spec = item.split("=", 1)
            pieces = spec.split(":")
            if len(pieces) not in (2, 3):
                raise ValueError(f"Invalid profile spec '{item}'. Expected reliability:durability[:depth].")
            reliability = canonical_reliability(pieces[0])
            durability = canonical_durability(pieces[1])
            depth = int(pieces[2]) if len(pieces) == 3 else 10
        else:
            parts = item.split("-")
            if len(parts) < 4:
                raise ValueError(f"Cannot infer reliability/durability from '{item}'. Use name=rel:dur form.")
            reliability = canonical_reliability(parts[-2])
            durability = canonical_durability(parts[-1])
            depth = 10
            name = item
        profiles.append(QoSConfig(name=name, reliability=reliability, durability=durability, depth=depth))
    return profiles


def encode_profile(config: QoSConfig) -> str:
    return json.dumps(config.as_dict())


def decode_profile(payload: str) -> QoSConfig:
    data = json.loads(payload)
    return QoSConfig(
        name=data.get("name", "unknown"),
        reliability=canonical_reliability(data["reliability"]),
        durability=canonical_durability(data["durability"]),
        depth=int(data.get("depth", 10)),
        metadata=dict(data.get("metadata", {})),
    )


def worst_status(statuses: Iterable[str]) -> str:
    severity = {"pass": 0, "caution": 1, "failed": 2}
    score = 0
    result = "pass"
    for status in statuses:
        normalized = status.lower()
        if normalized not in severity:
            raise ValueError(f"Unknown status '{status}'")
        if severity[normalized] >= score:
            score = severity[normalized]
            result = normalized
    return result


def split_items(raw: str) -> List[str]:
    tokens: List[str] = []
    for chunk in raw.replace(",", " ").split():
        token = chunk.strip()
        if token:
            tokens.append(token)
    return tokens


def generate_profiles(
    distros: Iterable[str],
    rmws: Iterable[str],
    reliability: str,
    durability: str,
    depth: int = 10,
) -> List[QoSConfig]:
    rel = canonical_reliability(reliability)
    dur = canonical_durability(durability)
    profiles: List[QoSConfig] = []
    for distro in distros:
        distro_clean = distro.strip()
        if not distro_clean:
            continue
        for rmw in rmws:
            rmw_clean = rmw.strip()
            if not rmw_clean:
                continue
            name = f"{distro_clean}-{rmw_clean}"
            metadata = {
                "ros_distro": distro_clean,
                "rmw": rmw_clean,
                "reliability": rel,
                "durability": dur,
            }
            profiles.append(
                QoSConfig(name=name, reliability=rel, durability=dur, depth=depth, metadata=metadata)
            )
    return profiles
