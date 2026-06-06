from __future__ import annotations

from enum import Enum

from robot_mcp.config import Nt4Config
from robot_mcp.nt4_source import NT4TelemetrySource
from robot_mcp.telemetry_source import TelemetrySource


class TelemetrySourceKind(str, Enum):
    NT4 = "nt4"
    WPILOG = "wpilog"


def build_source(kind: TelemetrySourceKind, nt4_config: Nt4Config) -> TelemetrySource:
    if kind is TelemetrySourceKind.NT4:
        return NT4TelemetrySource(nt4_config)
    if kind is TelemetrySourceKind.WPILOG:
        raise NotImplementedError("WPILOG telemetry source will be added in phase 2.")
    raise ValueError(f"Unsupported telemetry source kind: {kind}")
