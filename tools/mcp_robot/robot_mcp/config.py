from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class Nt4Config:
    host: str = "127.0.0.1"
    port: int = 5810
    client_name: str = "robot-telemetry-mcp"
    connect_timeout_sec: float = 1.0
    reconnect_backoff_sec: float = 1.0

    @staticmethod
    def from_env() -> "Nt4Config":
        return Nt4Config(
            host=os.getenv("ROBOT_MCP_NT_HOST", "127.0.0.1"),
            port=int(os.getenv("ROBOT_MCP_NT_PORT", "5810")),
            client_name=os.getenv("ROBOT_MCP_NT_CLIENT_NAME", "robot-telemetry-mcp"),
            connect_timeout_sec=float(os.getenv("ROBOT_MCP_CONNECT_TIMEOUT_SEC", "1.0")),
            reconnect_backoff_sec=float(os.getenv("ROBOT_MCP_RECONNECT_BACKOFF_SEC", "1.0")),
        )


def source_kind_from_env() -> str:
    return os.getenv("ROBOT_MCP_SOURCE", "nt4").strip().lower()
