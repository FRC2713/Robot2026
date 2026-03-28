from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class TopicInfo:
    name: str
    type: str
    exists: bool


@dataclass(frozen=True)
class TopicValue:
    name: str
    type: str
    exists: bool
    timestamp_us: int | None
    value: Any


@dataclass(frozen=True)
class StreamUpdate:
    stream_id: str
    topic: str
    type: str
    timestamp_us: int | None
    value: Any


class TelemetrySource(ABC):
    @abstractmethod
    def connection_state(self) -> dict[str, Any]:
        """Returns source connection metadata."""

    @abstractmethod
    def list_topics(self, prefix: str = "") -> list[TopicInfo]:
        """Lists available topics."""

    @abstractmethod
    def get_values(self, topics: list[str]) -> list[TopicValue]:
        """Gets current values for topics."""

    @abstractmethod
    def open_stream(
        self,
        topics: list[str],
        sample_period_ms: int = 100,
        queue_size: int = 256,
    ) -> str:
        """Starts collecting change updates for a topic set."""

    @abstractmethod
    def read_stream(self, stream_id: str, max_updates: int = 100) -> list[StreamUpdate]:
        """Reads buffered updates for an open stream."""

    @abstractmethod
    def close_stream(self, stream_id: str) -> bool:
        """Closes an existing stream."""
