from __future__ import annotations

import base64
import time
import uuid
from dataclasses import dataclass
from threading import RLock
from typing import Any

from robot_mcp.config import Nt4Config
from robot_mcp.telemetry_source import StreamUpdate, TelemetrySource, TopicInfo, TopicValue

try:
    from ntcore import MultiSubscriber, NetworkTableInstance, PubSubOptions
except ImportError as exc:  # pragma: no cover - exercised at runtime
    raise RuntimeError(
        "ntcore is not installed. Install dependencies with: "
        "python3 -m pip install -r tools/mcp_robot/requirements.txt"
    ) from exc


@dataclass
class _StreamState:
    stream_id: str
    topics: list[str]
    subscribers: dict[str, Any]


class NT4TelemetrySource(TelemetrySource):
    def __init__(self, config: Nt4Config):
        self._config = config
        self._inst = NetworkTableInstance.create()
        self._streams: dict[str, _StreamState] = {}
        self._topic_discovery_subscriber: Any | None = None
        self._lock = RLock()
        self._started = False
        self._last_connect_attempt = 0.0

    def _start_client_if_needed(self) -> None:
        if self._started:
            return
        self._inst.startClient4(self._config.client_name)
        self._inst.setServer(self._config.host, self._config.port)
        self._started = True

    def _attempt_reconnect(self) -> None:
        with self._lock:
            self._start_client_if_needed()
            if self._inst.isConnected():
                return
            now = time.monotonic()
            if now - self._last_connect_attempt < self._config.reconnect_backoff_sec:
                return
            self._last_connect_attempt = now
            self._inst.disconnect()

    def _wait_for_connection(self, timeout_sec: float) -> bool:
        if self._inst.isConnected():
            return True
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while time.monotonic() < deadline:
            if self._inst.isConnected():
                return True
            time.sleep(0.02)
        return self._inst.isConnected()

    def _ensure_topic_discovery_subscription(self) -> None:
        if self._topic_discovery_subscriber is not None:
            return
        with self._lock:
            if self._topic_discovery_subscriber is None:
                # NT4 sends topic/value traffic based on subscriptions. Keep
                # a root-prefix subscriber so list_topics() can see published topics.
                self._topic_discovery_subscriber = MultiSubscriber(
                    self._inst, ["/"], PubSubOptions(sendAll=True)
                )

    def _get_topics_with_discovery(self, prefix: str) -> list[Any]:
        topics = self._inst.getTopics(prefix)
        if topics:
            return topics
        deadline = time.monotonic() + max(self._config.connect_timeout_sec, 0.0)
        while time.monotonic() < deadline:
            topics = self._inst.getTopics(prefix)
            if topics:
                return topics
            time.sleep(0.02)
        return topics

    def connection_state(self) -> dict[str, Any]:
        self._attempt_reconnect()
        self._wait_for_connection(self._config.connect_timeout_sec)
        return {
            "connected": self._inst.isConnected(),
            "host": self._config.host,
            "port": self._config.port,
            "clientName": self._config.client_name,
        }

    def list_topics(self, prefix: str = "") -> list[TopicInfo]:
        self._attempt_reconnect()
        self._wait_for_connection(self._config.connect_timeout_sec)
        self._ensure_topic_discovery_subscription()
        topics = self._get_topics_with_discovery(prefix if prefix else "")
        result = [
            TopicInfo(name=topic.getName(), type=topic.getTypeString(), exists=topic.exists())
            for topic in topics
        ]
        result.sort(key=lambda topic: topic.name)
        return result

    def get_values(self, topics: list[str]) -> list[TopicValue]:
        self._attempt_reconnect()
        self._wait_for_connection(self._config.connect_timeout_sec)
        values: list[TopicValue] = []
        for topic_name in topics:
            topic = self._inst.getTopic(topic_name)
            entry = self._inst.getEntry(topic_name)
            nt_value = entry.getValue()
            value = self._serialize_value(nt_value) if nt_value.isValid() else None
            values.append(
                TopicValue(
                    name=topic_name,
                    type=topic.getTypeString(),
                    exists=topic.exists() or entry.exists(),
                    timestamp_us=self._value_timestamp(nt_value) if nt_value.isValid() else None,
                    value=value,
                )
            )
        return values

    def open_stream(
        self,
        topics: list[str],
        sample_period_ms: int = 100,
        queue_size: int = 256,
    ) -> str:
        self._attempt_reconnect()
        period_sec = max(sample_period_ms, 10) / 1000.0
        poll_storage = max(queue_size, 1)
        stream_id = uuid.uuid4().hex[:12]

        subscribers: dict[str, Any] = {}
        for topic_name in topics:
            topic = self._inst.getTopic(topic_name)
            subscribers[topic_name] = topic.genericSubscribe(
                PubSubOptions(pollStorage=poll_storage, periodic=period_sec, sendAll=True)
            )

        with self._lock:
            self._streams[stream_id] = _StreamState(
                stream_id=stream_id,
                topics=topics,
                subscribers=subscribers,
            )
        return stream_id

    def read_stream(self, stream_id: str, max_updates: int = 100) -> list[StreamUpdate]:
        self._attempt_reconnect()
        with self._lock:
            stream = self._streams.get(stream_id)
        if stream is None:
            raise KeyError(f"Unknown stream_id '{stream_id}'")

        updates: list[StreamUpdate] = []
        for topic_name, subscriber in stream.subscribers.items():
            for value in subscriber.readQueue():
                updates.append(
                    StreamUpdate(
                        stream_id=stream_id,
                        topic=topic_name,
                        type=self._inst.getTopic(topic_name).getTypeString(),
                        timestamp_us=self._value_timestamp(value),
                        value=self._serialize_value(value),
                    )
                )

        updates.sort(key=lambda update: update.timestamp_us or -1)
        if max_updates > 0:
            updates = updates[-max_updates:]
        return updates

    def close_stream(self, stream_id: str) -> bool:
        with self._lock:
            stream = self._streams.pop(stream_id, None)
        if stream is None:
            return False
        for subscriber in stream.subscribers.values():
            self._close_if_supported(subscriber)
        return True

    @staticmethod
    def _close_if_supported(obj: Any) -> None:
        close = getattr(obj, "close", None)
        if callable(close):
            close()

    @staticmethod
    def _value_timestamp(value: Any) -> int | None:
        for candidate in ("time", "last_change", "server_time"):
            attr = getattr(value, candidate, None)
            if callable(attr):
                try:
                    return int(attr())
                except Exception:
                    continue
        return None

    @staticmethod
    def _serialize_value(value: Any) -> Any:
        if value.isBoolean():
            return value.getBoolean()
        if value.isDouble():
            return value.getDouble()
        if value.isFloat():
            return value.getFloat()
        if value.isInteger():
            return value.getInteger()
        if value.isString():
            return value.getString()
        if value.isBooleanArray():
            return [bool(v) for v in value.getBooleanArray()]
        if value.isDoubleArray():
            return [float(v) for v in value.getDoubleArray()]
        if value.isFloatArray():
            return [float(v) for v in value.getFloatArray()]
        if value.isIntegerArray():
            return [int(v) for v in value.getIntegerArray()]
        if value.isStringArray():
            return [str(v) for v in value.getStringArray()]
        if value.isRaw():
            raw = bytes(value.getRaw())
            return {"base64": base64.b64encode(raw).decode("ascii"), "byteCount": len(raw)}

        fallback = value.value()
        if isinstance(fallback, memoryview):
            fallback = fallback.tobytes()
        if isinstance(fallback, (bytes, bytearray)):
            raw = bytes(fallback)
            return {"base64": base64.b64encode(raw).decode("ascii"), "byteCount": len(raw)}
        return fallback
