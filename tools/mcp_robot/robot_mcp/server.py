from __future__ import annotations

from typing import Any, Literal

from fastmcp import FastMCP

from robot_mcp.config import Nt4Config, source_kind_from_env
from robot_mcp.source_factory import TelemetrySourceKind, build_source

mcp = FastMCP(
    name="robot-telemetry",
    instructions=(
        "Read-only telemetry server for FRC robot simulation. "
        "Use nt_list_topics to discover keys, nt_get_values for snapshots, "
        "and nt_subscribe_stream for buffered updates."
    ),
)

_source = build_source(TelemetrySourceKind(source_kind_from_env()), Nt4Config.from_env())


def _asdict_topic_info(topic: Any) -> dict[str, Any]:
    return {"name": topic.name, "type": topic.type, "exists": topic.exists}


def _asdict_topic_value(value: Any) -> dict[str, Any]:
    return {
        "name": value.name,
        "type": value.type,
        "exists": value.exists,
        "timestampUs": value.timestamp_us,
        "value": value.value,
    }


def _asdict_stream_update(update: Any) -> dict[str, Any]:
    return {
        "streamId": update.stream_id,
        "topic": update.topic,
        "type": update.type,
        "timestampUs": update.timestamp_us,
        "value": update.value,
    }


@mcp.tool(name="nt_list_topics")
def nt_list_topics(prefix: str = "", limit: int = 500) -> dict[str, Any]:
    """Lists current NT4 topics, optionally filtered by prefix."""
    safe_limit = max(min(limit, 5000), 1)
    topics = _source.list_topics(prefix=prefix)
    return {
        "connection": _source.connection_state(),
        "prefix": prefix,
        "count": min(len(topics), safe_limit),
        "topics": [_asdict_topic_info(topic) for topic in topics[:safe_limit]],
    }


@mcp.tool(name="nt_get_values")
def nt_get_values(topics: list[str]) -> dict[str, Any]:
    """Reads the latest values for one or more NT4 topics."""
    normalized_topics = [topic for topic in dict.fromkeys(topics) if topic.strip()]
    if not normalized_topics:
        raise ValueError("Provide at least one topic path.")

    values = _source.get_values(normalized_topics)
    return {
        "connection": _source.connection_state(),
        "count": len(values),
        "values": [_asdict_topic_value(value) for value in values],
    }


@mcp.tool(name="nt_subscribe_stream")
def nt_subscribe_stream(
    action: Literal["start", "read", "stop"],
    stream_id: str | None = None,
    topics: list[str] | None = None,
    sample_period_ms: int = 100,
    queue_size: int = 256,
    max_updates: int = 100,
) -> dict[str, Any]:
    """
    Manages buffered topic streaming.

    start: requires topics, returns streamId
    read: requires streamId, returns queued updates
    stop: requires streamId, closes the stream
    """
    normalized_action = action.lower()
    if normalized_action == "start":
        normalized_topics = [topic for topic in dict.fromkeys(topics or []) if topic.strip()]
        if not normalized_topics:
            raise ValueError("topics is required for action='start'.")
        stream = _source.open_stream(
            topics=normalized_topics,
            sample_period_ms=sample_period_ms,
            queue_size=queue_size,
        )
        return {
            "connection": _source.connection_state(),
            "action": "start",
            "streamId": stream,
            "topics": normalized_topics,
            "samplePeriodMs": max(sample_period_ms, 10),
            "queueSize": max(queue_size, 1),
        }

    if normalized_action == "read":
        if not stream_id:
            raise ValueError("stream_id is required for action='read'.")
        updates = _source.read_stream(stream_id=stream_id, max_updates=max_updates)
        return {
            "connection": _source.connection_state(),
            "action": "read",
            "streamId": stream_id,
            "count": len(updates),
            "updates": [_asdict_stream_update(update) for update in updates],
        }

    if normalized_action == "stop":
        if not stream_id:
            raise ValueError("stream_id is required for action='stop'.")
        closed = _source.close_stream(stream_id=stream_id)
        return {
            "connection": _source.connection_state(),
            "action": "stop",
            "streamId": stream_id,
            "closed": closed,
        }

    raise ValueError("action must be one of: start, read, stop.")


def run() -> None:
    mcp.run()
