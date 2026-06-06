# Robot Telemetry MCP Server

This workspace includes a Python MCP server for reading live robot telemetry from NetworkTables (NT4) while running simulation.

## What It Supports (v1)

- `nt_list_topics`: list currently published topics, with optional prefix filtering.
- `nt_get_values`: read latest values for one or more topics.
- `nt_subscribe_stream`: buffered update stream (`start`, `read`, `stop`) for selected topics.

The server is read-only. It does not publish or mutate NT entries.

## Project Paths

- Server root: `tools/mcp_robot`
- Entry point: `tools/mcp_robot/main.py`
- NT4 adapter: `tools/mcp_robot/robot_mcp/nt4_source.py`

## Local Setup

1. Install Python dependencies:
   - `python3 -m pip install -r tools/mcp_robot/requirements.txt`
2. Start robot sim in a separate terminal:
   - `./gradlew simulateJava`
3. Start or reload MCP in Cursor (configured via `.cursor/mcp.json`).

## MCP Configuration

The server is configured to run over stdio from:

- `command`: `python3`
- `args`: `["tools/mcp_robot/main.py"]`

## NT Topic Naming Notes

Your robot code uses AdvantageKit NT publishing in sim, so topic paths generally reflect AdvantageKit-style subsystem paths. Typical paths to expect:

- `Drive/...`
- `Drive/Gyro/...`
- `Vision/...`
- subsystem outputs from `Logger.recordOutput(...)` (for example launcher/intake/trajectory telemetry)

Use `nt_list_topics` first, then query exact keys with `nt_get_values`.

## Environment Variables

Optional environment variables for the MCP process:

- `ROBOT_MCP_SOURCE` (default `nt4`)
  - `wpilog` is reserved for phase 2 and not implemented yet.
- `ROBOT_MCP_NT_HOST` (default `127.0.0.1`)
- `ROBOT_MCP_NT_PORT` (default `5810`)
- `ROBOT_MCP_NT_CLIENT_NAME` (default `robot-telemetry-mcp`)
- `ROBOT_MCP_CONNECT_TIMEOUT_SEC` (default `1.0`)
- `ROBOT_MCP_RECONNECT_BACKOFF_SEC` (default `1.0`)

## Troubleshooting

- No topics returned:
  - Confirm sim is running (`./gradlew simulateJava`).
  - Confirm NT host/port env vars match your target.
  - Check that `Constants.currentMode` is `SIM` and `NT4Publisher` is active in `Robot.java`.
- Topics exist but values are null:
  - Some keys may be announced before first publish; wait for periodic updates.
  - Verify exact topic string from `nt_list_topics`.
- Connection drops after restarting sim:
  - Retry tool calls after a second; reconnect/backoff is built in.
  - If needed, restart the MCP server process in Cursor.

## Phase 2 Preview

The code already has a telemetry-source abstraction (`TelemetrySource`) and source factory (`source_factory.py`) so an AdvantageKit `.wpilog` reader can be added without changing tool names or payload shape.
