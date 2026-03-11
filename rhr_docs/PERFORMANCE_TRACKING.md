# Performance Tracking (Subsystem Loop Timing)

This codebase includes lightweight timing for selected subsystem `periodic()` methods so we can see where loop time is spent during robot operation.

## What Is Implemented

Performance tracking is implemented with:

- `@TimeLogged("...")` on a subsystem's `periodic()` method.
- `PeriodicTimingLogger.time(this)` wrapped in a `try`-with-resources block inside `periodic()`.

Example pattern:

```java
@Override
@TimeLogged("Performance/SubsystemPeriodic/Drive")
public void periodic() {
  try (var ignored = PeriodicTimingLogger.time(this)) {
    // subsystem periodic work
  }
}
```

### How it works internally

- `TimeLogged` is a runtime annotation on methods.
- `PeriodicTimingLogger` checks (via reflection) whether the class's `periodic()` method has `@TimeLogged`.
- Reflection metadata is cached with `ClassValue`, so lookup happens once per class and is reused.
- If not annotated, `time(this)` returns a no-op scope (near-zero overhead).
- If annotated, timing starts with `System.nanoTime()` and records on scope close.
- Timing is logged to AdvantageKit under the annotation path with these outputs:
  - `lastMs`
  - `avgMs`
  - `minMs`
  - `maxMs`
  - `ewmaMs` (exponential weighted moving average, alpha = `0.1`)
  - `samples`

## How To Use It

To add timing for a new subsystem/mechanism:

1. Import:
   - `frc2713.lib.logging.TimeLogged`
   - `frc2713.lib.logging.PeriodicTimingLogger`
2. Annotate `periodic()` with a unique path, usually:
   - `Performance/SubsystemPeriodic/<SubsystemName>`
3. Wrap the `periodic()` body:
   - `try (var ignored = PeriodicTimingLogger.time(this)) { ... }`
4. Deploy/run and open AdvantageScope (or review log files), then inspect the chosen path.

Recommended naming:

- Keep everything under `Performance/SubsystemPeriodic/` for consistency.
- Use subsystem/mechanism class name as the final path segment.

Notes:

- Trailing `/` in the annotation path is sanitized away.
- Stats are in-process and reset when robot code restarts.

## What It Is Good For

This tracking is useful for:

- Finding which subsystem `periodic()` methods are expensive.
- Detecting intermittent spikes (`lastMs` and `maxMs`) vs steady load (`ewmaMs`/`avgMs`).
- Catching regressions after code changes by comparing logs across runs.
- Prioritizing optimization work where it affects the 20ms loop budget the most.
- Supporting evidence-based tuning/debugging instead of guesswork.

## Currently Instrumented Components

As of this branch, timing is enabled for:

- `Drive`
- `Vision`
- `KinematicsManager`
- `LaunchingSolutionManager`
- `Flywheels`
- `Hood`
- `Turret`
- `IntakeExtension`
- `IntakeRoller`
- `DyeRotor`
- `Feeder`
