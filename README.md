# robot-agent

MVP skeleton of a robotic agent: **the LLM proposes, the gate executes**.
Fully simulated (5x5 text grid), pure Python, no dependencies, runs on Windows/Linux/macOS.

## Run

```bash
pip install -e .[dev]        # or just: pip install pytest
python -m robot_agent        # demo: "go to the red ball"
pytest                       # tests
```

Without installing, on Windows PowerShell: `$env:PYTHONPATH="src"; python -m robot_agent`.

## Architecture

```
goal ─► Planner.propose() ─► ToolCall ─► SafetyGate.check() ─┬─ BLOCK ─► trace (+ stop if terminal)
            ▲                                                └─ APPROVE ─► Toolbox.execute() ─► Robot (HAL)
            └──────────────── history (StepRecord: proposal, decision, result/error) ◄──────────┘
```

| Module | Role |
|---|---|
| `models.py` | Immutable value types: `Direction`, `Position`, `ToolCall`, `Finish`, `GateDecision`, `StepRecord`, observations |
| `hal.py` | `Robot` protocol + `HardwareError` / `SensorTimeoutError` / `CollisionError`. **Hardware swap point.** |
| `world.py` | `GridWorld`: in-memory 5x5 grid (walls, objects, robot) |
| `sim.py` | `SimRobot`: `Robot` implementation over `GridWorld`, with fault injection (`sensor_timeout`) |
| `tools.py` | `ToolSpec` (typed params, validation/coercion, JSON schema) + `Toolbox` (binds specs to a `Robot`) |
| `gate.py` | `SafetyGate`: pure, deterministic, ordered rules, fail-closed |
| `planner.py` | `Planner` protocol + `ScriptedPlanner` (mock LLM) |
| `agent.py` | The loop |
| `trace.py` | `PrintTracer`: per step, what was proposed, gate verdict and reason, result or error |

### Gate rules (first BLOCK wins)

1. **Step limit** — `steps_taken >= max_steps` (default 20) → terminal BLOCK, loop stops.
2. **Tool exists** — unknown tool name → BLOCK.
3. **Args valid** — missing/extra/wrongly typed args → BLOCK.
4. **Move is clear** — fresh sensor clearance in that direction must be > 0 (wall or border → BLOCK).
   If the sensor fails, clearance is unknown → BLOCK (fail-closed).

Any exception inside the gate → terminal BLOCK.

### Failure handling

- Hardware errors during an approved tool call are caught and recorded in the step (`error`), fed back to the planner; the loop does not crash.
- `SimRobot.move` also refuses collisions (defense in depth), even though the gate should make that unreachable.

## Plugging in a real LLM

Implement `Planner.propose(goal, history) -> ToolCall | Finish`:
send the goal, `[spec.json_schema() for spec in toolbox.specs.values()]` and the
history to the model, map its tool-use response to `ToolCall`, or return `Finish`
when it stops calling tools. Its output is untrusted: the gate validates everything.

## Moving to real hardware

Write a class implementing `hal.Robot` (`look`, `move`, `read_sensor`), raising
`HardwareError` subclasses on failures (e.g. wrap serial reads with a timeout that
raises `SensorTimeoutError`). Pass it to `build_toolbox()` and `Agent`. The loop and
the gate stay untouched.
