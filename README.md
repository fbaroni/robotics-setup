# robot-agent

A small, fully typed skeleton for a robotic agent built on one rule:
**the planner (LLM) proposes, a deterministic gate decides, and only approved actions reach the robot.**

Everything is simulated on a 5x5 text grid. Pure Python 3.10+, zero runtime dependencies,
runs on Windows, Linux and macOS.

```
$ robot-agent "go to the red ball"

@ . # . .
. . # . .
. . R . .
. # . . .
. . . . B

GOAL: go to the red ball
[step 01] propose: look()
          gate:    APPROVE - all rules passed
          result:  robot@(0,0); sees blue_box@(4,4), red_ball@(2,2)
[step 02] propose: move(direction=south)
          gate:    APPROVE - all rules passed
          result:  robot now @(1,0)
[step 03] propose: move(direction=east)
          gate:    APPROVE - all rules passed
          result:  robot now @(1,1)
[step 04] propose: move(direction=south)
          gate:    APPROVE - all rules passed
          result:  robot now @(2,1)
[step 05] propose: move(direction=east)
          gate:    APPROVE - all rules passed
          result:  robot now @(2,2)
END: finished - reached red_ball at (2,2) (5 executed, 0 blocked)
```

And when the planner is wrong, the gate says why:

```
$ robot-agent --planner scripted
...
[step 04] propose: move(direction=east)
          gate:    BLOCK - wall or boundary east (clearance 0)
[step 05] propose: fly(height=3)
          gate:    BLOCK - unknown tool 'fly'; allowed: ['look', 'move', 'read_sensor']
```

## Quick start

```bash
pip install -e .[dev]
robot-agent                              # greedy planner, default goal
robot-agent "go to the blue box"         # any goal naming a visible object
robot-agent --planner scripted           # a deliberately flawed plan: watch the gate block it
robot-agent --fail-sensor "red ball"     # sensor times out: every move is refused (fail-closed)
robot-agent --max-steps 3                # step budget: the gate halts the run
pytest
```

Without installing: `PYTHONPATH=src python -m robot_agent` (PowerShell: `$env:PYTHONPATH="src"`).

## Architecture

```
          ┌──────────────────────────── history: StepRecord(proposal, decision, result|error) ─────┐
          │                                                                                         │
 goal ──► Planner.propose() ──► ToolCall ──► SafetyGate.check(call, GateContext) ──┬─ BLOCK ──► trace ─┤
          (LLM / Greedy / Scripted)                                                 │   (terminal? stop) │
                                                                                    └─ APPROVE ──► Toolbox.execute() ──► Robot (HAL)
                                                                                                        │
                                                                                               observation | HardwareError
```

| Module | Responsibility |
|---|---|
| `models.py` | Immutable value types: `Direction`, `Position`, `ToolCall`, `Finish`, observations, `GateDecision`, `StepRecord` |
| `hal.py` | `Robot` protocol and `HardwareError` family. **The only thing to reimplement for real hardware.** |
| `world.py` | `GridWorld`: in-memory grid with walls, objects and the robot |
| `sim.py` | `SimRobot`: `Robot` over a `GridWorld`, with fault injection (`sensor_timeout=True`) |
| `tools.py` | `ToolSpec` (typed params, validation and coercion, JSON schema for LLM tool-use) and `Toolbox` |
| `gate.py` | `SafetyGate`: pure function of (call, context); ordered rules; fail-closed |
| `planner.py` | `Planner` protocol, `ScriptedPlanner` (fixed list), `GreedyPlanner` (navigates from observations) |
| `agent.py` | The loop, and `AgentResult` |
| `trace.py` | `PrintTracer`: per step, what was proposed, the verdict, the reason, the result |
| `cli.py` | `robot-agent` command |

### Design rules

- **The planner never touches the robot.** It only returns `ToolCall | Finish`. Its output is untrusted.
- **The gate is pure.** `SafetyGate.check(call, ctx)` does no I/O and keeps no state; the agent loop builds a
  `GateContext` snapshot (steps taken, fresh sensor clearance) before each check. This makes every rule unit-testable.
- **Fail-closed.** Unknown clearance → BLOCK. Exception inside a rule → terminal BLOCK. The simulator also refuses
  collisions on its own, as defense in depth.
- **Hardware errors are data, not crashes.** A `HardwareError` raised by an approved call is recorded in
  `StepRecord.error` and fed back to the planner on the next turn.
- **Planners are stateless.** `GreedyPlanner` recovers position, visited cells and blocked directions from the
  history on every call, the same way an LLM re-reads its context. Nothing to reset between runs.

### Gate rules (first BLOCK wins)

| # | Rule | On failure |
|---|---|---|
| 1 | `steps_taken < max_steps` (default 20) | terminal BLOCK: the run halts |
| 2 | The tool exists | BLOCK |
| 3 | Args match the `ToolSpec` (no missing, no extra, right type) | BLOCK |
| 4 | `move` only: fresh sensor clearance in that direction is > 0 | BLOCK (also when the sensor is unavailable) |

### Outcomes

`AgentResult.outcome` is one of `FINISHED` (the planner returned `Finish`), `HALTED` (a terminal gate block, e.g. the
step limit) or `PLANNER_ERROR` (the planner raised). `result.reason` says why; `result.steps` holds the full trace.

## Tests

`pytest` runs 41 tests. The three failure scenarios the skeleton exists for live in `tests/test_failures.py`:

| Scenario | What must happen |
|---|---|
| Planner proposes a tool that does not exist | Gate blocks it, nothing runs, the loop continues |
| `read_sensor()` times out | Error recorded, no crash; the next `move` is blocked because clearance is unknown |
| Planner loops (`look` forever) | Step 21 is a terminal block: `HALTED`, "step limit reached" |

Also covered: every gate rule in isolation, `ToolSpec` validation and schema, grid geometry, the greedy planner
(reaches the target, routes around a wall it bumps into, gives up when boxed in or when the target is not visible)
and the CLI end to end.

## Extending

**Add a tool.** Declare a `ToolSpec` in `tools.py`, bind it to a `Robot` method in `Toolbox.__init__`, add the method
to the `Robot` protocol and to `SimRobot`. The gate validates its args automatically from the spec.

**Add a gate rule.** Write a method `_rule_<name>(self, call, ctx) -> GateDecision | None` in `SafetyGate` and
append it to `self._rules`. Return `None` to pass, `GateDecision.block(reason)` to refuse, `terminal=True` to stop
the run. If the rule needs new world data, add a field to `GateContext` and fill it in `Agent._step`.

**Plug in a real LLM.** Implement `Planner`:

```python
class LLMPlanner:
    def __init__(self, client, specs: Mapping[str, ToolSpec]) -> None:
        self._client = client
        self._tools = [spec.json_schema() for spec in specs.values()]

    def propose(self, goal: str, history: Sequence[StepRecord]) -> Action:
        response = self._client.respond(goal, self._tools, history)  # your SDK call here
        if response.tool_name is None:
            return Finish(response.text)
        return ToolCall(response.tool_name, response.tool_args)
```

`history` carries every proposal, decision, reason and result, so the model sees exactly why a move was refused.
The gate still validates everything it returns.

**Move to real hardware.** Implement `hal.Robot` (`look`, `move`, `read_sensor`) for your platform, raising
`HardwareError` subclasses on failure (wrap a serial read with a timeout that raises `SensorTimeoutError`).
Pass it to `build_toolbox()` and `Agent`. The loop, the gate and the planners do not change.

## Development

```bash
ruff check . && ruff format --check .   # lint + format
mypy                                    # strict typing on src/
pytest -q
```

CI runs the same on Ubuntu and Windows, Python 3.10 and 3.13.
