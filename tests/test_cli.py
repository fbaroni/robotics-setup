"""End-to-end through the CLI, as a user would run it."""

from __future__ import annotations

import pytest

from robot_agent.cli import main


def test_greedy_demo_reaches_the_ball(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["go to the red ball"]) == 0
    out = capsys.readouterr().out
    assert "END: finished - reached red_ball at (2,2)" in out


def test_scripted_demo_shows_gate_blocks(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--planner", "scripted"]) == 1  # script never declares "reached"
    out = capsys.readouterr().out
    assert "BLOCK - wall or boundary east" in out
    assert "BLOCK - unknown tool 'fly'" in out


def test_sensor_failure_blocks_every_move(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--fail-sensor", "go to the blue box"]) == 1
    out = capsys.readouterr().out
    assert "sensor unavailable" in out
    assert "robot now" not in out
