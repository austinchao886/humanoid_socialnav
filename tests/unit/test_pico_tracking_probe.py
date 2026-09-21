import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location(
    "pico_tracking_probe", Path(__file__).resolve().parents[2] / "tools/analysis/pico_tracking_probe.py"
)
probe_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe_module)


class Clock:
    now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class SDK:
    def __init__(self, clock, freeze=False, invalid=False):
        self.clock, self.freeze, self.invalid = clock, freeze, invalid
        self.closed = False

    def init(self):
        pass

    def close(self):
        self.closed = True

    def is_body_data_available(self):
        return True

    def get_time_stamp_ns(self):
        return 1 if self.freeze else 1 + int(self.clock() * 50) * 20_000_000

    def get_body_joints_pose(self):
        body = np.zeros((24, 7))
        body[:, 6] = 1
        if self.invalid:
            body[0, 0] = np.nan
        return body


def test_fresh_body_stream_passes_and_closes_sdk():
    clock = Clock()
    sdk = SDK(clock)
    report, bodies, stamps = probe_module.probe(sdk, 1, clock, clock.sleep)
    assert report["status"] == "PASS"
    assert bodies.shape == (50, 24, 7)
    assert np.all(np.diff(stamps) > 0)
    assert sdk.closed


@pytest.mark.parametrize("freeze,invalid", [(True, False), (False, True)])
def test_stale_or_invalid_body_is_not_ready(freeze, invalid):
    clock = Clock()
    sdk = SDK(clock, freeze, invalid)
    report, _, _ = probe_module.probe(sdk, 1, clock, clock.sleep)
    assert report["status"] == "NOT_READY"
    assert sdk.closed


def test_missing_tail_is_included_in_freshness_gate():
    clock = Clock()
    sdk = SDK(clock)
    sdk.is_body_data_available = lambda: clock() < 0.5
    report, _, _ = probe_module.probe(sdk, 1, clock, clock.sleep)
    assert report["fresh_hz"] > 30
    assert report["status"] == "NOT_READY"


def test_body_shape_and_zero_quaternions_rejected():
    for body in [np.zeros((3, 7)), np.zeros((24, 7))]:
        with pytest.raises(ValueError):
            probe_module.validate_body(body)
