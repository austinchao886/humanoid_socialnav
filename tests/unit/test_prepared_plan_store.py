import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from motion_contracts.protocol import ControlCommand, ProtocolError
from sonic_tracker.prepared_plan_store import PreparedPlanStore
from sonic_tracker.prepared_gesture import PreparedGesture
from sonic_tracker.supervisor import SonicSupervisor


class PlanStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.artifact = self.root / "wave"
        self.artifact.mkdir()
        (self.artifact / "manifest.json").write_text(json.dumps(dict(
            motion_id="wave", request_id="request",
            execution_contract={"asset": "sonic_official_g1"})))
        for name in ("validation.json", "joint_pos.csv", "joint_vel.csv"):
            (self.artifact / name).write_text("fixture")
        gesture = PreparedGesture("content", "wave", 2., .25, 2.,
                                  ((0.,)*29,)*3, ((0.,)*29,)*3)
        patcher = patch("sonic_tracker.prepared_plan_store.prepare_gesture", return_value=gesture)
        self.prepare_mock = patcher.start()
        self.addCleanup(patcher.stop)
        self.store = PreparedPlanStore(self.root, capacity=1)
        self.options = dict(amplitude=.25, time_scale=2., start_offset_s=0.,
                            end_offset_s=2., entry_s=.5, exit_s=.5)

    def prepare(self):
        return self.store.prepare("wave", **self.options)

    def command(self, entry, **changes):
        fields = dict(request_id="request", motion_id="wave", action="approve_execute",
                      prepared_plan_id=entry.plan.plan_id)
        fields.update(changes)
        return ControlCommand(**fields)

    def resolve(self, command):
        return self.store.resolve(command, required_reference_contract="sonic_official_g1")

    def test_lookup_reuses_immutable_plan_without_csv_parsing(self):
        entry = self.prepare()
        self.prepare_mock.reset_mock()
        self.assertIs(self.resolve(self.command(entry)), entry.plan)
        self.prepare_mock.assert_not_called()

    def test_ownership_unknown_and_legacy_approval_rejected(self):
        entry = self.prepare()
        for changes in (dict(request_id="other"), dict(motion_id="other"),
                        dict(prepared_plan_id="b"*64), dict(prepared_plan_id=None)):
            with self.subTest(changes=changes), self.assertRaises(ProtocolError):
                self.resolve(self.command(entry, **changes))

    def test_changed_source_requires_new_preparation_and_approval(self):
        entry = self.prepare()
        (self.artifact / "joint_pos.csv").write_text("changed")
        with self.assertRaisesRegex(ProtocolError, "stale"):
            self.resolve(self.command(entry))

    def test_source_change_during_preparation_rejected(self):
        gesture = self.prepare_mock.return_value
        def change(*args, **kwargs):
            (self.artifact / "joint_pos.csv").write_text("changed")
            return gesture
        self.prepare_mock.side_effect = change
        with self.assertRaisesRegex(ProtocolError, "during preparation"):
            self.prepare()

    def test_bounded_cache_requires_explicit_discard(self):
        entry = self.prepare()
        self.options["entry_s"] = .6
        with self.assertRaisesRegex(ProtocolError, "cache full"):
            self.prepare()
        self.store.discard(entry.plan.plan_id)
        with self.assertRaisesRegex(ProtocolError, "unavailable"):
            self.resolve(self.command(entry))
        self.prepare()

    def test_path_escape_rejected(self):
        with self.assertRaises(ProtocolError):
            self.store.prepare("../wave", **self.options)

    def test_catalog_load_is_atomic_and_does_not_approve(self):
        entry = self.prepare()
        catalog = self.root / "catalog.json"
        recipe = dict(motion_id="wave",plan_id=entry.plan.plan_id,**self.options)
        catalog.write_text(json.dumps(dict(schema_version=1,plans=[recipe])))
        supervisor = SonicSupervisor(self.root,self.root,object())
        with patch("sonic_tracker.supervisor.validate_artifact",return_value=SimpleNamespace(valid=True)):
            summaries = supervisor._load_prepared_catalog(catalog)
            self.assertFalse(summaries[0]["approved"])
            self.assertFalse(summaries[0]["dynamic_qualification"])
            self.assertEqual(summaries[0]["prepared_plan_id"],entry.plan.plan_id)
            old_store = supervisor.prepared_plans
            bad = dict(recipe,plan_id="b"*64)
            catalog.write_text(json.dumps(dict(schema_version=1,plans=[bad])))
            with self.assertRaisesRegex(ProtocolError,"hash mismatch"):
                supervisor._load_prepared_catalog(catalog)
            self.assertIs(supervisor.prepared_plans,old_store)

    def test_catalog_rejects_unknown_fields_and_live_reload(self):
        catalog = self.root / "catalog.json"
        catalog.write_text(json.dumps(dict(schema_version=1,plans=[],approve=True)))
        supervisor = SonicSupervisor(self.root,self.root,object())
        with self.assertRaisesRegex(ProtocolError,"invalid prepared catalog"):
            supervisor._load_prepared_catalog(catalog)
        supervisor.child = object()
        with self.assertRaisesRegex(ProtocolError,"before runtime startup"):
            supervisor._load_prepared_catalog(catalog)

    def test_supervisor_preparation_and_approval_both_validate(self):
        supervisor = SonicSupervisor(self.root, self.root, object())
        with patch("sonic_tracker.supervisor.validate_artifact",
                   return_value=SimpleNamespace(valid=True)) as validate, \
             patch.object(supervisor, "_write_runtime_request") as runtime:
            entry = supervisor.prepare_gesture_plan("wave", **self.options)
            plan, result = supervisor._resolve_prepared_approval(self.command(entry))
            self.assertIs(plan, entry.plan)
            self.assertTrue(result.valid)
            self.assertEqual(validate.call_count, 2)
            runtime.assert_not_called()

    def test_supervisor_invalid_preparation_removes_cache_entry(self):
        supervisor = SonicSupervisor(self.root, self.root, object())
        with patch("sonic_tracker.supervisor.validate_artifact",
                   return_value=SimpleNamespace(valid=False)):
            with self.assertRaisesRegex(ProtocolError, "prepared-plan validation"):
                supervisor.prepare_gesture_plan("wave", **self.options)
        self.assertEqual(supervisor.prepared_plans.entries, {})

    def test_supervisor_detects_change_during_execution_validation(self):
        supervisor = SonicSupervisor(self.root, self.root, object())
        with patch("sonic_tracker.supervisor.validate_artifact",
                   return_value=SimpleNamespace(valid=True)):
            entry = supervisor.prepare_gesture_plan("wave", **self.options)
        def mutate(_, **kwargs):
            (self.artifact / "joint_vel.csv").write_text("changed")
            return SimpleNamespace(valid=True)
        with patch("sonic_tracker.supervisor.validate_artifact", side_effect=mutate):
            with self.assertRaisesRegex(ProtocolError, "stale"):
                supervisor._resolve_prepared_approval(self.command(entry))


if __name__ == "__main__":
    unittest.main()
