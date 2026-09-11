"""Guard canonical package ownership and temporary compatibility imports."""


def test_contract_compatibility_imports_resolve_to_canonical_symbols():
    from motion_contracts.protocol import ControlCommand as CanonicalControlCommand
    from motion_pipeline.protocol import ControlCommand as CompatibleControlCommand

    assert CompatibleControlCommand is CanonicalControlCommand


def test_service_compatibility_imports_resolve_to_canonical_entrypoints():
    from motion_generator.service import main as generator_main
    from motion_pipeline.generator_service import main as compatible_generator_main
    from motion_pipeline.sonic_supervisor import main as compatible_sonic_main
    from sonic_tracker.supervisor import main as sonic_main

    assert compatible_generator_main is generator_main
    assert compatible_sonic_main is sonic_main


def test_isaac_compatibility_import_resolves_to_canonical_helper():
    from isaac_runtime.bootstrap_support import elastic_support_scale as canonical
    from motion_pipeline.bootstrap_support import elastic_support_scale as compatible

    assert compatible is canonical
