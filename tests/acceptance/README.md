# Acceptance tests

Hardware- and simulator-backed qualification scenarios belong here. Shell
entrypoints live under `tools/acceptance/` and write reports outside Git.

`tools/acceptance/run_persistent_custom_acceptance.sh` approves multiple
validated references and fails unless every execution returns to
`READY_STANDING` with the original Isaac session id, Isaac PID, and SONIC PID.
