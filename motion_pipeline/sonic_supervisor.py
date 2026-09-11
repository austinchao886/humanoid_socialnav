"""Compatibility import for :mod:`sonic_tracker.supervisor`."""

from sonic_tracker.supervisor import *  # noqa: F401,F403
from sonic_tracker.supervisor import main


if __name__ == "__main__":
    main()
