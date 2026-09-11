"""Compatibility import for :mod:`video_generator.service`."""

from video_generator.service import *  # noqa: F401,F403
from video_generator.service import main


if __name__ == "__main__":
    main()
