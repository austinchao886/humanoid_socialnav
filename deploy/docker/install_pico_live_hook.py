"""Narrow, fail-closed patch for the pinned composition image supervisor."""
from pathlib import Path
path=Path('/usr/local/lib/python3.10/dist-packages/sonic_tracker/supervisor.py')
source=path.read_text()
anchor='            except queue.Empty:\n'
if source.count(anchor)!=1:
    raise RuntimeError('unexpected supervisor layout; review image before rebuilding')
hook='                from sonic_tracker.pico_live_session import poll as poll_pico_live\n                poll_pico_live(self)\n'
if hook not in source:
    source=source.replace(anchor,anchor+hook)
compile(source,str(path),'exec')
path.write_text(source)
