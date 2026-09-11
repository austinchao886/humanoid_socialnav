# Runtime data

Runtime artifacts do not belong in Git. The deployed workspace currently keeps
large shared state in sibling directories such as `../motion_exchange` and
`../motion_models`. This directory documents the ownership boundary and can be
used for small local-development state later; generated contents must remain
ignored.
