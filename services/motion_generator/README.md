# Motion generator

The `kimodo` Compose service owns prompt-to-motion generation. Its installed
entry point is `motion_generator.service:main`; generated
motion artifacts cross into other services only through the shared filesystem
and `motion_contracts` validation rules.
