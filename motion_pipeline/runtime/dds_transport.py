from __future__ import annotations

from collections.abc import Callable
import time

class JsonDDS:
    def __init__(self, domain: int = 0, interface: str | None = None):
        # Keep the hardware/DDS dependency at the transport boundary. Pure
        # protocol, validation, and supervisor tests can then import the
        # package without installing Unitree SDK or CycloneDDS.
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        if interface:
            ChannelFactoryInitialize(domain, interface)
        else:
            ChannelFactoryInitialize(domain)
        self._publisher_type = ChannelPublisher
        self._subscriber_type = ChannelSubscriber
        self._message_type = String_
        self._publishers: dict[str, object] = {}
        self._subscribers: list[object] = []

    def prepare_publisher(self, topic: str) -> None:
        """Pay one-shot discovery before entering a time-sensitive producer loop."""
        publisher = self._publishers.get(topic)
        if publisher is None:
            publisher = self._publisher_type(topic, self._message_type)
            publisher.Init()
            self._publishers[topic] = publisher
            # Give CycloneDDS discovery a brief window before the first one-shot
            # CLI command; long-running services only pay this once per topic.
            time.sleep(0.25)

    def publish(self, topic: str, payload: str) -> None:
        self.prepare_publisher(topic)
        publisher = self._publishers[topic]
        message = self._message_type(data=payload)
        publisher.Write(message)

    def subscribe(self, topic: str, callback: Callable[[str], None]) -> None:
        subscriber = self._subscriber_type(topic, self._message_type)
        subscriber.Init(lambda msg: callback(msg.data), 10)
        self._subscribers.append(subscriber)
