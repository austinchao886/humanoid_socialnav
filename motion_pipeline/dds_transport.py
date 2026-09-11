from __future__ import annotations

from collections.abc import Callable
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


class JsonDDS:
    def __init__(self, domain: int = 0, interface: str | None = None):
        if interface:
            ChannelFactoryInitialize(domain, interface)
        else:
            ChannelFactoryInitialize(domain)
        self._publishers: dict[str, ChannelPublisher] = {}
        self._subscribers: list[ChannelSubscriber] = []

    def publish(self, topic: str, payload: str) -> None:
        publisher = self._publishers.get(topic)
        if publisher is None:
            publisher = ChannelPublisher(topic, String_)
            publisher.Init()
            self._publishers[topic] = publisher
            # Give CycloneDDS discovery a brief window before the first one-shot
            # CLI command; long-running services only pay this once per topic.
            time.sleep(0.25)
        message = String_(data=payload)
        publisher.Write(message)

    def subscribe(self, topic: str, callback: Callable[[str], None]) -> None:
        subscriber = ChannelSubscriber(topic, String_)
        subscriber.Init(lambda msg: callback(msg.data), 10)
        self._subscribers.append(subscriber)
