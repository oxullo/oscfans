#!/usr/bin/env python

import time
import click

import imgui
from aimgui import App
from aimgui.windows.window import Window
import pythonosc.udp_client


class Sender(Window):
    def __init__(self, id, ip, port):
        super().__init__(label=f'Fan {id}')
        self._id = id
        self._client = pythonosc.udp_client.SimpleUDPClient(ip, port)
        self._speed = 0
        self._enabled = False

    def send(self):
        if self._enabled:
            self._client.send_message(f'/s/{self._id}', self._speed)

    def _on_frame(self):
        _, self._enabled = imgui.checkbox('Enable send', self._enabled)
        changed, speed = imgui.slider_float('Fan speed', self._speed, 0, 100)


class DemoApp(App):
    VERSION = '0.1.0'
    WINDOW_SIZE = (640, 200)

    _WIN_FLAGS = imgui.WINDOW_NO_COLLAPSE
    _GRID_DIMS = (2, 1)

    def __init__(self, ip, port, update_period):
        super().__init__(
            title='ATONAL OSC Sender',
            width=self.WINDOW_SIZE[0], height=self.WINDOW_SIZE[1], resizable=False
        )
        self._senders = [
            Sender(1, ip, port),
            Sender(2, ip, port),
        ]
        self._timers.make_interval(update_period / 1000, self._on_timer, start=True)

    def _on_frame(self):
        for offset, sender in enumerate(self._senders):
            sender.on_frame_grid(flags=self._WIN_FLAGS, dims=self._GRID_DIMS, cell=(offset, 0))

    def _on_timer(self):
        for sender in self._senders:
            sender.send()


@click.command()
@click.option('--ip', default='192.168.1.20')
@click.option('--port', default=8888)
@click.option('--period', default=100)
def run(ip, port, period):
    import logging

    logging.basicConfig(level=logging.INFO)
    DemoApp(ip, port, period).main_loop()


if __name__ == '__main__':
    run()
