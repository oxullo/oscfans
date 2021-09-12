#!/usr/bin/env python

import click
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer


def print_handler(address, *args):
    print(f'{address}: {args}')


def default_handler(address, *args):
    print(f'DEFAULT {address}: {args}')


@click.command()
@click.option('--listen-addr', default='0.0.0.0')
@click.option('--port', default=8888)
def run(listen_addr, port):
    dispatcher = Dispatcher()
    dispatcher.map('/s/*', print_handler)
    dispatcher.set_default_handler(default_handler)

    print(f'Starting server listen_addr={listen_addr}:{port}')
    server = BlockingOSCUDPServer((listen_addr, port), dispatcher)
    server.serve_forever()  # Blocks forever


if __name__ == '__main__':
    run()
