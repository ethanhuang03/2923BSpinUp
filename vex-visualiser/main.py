import signal
import time

from pros.cli.common import logger, resolve_v5_port
import logging

from pros.serial.devices.vex import V5UserDevice
from pros.serial.ports import DirectPort
from pros.common.ui.log import PROSLogHandler, PROSLogFormatter

from pros.serial.devices import StreamDevice
from pros.serial.terminal import Terminal

from pros.common.utils import logger
from pros.serial import decode_bytes_to_str
from pros.serial.ports import PortConnectionException

from chart_manager import ChartManager


class GUITerminal(Terminal):
    def __init__(self, port_instance: StreamDevice, transformations=(),
                 output_raw: bool = False, request_banner: bool = True):
        super().__init__(port_instance, transformations, output_raw, request_banner)

        self.chart_manager = ChartManager()

    def reader(self):
        if self.request_banner:
            try:
                self.device.write(b'pRb')
            except Exception as e:
                logger(__name__).exception(e)
        try:
            while not self.alive.is_set() and self._reader_alive:
                data = self.device.read()
                if not data:
                    continue

                text = decode_bytes_to_str(data[1])
                self.chart_manager.parse(self, text)

        except UnicodeError as e:
            logger(__name__).exception(e)
        except PortConnectionException:
            logger(__name__).warning(f'Connection to {self.device.name} broken')
            if not self.alive.is_set():
                self.stop()
        except (BrokenPipeError, OSError) as e:
            logger(__name__).warning('Connection to GUI was abruptly stopped. Closing terminal...')
            if not self.alive.is_set():
                self.stop()
        except Exception as e:
            if not self.alive.is_set():
                logger(__name__).exception(e)
            else:
                logger(__name__).debug(e)
            try:
                logger(__name__).info("Beginning terminal closure..")
                self.stop()
            except Exception as exceptionException:
                logger(__name__).error("Encountered exception while closing:")
                logger(__name__).exception(exceptionException)

    def stop(self, *args):
        super().stop()
        quit()


def main():
    # Use same logging system as PROS
    ctx_obj = {}
    click_handler = PROSLogHandler(ctx_obj=ctx_obj)
    ctx_obj['click_handler'] = click_handler
    formatter = PROSLogFormatter('%(levelname)s - %(name)s:%(funcName)s - %(message)s', ctx_obj)
    click_handler.setFormatter(formatter)
    logging.basicConfig(level=logging.INFO, handlers=[click_handler])

    logger(__name__).info("Starting Application...")

    # Wait for GUI to launch
    logger(__name__).info("Application successfully started, waiting for connection")

    logger(__name__).debug(f"Finding port...")

    port = DirectPort(resolve_v5_port(None, 'user')[0])
    device = V5UserDevice(port)
    app = GUITerminal(device)

    logger(__name__).info(f"Attempting to receive data...")

    signal.signal(signal.SIGINT, app.stop)
    app.start()

    while not app.alive.is_set():
        time.sleep(0.005)
    app.join()
    logger(__name__).info("Shutting down terminal...")


if __name__ == "__main__":
    main()