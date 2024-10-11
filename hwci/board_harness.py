# board_harness.py
class BoardHarness:
    arch = None
    kernel_board_path = None

    def __init__(self):
        self.serial = self.get_serial_port()
        self.gpio = self.get_gpio_interface()

    def get_uart_port(self):
        pass

    def get_uart_baudrate(self):
        pass

    def get_serial_port(self):
        pass

    def get_gpio_interface(self):
        pass

    def erase_board(self):
        pass

    def flash_kernel(self):
        pass

    def flash_app(self, app):
        pass
