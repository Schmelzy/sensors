import smbus

BH1750_ADDR = 0x23
CMD_READ = 0x10

class BH1750(object):
    def __init__(self):
        # Rev 2 of Raspberry Pi and all newer use bus 1
        self.bus = smbus.SMBus(1)

    def light(self):
        data = self.bus.read_i2c_block_data(BH1750_ADDR, CMD_READ)
        result = (data[1] + (256 * data[0])) / 1.2
        return format(result, '.0f')
