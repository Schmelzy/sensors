import spidev

class SoilMoistureSensor:
    def __init__(self, spi_channel):
        self.spi = spidev.SpiDev(0, spi_channel)
        self.spi.max_speed_hz = 1200000

    def get_adc(self, channel):
        # Make sure ADC channel is 0 or 1
        if channel != 0:
            channel = 1

        # Construct SPI message
        msg = 0b11
        msg = ((msg << 1) + channel) << 5
        msg = [msg, 0b00000000]
        reply = self.spi.xfer2(msg)

        # Construct single integer out of the reply (2 bytes)
        adc = 0
        for n in reply:
            adc = (adc << 8) + n

        # Last bit (0) is not part of ADC value, shift to remove it
        adc = adc >> 1

        # Calculate voltage from ADC value
        # considering the soil moisture sensor is working at 5V
        voltage = (5 * adc) / 1024

        return voltage

    @staticmethod
    def valmap(value, istart, istop, ostart, ostop):
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))
