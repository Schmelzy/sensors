import signal
import sys
import time
import spidev
import RPi.GPIO as GPIO
import smbus

BH1750_ADDR = 0x23
CMD_READ = 0x10

HTU21D_ADDR = 0x40
CMD_READ_TEMP = 0xE3
CMD_READ_HUM = 0xE5
CMD_RESET = 0xFE

# Pin 15 on Raspberry Pi corresponds to GPIO 22
LED1 = 15
# Pin 16 on Raspberry Pi corresponds to GPIO 23
LED2 = 16

spi_ch = 0

# Enable SPI
spi = spidev.SpiDev(0, spi_ch)
spi.max_speed_hz = 1200000

# to use Raspberry Pi board pin numbers
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# set up GPIO output channels for LEDs
GPIO.setup(LED1, GPIO.OUT)
GPIO.setup(LED2, GPIO.OUT)

class BH1750(object):
    def __init__(self):
        # Rev 2 of Raspberry Pi and all newer use bus 1
        self.bus = smbus.SMBus(1)

    def light(self):
        data = self.bus.read_i2c_block_data(BH1750_ADDR, CMD_READ)
        result = (data[1] + (256 * data[0])) / 1.2
        return format(result, '.0f')

class HTU21D(object):
    def __init__(self):
        # Rev 2 of Raspberry Pi and all newer use bus 1
        self.bus = smbus.SMBus(1)

    def reset(self):
        self.bus.write_byte(HTU21D_ADDR, CMD_RESET)

    def temperature(self):
        self.reset()
        msb, lsb, crc = self.bus.read_i2c_block_data(HTU21D_ADDR, CMD_READ_TEMP, 3)
        return -46.85 + 175.72 * (msb * 256 + lsb) / 65536

    def humidity(self):
        self.reset()
        msb, lsb, crc = self.bus.read_i2c_block_data(HTU21D_ADDR, CMD_READ_HUM, 3)
        return -6 + 125 * (msb * 256 + lsb) / 65536.0

def close(signal, frame):
    GPIO.output(LED1, 0)
    GPIO.output(LED2, 0)
    sys.exit(0)

signal.signal(signal.SIGINT, close)

def valmap(value, istart, istop, ostart, ostop):
    value = ostart + (ostop - ostart) * ((value - istart) / (istop - istart))
    if value > ostop:
        value = ostop
    return value

def get_adc(channel):
    # Make sure ADC channel is 0 or 1
    if channel != 0:
        channel = 1

    # Construct SPI message
    msg = 0b11
    msg = ((msg << 1) + channel) << 5
    msg = [msg, 0b00000000]
    reply = spi.xfer2(msg)

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

if __name__ == '__main__':
    try:
        obj_bh1750 = BH1750()
        obj_htu21d = HTU21D()

        while True:
            # Read light intensity
            light_intensity = obj_bh1750.light()
            print(f'Light Intensity: {light_intensity} Lux')

            # Read temperature and humidity
            temp = obj_htu21d.temperature()
            humidity = obj_htu21d.humidity()
            print(f'Temperature: {temp:.2f}°C, Humidity: {humidity:.0f}%')

            # Read soil moisture
            adc_0 = get_adc(0)
            adc_1 = get_adc(1)
            sensor1 = round(adc_0, 2)
            if sensor1 < 0.5:
                moisture1 = 0
            else:
                moisture1 = round(valmap(sensor1, 5, 3.5, 0, 100), 0)

            sensor2 = round(adc_1, 2)
            if sensor2 < 0.5:
                moisture2 = 0
            else:
                moisture2 = round(valmap(sensor2, 5, 3.5, 0, 100), 0)

            print(f"Soil Moisture Sensor 1: {moisture1}%, Soil Moisture Sensor 2: {moisture2}%")
            print('\n')
            # Control LEDs based on moisture levels
            if moisture1 < 40 or moisture2 < 40:
                GPIO.output(LED1, 1)
                GPIO.output(LED2, 0)
            else:
                GPIO.output(LED1, 0)
                GPIO.output(LED2, 1)

            time.sleep(2.5)

    except FileNotFoundError:
        print('ERROR: Please enable I2C.')
    except OSError:
        print('ERROR: I2C device not found. Please check sensor wiring.')
    except Exception as e:
        print(f'ERROR: {e}')

    finally:
        GPIO.cleanup()
