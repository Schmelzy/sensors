import signal
import sys
import time
import spidev
import RPi.GPIO as GPIO
import smbus
from bh1750 import BH1750
from htu21d import HTU21D

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

        # Store initial values
        light_intensity = obj_bh1750.light()
        temp = obj_htu21d.temperature()
        humidity = obj_htu21d.humidity()
        adc_0 = get_adc(0)
        moisture1 = round(valmap(adc_0, 5, 3.5, 0, 100), 0)

        # Print initial values
        print(f'Light Intensity: {light_intensity} Lux')
        print(f'Temperature: {temp:.2f}°C, Humidity: {humidity:.0f}%')
        print(f'Soil Moisture Sensor: {moisture1}%')
        print('\n')

        while True:
            # Read light intensity
            current_light_intensity = obj_bh1750.light()
            # Read temperature and humidity
            current_temp = obj_htu21d.temperature()
            current_humidity = obj_htu21d.humidity()
            # Read soil moisture
            current_adc_0 = get_adc(0)
            current_moisture1 = round(valmap(current_adc_0, 5, 3.5, 0, 100), 0)

            # Check for differences
            temp_difference = abs(current_temp - temp)
            humidity_difference = abs(current_humidity - humidity)
            light_difference = abs(float(current_light_intensity) - float(light_intensity))
            moisture1_difference = abs(current_moisture1 - moisture1)

            # Print values if differences are detected
            if (
                temp_difference >= 1 or humidity_difference >= 10 or
                light_difference >= 50 or moisture1_difference >= 10
            ):
                print(f'Light Intensity: {current_light_intensity} Lux')
                print(f'Temperature: {current_temp:.2f}°C, Humidity: {current_humidity:.0f}%')
                print(f'Soil Moisture Sensor: {current_moisture1}%')
                print('\n')

                # Update initial values
                light_intensity = current_light_intensity
                temp = current_temp
                humidity = current_humidity
                moisture1 = current_moisture1

            # Control LEDs based on moisture levels
            if current_moisture1 < 40:
                GPIO.output(LED1, 1)
                GPIO.output(LED2, 0)
            else:
                GPIO.output(LED1, 0)
                GPIO.output(LED2, 1)

            time.sleep(0.5)

    except FileNotFoundError:
        print('ERROR: Please enable I2C.')
    except OSError:
        print('ERROR: I2C device not found. Please check sensor wiring.')
    except Exception as e:
        print(f'ERROR: {e}')

    finally:
        GPIO.cleanup()
