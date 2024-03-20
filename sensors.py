import signal
import sys
import time
import RPi.GPIO as GPIO
from bh1750 import BH1750
from htu21d import HTU21D
from soil_moisture_sensor import SoilMoistureSensor

# Pin 15 on Raspberry Pi corresponds to GPIO 22
LED1 = 15
# Pin 16 on Raspberry Pi corresponds to GPIO 23
LED2 = 16

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

if __name__ == '__main__':
    try:
        obj_bh1750 = BH1750()
        obj_htu21d = HTU21D()
        moisture_sensor = SoilMoistureSensor(spi_channel=0)

        # Define measurement intervals (in seconds)
        BH1750_INTERVAL = 0.15
        HTU21D_INTERVAL = 0.1
        MOISTURE_SENSOR_INTERVAL = 0.1

        # Store initial values
        light_intensity = float(obj_bh1750.light())
        temp = float(obj_htu21d.temperature())
        humidity = float(obj_htu21d.humidity())
        current_adc_value = moisture_sensor.get_adc(0)
        moisture1 = round(moisture_sensor.valmap(float(current_adc_value), 5, 3.5, 0, 100), 0)

        # Print initial values
        print(f'Light Intensity: {light_intensity} Lux')
        print(f'Temperature: {temp:.2f}°C, Humidity: {humidity:.0f}%')
        print(f'Soil Moisture Sensor: {moisture1}%')
        print('\n')

        # Get initial time
        last_bh1750_measurement_time = last_htu21d_measurement_time = last_moisture_measurement_time = time.time()

        # Define current_moisture1 before the while loop
        current_moisture1 = moisture1

        while True:
            current_time = time.time()

            # Read light intensity
            if current_time - last_bh1750_measurement_time >= BH1750_INTERVAL:
                current_light_intensity = float(obj_bh1750.light())
                last_bh1750_measurement_time = current_time
                
                if abs(current_light_intensity - light_intensity) >= 100:
                    print(f'Light Intensity: {current_light_intensity} Lux')
                    light_intensity = current_light_intensity

            # Read temperature and humidity
            if current_time - last_htu21d_measurement_time >= HTU21D_INTERVAL:
                current_temp, current_humidity = float(obj_htu21d.temperature()), float(obj_htu21d.humidity())
                last_htu21d_measurement_time = current_time
                
                if abs(current_temp - temp) >= 1:
                    print(f'Temperature: {current_temp:.2f}°C')
                    temp = current_temp

                if abs(current_humidity - humidity) >= 10:
                    print(f'Humidity: {current_humidity:.0f}%')
                    humidity = current_humidity

            # Read soil moisture
            if current_time - last_moisture_measurement_time >= MOISTURE_SENSOR_INTERVAL:
                current_adc_value = moisture_sensor.get_adc(0)
                current_moisture1 = round(moisture_sensor.valmap(float(current_adc_value), 5, 3.5, 0, 100), 0)
                last_moisture_measurement_time = current_time
                
                # Ensure moisture value is within the range of 0 to 100
                current_moisture1 = max(0, min(100, current_moisture1))
                
                if abs(current_moisture1 - moisture1) >= 10:
                    print(f'Soil Moisture Sensor: {current_moisture1}%')
                    moisture1 = current_moisture1

            # Control LEDs based on moisture levels
            if current_moisture1 < 40:
                GPIO.output(LED1, 1)
                GPIO.output(LED2, 0)
            else:
                GPIO.output(LED1, 0)
                GPIO.output(LED2, 1)

    except FileNotFoundError:
        print('ERROR: Please enable I2C.')
    except OSError:
        print('ERROR: I2C device not found. Please check sensor wiring.')
    except Exception as e:
        print(f'ERROR: {e}')

    finally:
        GPIO.cleanup()
