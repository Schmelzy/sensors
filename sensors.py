import paho.mqtt.client as mqtt
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

mqttc = None
msg_light = msg_temp = msg_humidity = msg_soil_moisture = None

# to use Raspberry Pi board pin numbers
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# set up GPIO output channels for LEDs
GPIO.setup(LED1, GPIO.OUT)
GPIO.setup(LED2, GPIO.OUT)

def close(signal, frame):
    GPIO.output(LED1, 0)
    GPIO.output(LED2, 0)
    if mqttc != None:
        mqttc.disconnect()
        mqttc.loop_stop()
    sys.exit(0)

signal.signal(signal.SIGINT, close)

def read_and_publish_light_intensity(obj_bh1750, current_time, last_bh1750_measurement_time, light_intensity, mqttc, unacked_publish):
    if current_time - last_bh1750_measurement_time >= BH1750_INTERVAL:
        current_light_intensity = float(obj_bh1750.light())
        
        if abs(current_light_intensity - light_intensity) >= 100:
            print(f'Light Intensity: {current_light_intensity:.0f} Lux')
            light_intensity = current_light_intensity
            msg_light = mqttc.publish("tugay/light", f'{current_light_intensity:.0f} Lux', qos=1)
            unacked_publish.add(msg_light.mid)

        last_bh1750_measurement_time = current_time  # Update last measurement time    
    return light_intensity, last_bh1750_measurement_time

def read_and_publish_temperature_and_humidity(obj_htu21d, current_time, last_htu21d_measurement_time, temp, humidity, mqttc, unacked_publish):  
    if current_time - last_htu21d_measurement_time >= HTU21D_INTERVAL:
        current_temp, current_humidity = float(obj_htu21d.temperature()), float(obj_htu21d.humidity())
        
        if abs(current_temp - temp) >= 1:
            print(f'Temperature: {current_temp:.2f}°C')
            temp = current_temp
            msg_temp = mqttc.publish("tugay/temperature", f'{current_temp:.2f}°C', qos=1)
            unacked_publish.add(msg_temp.mid)

        if abs(current_humidity - humidity) >= 10:          
            print(f'Humidity: {current_humidity:.0f}%')
            humidity = current_humidity
            msg_humidity = mqttc.publish("tugay/humidity", f'{current_humidity:.0f}%', qos=1)
            unacked_publish.add(msg_humidity.mid)

        last_htu21d_measurement_time = current_time  # Update last measurement time   
    return temp, humidity, last_htu21d_measurement_time

def read_soil_moisture_sensor(moisture_sensor):
    current_adc_value = moisture_sensor.get_adc(0)
    current_moisture = round(moisture_sensor.valmap(float(current_adc_value), 5, 3.5, 0, 100), 0)
    return current_moisture

def read_and_publish_soil_moisture(moisture_sensor, current_time, last_moisture_measurement_time, moisture, mqttc, unacked_publish):
    if current_time - last_moisture_measurement_time >= MOISTURE_SENSOR_INTERVAL:
        current_moisture = read_soil_moisture_sensor(moisture_sensor)
        
        if abs(current_moisture - moisture) >= 10:           
            print(f'Soil Moisture Sensor: {current_moisture:.0f}%')
            moisture = current_moisture
            msg_soil_moisture = mqttc.publish("tugay/soil_moisture", f'{current_moisture:.0f}%', qos=1)
            unacked_publish.add(msg_soil_moisture.mid)

        last_moisture_measurement_time = current_time  # Update last measurement time
    return moisture, last_moisture_measurement_time

def on_publish(client, userdata, mid, reason_code, properties):
    # reason_code and properties will only be present in MQTTv5. It's always unset in MQTTv3
    try:
        userdata.remove(mid)
    except KeyError:
        print("on_publish() is called with a mid not present in unacked_publish")

if __name__ == '__main__':
    try:
        unacked_publish = set()
        mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        mqttc.on_publish = on_publish
        mqttc.user_data_set(unacked_publish)
        mqttc.connect("localhost", 1883, 60)
        mqttc.loop_start()

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
        moisture = read_soil_moisture_sensor(moisture_sensor)

        # Publish initial values as MQTT messages
        msg_light = mqttc.publish("tugay/light", f'{light_intensity:.0f} Lux', qos=1)
        unacked_publish.add(msg_light.mid)
        msg_temp = mqttc.publish("tugay/temperature", f'{temp:.2f}°C', qos=1)
        unacked_publish.add(msg_temp.mid)
        msg_humidity = mqttc.publish("tugay/humidity", f'{humidity:.0f}%', qos=1)
        unacked_publish.add(msg_humidity.mid)
        msg_soil_moisture = mqttc.publish("tugay/soil_moisture", f'{moisture:.0f}%', qos=1)
        unacked_publish.add(msg_soil_moisture.mid)

        # Print initial values
        print(f'Light Intensity: {light_intensity:.0f} Lux')
        print(f'Temperature: {temp:.2f}°C, Humidity: {humidity:.0f}%')
        print(f'Soil Moisture Sensor: {moisture:.0f}%')
        print('\n')

        # Get initial time
        last_bh1750_measurement_time = last_htu21d_measurement_time = last_moisture_measurement_time = time.time()

        while True:
            current_time = time.time()

            # Read and publish light intensity
            light_intensity, last_bh1750_measurement_time = read_and_publish_light_intensity(obj_bh1750, current_time, last_bh1750_measurement_time, light_intensity, mqttc, unacked_publish)
            # Read and publish temperature and humidity
            temp, humidity, last_htu21d_measurement_time = read_and_publish_temperature_and_humidity(obj_htu21d, current_time, last_htu21d_measurement_time, temp, humidity, mqttc, unacked_publish)
            # Read and publish soil moisture
            moisture, last_moisture_measurement_time = read_and_publish_soil_moisture(moisture_sensor, current_time, last_moisture_measurement_time, moisture, mqttc, unacked_publish)

            # Control LEDs based on moisture levels
            if moisture < 40:
                GPIO.output(LED1, 1)
                GPIO.output(LED2, 0)
            else:
                GPIO.output(LED1, 0)
                GPIO.output(LED2, 1)

            # Wait for all message to be published
            while len(unacked_publish):
                time.sleep(0.1)

            # Due to race-condition described above, the following way to wait for all publish is safer
            messages = [msg_light, msg_temp, msg_humidity, msg_soil_moisture]

            for msg in messages:
                if msg is not None:
                    msg.wait_for_publish()
                  
    except FileNotFoundError:
        print('ERROR: Please enable I2C.')
    except OSError:
        print('ERROR: I2C device not found. Please check sensor wiring.')
    except Exception as e:
        print(f'ERROR: {e}')

    finally:
        GPIO.cleanup()
        if mqttc != None:
            mqttc.disconnect()
            mqttc.loop_stop()

