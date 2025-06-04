import paho.mqtt.client as mqtt
import json

def send_mqtt(data, broker, topic):
    client = mqtt.Client()
    client.connect(broker, 1883, 60)
    client.publish(topic, json.dumps(data))
    client.disconnect()

if __name__ == "__main__":
    data = {
        "motion_detected": True,
        "distance": 1.2
    }
    send_mqtt(data, "mqtt.example.com", "bgt60tr13c/sensor1")
