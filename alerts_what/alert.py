import json
import time
from twilio.rest import Client

# Load config
with open("config.json", "r") as file:
    config = json.load(file)

# Twilio credentials
client = Client(
    config["account_sid"],
    config["auth_token"]
)

FROM_WHATSAPP = "whatsapp:" + config["FROM_WHATSAPP"]
TO_WHATSAPP = "whatsapp:" + config["TO_WHATSAPP"]

while True:
    try:
        with open("../nanodrone_auto/human_count.txt", "r") as file:
            human_count = file.read().strip()

        message = client.messages.create(
            from_=FROM_WHATSAPP,
            body=f"🚨 ALERT: {human_count} human(s) detected by drone!",
            to=TO_WHATSAPP
        )

        print(f"Message sent! Count = {human_count}")
        print("SID:", message.sid)

    except Exception as e:
        print("Error:", e)

    time.sleep(120)