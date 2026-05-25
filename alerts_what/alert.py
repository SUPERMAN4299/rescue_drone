import json
import time
from twilio.rest import Client

# Load config
with open("config.json", "r") as file:
    config = json.load(file)

with open("../camera_ana/human_count.txt", "r") as file:
    human_count = file.read().strip()

# Twilio credentials
account_sid = config["account_sid"]
auth_token = config["auth_token"]

FROM_WHATSAPP = "whatsapp:" + config["FROM_WHATSAPP"]
TO_WHATSAPP = "whatsapp:" + config["TO_WHATSAPP"]

# Create client
client = Client(account_sid, auth_token)

# Send message
while True:
    message = client.messages.create(
        from_=FROM_WHATSAPP,
        body=f"🚨 ALERT: {human_count} Human detected by drone in the frame!",
        to=TO_WHATSAPP
    )

    print("Message sent!")
    print("SID:", message.sid)
    time.sleep(120)