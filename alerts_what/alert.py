import json
from twilio.rest import Client 

# Load config
with open("config.json", "r") as file:
    config = json.load(file)

# Twilio credentials
account_sid = config["account_sid"]
auth_token = config["auth_token"]

FROM_WHATSAPP = "whatsapp:" + config["FROM_WHATSAPP"]
TO_WHATSAPP = "whatsapp:" + config["TO_WHATSAPP"]

# Create client
client = Client(account_sid, auth_token)

# Send message
message = client.messages.create(
    from_=FROM_WHATSAPP,
    body="🚨 ALERT: Human detected by drone!",
    to=TO_WHATSAPP
)

print("Message sent!")
print("SID:", message.sid)