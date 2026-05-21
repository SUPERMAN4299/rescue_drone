import subprocess

def get_wifi():
    data = subprocess.check_output(
        "netsh wlan show interfaces",
        shell=True,
        text=True
    )

    ssid = ""

    for line in data.split("\n"):
        if "SSID" in line and "BSSID" not in line:
            ssid = line.split(":")[1].strip()
            break

    profile = subprocess.check_output(
        f'netsh wlan show profile name="{ssid}" key=clear',
        shell=True,
        text=True
    )

    password = ""

    for line in profile.split("\n"):
        if "Key Content" in line:
            password = line.split(":")[1].strip()
            break

    with open("esp32cam-stream/wifi_config.h", "w") as f:
        f.write(f'#define WIFI_SSID "{ssid}"\n')
        f.write(f'#define WIFI_PASSWORD "{password}"\n')

    print("esp32cam-stream/wifi_config.h created")

get_wifi()