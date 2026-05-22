import cv2
import asyncio
import websockets

async def send_frames():

    uri = "ws://127.0.0.1:3001"

    async with websockets.connect(uri, max_size=None) as websocket:

        cap = cv2.VideoCapture(0)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        while True:

            ret, frame = cap.read()

            if not ret:
                print("Camera failed")
                break

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
            success, buffer = cv2.imencode('.jpg', frame, encode_param)

            if not success:
                print("JPG encoding failed")
                continue

            await websocket.send(buffer.tobytes())

            print("Frame sent")

            await asyncio.sleep(0.03)

asyncio.run(send_frames())