import cv2
import time
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

app = FastAPI()


# Websocket que recebe a imagem da webcam
@app.websocket("/wsVideo")
async def websocket_video(websocket: WebSocket):
    await websocket.accept()
    print("Video websocket connected")
    video_capture = cv2.VideoCapture(0)

    try:
        print("Video capture opened")
        while True:
            # print("Reading frame")
            ret, frame = video_capture.read()
            if not ret:
                print("Frame not read")
                break
            _, buffer = cv2.imencode('.jpg', frame)
            # Adicionando um timestamp para calcular o ping 
            timestamp = str(time.time()).encode('utf-8')

            # Aumentando a compressão da imagem
            encoding = [int(cv2.IMWRITE_JPEG_QUALITY), 42]
            _, buffer = cv2.imencode('.jpg', frame, encoding)

            buffer_bytes = buffer.tobytes()
            delimiter = b'::'

            ws_msg = buffer_bytes + delimiter + timestamp
            # print(f"Sending frame  {(ws_msg)}")
            await websocket.send_bytes(ws_msg)
    except WebSocketDisconnect:
        video_capture.release()
        await websocket.close()
    except Exception as e:
        print(f"Error as exception: {e}")
        video_capture.release()
        await websocket.close()


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8500)
