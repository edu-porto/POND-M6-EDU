from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import cv2
import numpy as np

app = FastAPI()

@app.websocket("/wsVideo")
async def websocket_video(websocket: WebSocket):
    await websocket.accept()
    print("Video websocket connected")
    video_capture = cv2.VideoCapture(0)
    if not video_capture.isOpened():
        await websocket.close()
        print("Failed to open video capture")
        return
    try:
        while True:
            ret, frame = video_capture.read()
            if not ret:
                print("Failed to read frame")
                break
            _, buffer = cv2.imencode('.jpg', frame)

              # Aumentando a compressão da imagem
            encoding = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
            _, buffer = cv2.imencode('.jpg', frame, encoding)
            
            await websocket.send_bytes(buffer.tobytes())
    except WebSocketDisconnect:
        print("WebSocket disconnected")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        video_capture.release()
        await websocket.close()
