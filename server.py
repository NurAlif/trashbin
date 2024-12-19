# Server Code
import ultralytics
from ultralytics import YOLO
import multiprocessing

import argparse
import signal
import asyncio
import json
import logging
import os
import uuid
import json
import threading
import time
import websockets
import cv2
from aiohttp import web
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRecorder, MediaRelay
from av import VideoFrame
import websockets.asyncio
from websockets.asyncio.server import serve

ROOT = os.path.dirname(__file__)

logger = logging.getLogger("pc")
pcs = set()
relay = MediaRelay()
# model = YOLO('/home/alif/yolo/yolov8/yolov11x_trash_classification/run16/weights/best.pt')
onnx_model = YOLO("/home/alif/yolo/yolov8/yolov11x_trash_classification/run16/weights/best.pt")

class VideoTransformTrack(MediaStreamTrack):

    kind = "video"
    count = 0

    def __init__(self, track):
        super().__init__()
        self.track = track
        self.count = 0

    async def recv(self):
        frame = await self.track.recv()
        img = frame.to_ndarray(format="bgr24")  # Convert frame to NumPy array
        res = onnx_model(img, stream=True)
        try:
            for r in res:
                classes = r.boxes.cls.cpu().detach().numpy().astype(int).tolist()
                boxes = r.boxes.xywh.cpu().detach().numpy().astype(int).tolist()
                await send_message("det", [classes, boxes])
                print(classes)
        except:
            print("fail")
        self.count+=1
        return frame

async def offer(request):
    params = request
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pc_id = "PeerConnection(%s)" % uuid.uuid4()
    pcs.add(pc)


    def log_info(msg, *args):
        logger.info(pc_id + " " + msg, *args)

    # log_info("Created for %s", request.remote)

    @pc.on("datachannel")
    def on_datachannel(channel):
        @channel.on("message")
        def on_message(message):
            if isinstance(message, str) and message.startswith("ping"):
                channel.send("pong" + message[4:])

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print("Connection state is %s", pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    @pc.on("track")
    def on_track(track):
        print("Track %s received", track.kind)

        if track.kind == "video":
            pc.addTrack(
                VideoTransformTrack(
                    relay.subscribe(track)
                )
            )

        @track.on("ended")
        async def on_ended():
            print("Track %s ended", track.kind)

    # handle offer
    await pc.setRemoteDescription(offer)

    # send answer
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
        # warmup
    print("warmup")
    for i in range(30):
        res = onnx_model("/home/alif/yolo/yolov8/bus.jpg", stream=True)
    print("warmup done")
    return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}


async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


############################ ws


client = None
server_loop = None
async def ws_handler(websocket):
    global client
    client = websocket


    print(id(websocket), 'connected')
    await send_message("device_connected", id(websocket))
    try:
        while True:
            message = await websocket.recv()
            data = json.loads(message)
            print(data)
            cmd = data['cmd']

            if cmd == 'stream_offer':
                await streamOfferHandle(data)
    finally:
        del client
        print(id(websocket), 'closed')

async def main():
    async with serve(ws_handler, '0.0.0.0', 8077) as server:
        await server.serve_forever()

async def send_message2(websocket_con, message):
    await websocket_con.send(message)

async def send_message(cmd, params):
    if not client:
        return
    resp = {
        "cmd" : cmd,
        "params" : params
    }
    
    respJson = json.dumps(resp)
    await send_message2(client, respJson)



async def offering(request):
    print("processing offer...")
    
    answer = await offer(request)

    print("ANSWER")
    print(answer)
    print(type(answer))
    
    await send_message("stream_answer", answer)
    print("answer sent")

async def streamOfferHandle(data):
    global server_loop
    await offering(data)


if __name__ == '__main__':

        asyncio.run(main())
