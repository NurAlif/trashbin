import asyncio
import json
import os
import fractions
import cv2
import queue
import threading
from typing import Tuple
import time
from websockets.asyncio.client import connect
import websockets
import serial

from av import VideoFrame

from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack

frame_width=640
frame_height=480

pcs = set()

video = None

# ser = serial.Serial(port='/dev/cu.usbserial-1240',baudrate=9600)



AUDIO_PTIME = 0.020  # 20ms audio packetization
VIDEO_CLOCK_RATE = 900
VIDEO_PTIME = 1 / 5  # 30fps
VIDEO_TIME_BASE = fractions.Fraction(1, VIDEO_CLOCK_RATE)

# 65 70 75 80 85


class VideoCapture:
    def __init__(self):
        self.frameOut = None
        self.frameIn = None
        self.cap = None
        self.testing = True
        self.q = None
        if self.testing:
            self.cap = cv2.VideoCapture('/mnt/d/backuop/109.avi')
        else:
            self.cap = cv2.VideoCapture(0)
            self.q = queue.Queue()
            t = threading.Thread(target=self._reader)
            t.daemon = True
            t.start()

    # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            self.q.put(frame)

    def read_in(self):
        if self.testing:
            ret, frame = self.cap.read()
            if not ret:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                print("frame end")
            return frame
        else:
            frame = self.q.get()
            return frame
        
    def store_out(self, frame):
        # print("store out")
        self.frameIn = frame
            
    def read_out(self):
        return self.frameIn

    def release(self):
        self.cap.release()


class VideoOpencvTrack(VideoStreamTrack):
    def __init__(self, cap):
        super().__init__()
        self.cap = cap

    _start: float
    _timestamp: int
    
    def play_live(self):
        self.cap_replay.release()
        self.replay = False

    def get_frame_size(self):
        # hw fliped
        width, height, _ = self.cap.read_in().shape
        print("frame size : ", height, ", ", width)
        return (width, height)

    async def next_timestamp(self) -> Tuple[int, fractions.Fraction]:
        if self.readyState != "live":
            print("ERROR track unready!")
            return None

        if hasattr(self, "_timestamp"):
            self._timestamp += int(VIDEO_PTIME * VIDEO_CLOCK_RATE)
            wait = self._start + (self._timestamp / VIDEO_CLOCK_RATE) - time.time()
            await asyncio.sleep(wait)
        else:
            self._start = time.time()
            self._timestamp = 0
        return self._timestamp, VIDEO_TIME_BASE

    async def recv(self):
        src = self.cap.read_out()
        if src is None:
            print("emptyframe")
            return None
        # img = cv2.rotate(src, cv2.ROTATE_90_COUNTERCLOCKWISE)


        pts, time_base = await self.next_timestamp()

        new_frame = VideoFrame.from_ndarray(src, format="bgr24")
        new_frame.pts = pts
        new_frame.time_base = time_base
        print("recv")
        return new_frame

pc = None
async def create_offer():
    global video
    global pc
    print("processing answer...")
    # print(params["sdp"])
    pc = RTCPeerConnection()
    pcs.add(pc)

    video_track = VideoOpencvTrack(video)
    pc.addTrack(video_track)

    @pc.on("datachannel")
    def on_datachannel(channel):
        @channel.on("message")
        def on_message(message):
            if isinstance(message, str) and message.startswith("ping"):
                channel.send("pong" + message[4:])

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print("Connection state is %s" % pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # if video:
    #     pc.addTrack(video)

    offer = await pc.createOffer()
    await pc.setLocalDescription(offer)

    print("remote description has been set")

    return {"cmd" : "stream_offer", "sdp": pc.localDescription.sdp, "type": pc.localDescription.type}

class ReaderFrames:
    def __init__(self):
        self.loop = asyncio.get_event_loop()

    async def _frameReader(self):
        while True:
            print("readin")
            await asyncio.sleep(0.5)
            if video != None:
                video.store_out(video.read_in())
    
    def frameReader(self):
        asyncio.ensure_future(self._frameReader())

class ServerThread:
    def __init__(self):
        self.loop = asyncio.get_event_loop()
        self.accumulation     = [0,0,0,0]
        self.identified_trash = None
        self.identified_trash_safe_count = 0
        self.bb_to_size_const = 1.65
        self.trigger = None
        self.timeStarted = 0
        self.timeWindow = 2

    async def _server(self, uri):
        try:
            async with websockets.connect(uri) as websocket:
                # Send an initial message to the server
                await websocket.send(json.dumps({"cmd":"test"}))
                print("Initial message sent to server.")
                await websocket.send(json.dumps(await create_offer()))

                while True:
                    try:
                        message = await websocket.recv()
                        data = json.loads(message)
                        # print(data)
                        if(data["cmd"] == "stream_answer"):
                            params = data["params"]
                            offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
                            await pc.setRemoteDescription(offer)
                        if(data["cmd"] == "det"):

                            if self.trigger != None:
                                if time.time() - self.timeStarted > self.timeWindow:
                                    cls_idx = self.trigger[0]
                                    self.accumulation[cls_idx] += self.trigger[1]
                                    self.trigger = None
                                    print(" finish")
                            else:
                                params = data["params"]
                                print(params)
                                classes = params[0]
                                box_sizes = [(c, i[2]*i[3]) for (c,i) in zip(classes, params[1])]
                                # print(box_sizes)
                                if len(classes) > 0:
                                    largest = None
                                    for box in box_sizes:
                                        if largest == None: 
                                            largest = box
                                        elif largest[1] < box[1] :
                                            largest = box
                                    if largest != None:
                                        if self.identified_trash == None:
                                            self.identified_trash = largest
                                        else:
                                            self.identified_trash_safe_count += 1
                                            if self.identified_trash_safe_count >= 3:
                                                self.identified_trash_safe_count = 0
                                                self.timeStarted = time.time()
                                                self.trigger = self.identified_trash
                                                self.identified_trash = None
                                                # trigger open
                                                byte = 70 + self.trigger[0]*5
                                                print(byte)
                                                # ser.write(byte)

                    except websockets.ConnectionClosed:
                        print("Connection closed by the server.")
                        break
        except Exception as e:
            print(f"An error occurred: {e}")
    
    def server(self, url):
        asyncio.ensure_future(self._server(url))

if __name__ == "__main__":
    # uri = "ws://localhost:8077"
    uri = "ws://ai-ndhu-lab:8077"
    video = VideoCapture()

    rf = ReaderFrames()
    st = ServerThread()

    t1 = threading.Thread(target=asyncio.run, args=(st.server(uri)))
    t2 = threading.Thread(target=asyncio.run, args=(rf.frameReader()))
    
    # rf.loop.run_until_complete()
    st.loop.run_forever()

    t1.start()
    t2.start()

    t1.join()
    t2.join()
        
