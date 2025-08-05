import logging
import json 
import asyncio
import websockets 
from websockets.server import serve


logging.basicConfig(level=logging.INFO, format=':%(levelname)s:%(message)s') 

class Server:
    def __init__(self, port,notify = print, gather_info =input):
        self.logger = logging.getLogger(__name__)
        self.notify = notify
        self.gather_info = gather_info
        with open(r"communication/scripts/com_vars.json", "r") as config:
            config = json.load(config)
            self.host = config['server_ip']
            self.port = config[port]
            self.valid_token = config["com_token"]

        self.stop_server = False
        self.buffer = None 
    
    def kill(self):
        self.stop_server = True

    
    async def recv_loop(self, websocket):
        try:
            while not self.stop_server:
                message = await websocket.recv()
                self.buffer = message
                self.logger.debug(f"Received message: {message}")
                if message != "":
                    self.notify()
        except Exception as e:
            self.logger.error(f"Recv loop error: {e}")

    async def send_loop(self, websocket):
        try:
            while not self.stop_server:
                to_send = await asyncio.to_thread(self.gather_info)
                if to_send:
                    await websocket.send(to_send)
                await asyncio.sleep(0.01)  
        except Exception as e:
            self.logger.error(f"Send loop error: {e}")

    async def handler(self, websocket):
        self.logger.info(f"Client connected: {websocket.remote_address}")
        
        path = websocket.request.path
        if "?" in path:
            query = dict(p.split('=') for p in path.split('?')[1].split('&'))
        else:
            query = {}

        if query.get("token") != self.valid_token:
            self.logger.warning("Unauthorized client loooooool")
            await websocket.close()
            return
        self.logger.info(f"Authorized client: {websocket.remote_address}")

        recv_task = asyncio.create_task(self.recv_loop(websocket))
        send_task = asyncio.create_task(self.send_loop(websocket))

        done, pending = await asyncio.wait(
            [recv_task, send_task], return_when=asyncio.FIRST_COMPLETED
        )                       

    async def start(self):

        async with websockets.serve( self.handler, self.host, self.port):
            self.logger.info(f"Server started on {self.host}:{self.port}")
            await asyncio.Future()

if __name__ == "__main__":
    server = Server("communication_port")
    asyncio.run(server.start())