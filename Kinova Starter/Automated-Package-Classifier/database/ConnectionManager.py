from fastapi import WebSocket


class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []
        self.ws_identity_index: dict = {}

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    def get_websocket_index(self, websocket: WebSocket):
        return self.active_connections.index(websocket)

    def get_other_ws(self, websocket: WebSocket):
        for ws in self.active_connections:
            if ws != websocket:
                return ws

    def get_ws_from_identity(self, identity: str):
        # TODO: Will give error if identity has not been set
        index = self.ws_identity_index[identity]
        return self.active_connections[index]

    def set_identity_for_ws(self, identity: str, websocket: WebSocket):
        index = self.get_websocket_index(websocket)
        self.ws_identity_index[identity] = index 


    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            await connection.send_text(message)


