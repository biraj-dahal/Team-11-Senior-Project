from fastapi import FastAPI, WebSocket, Request
import sqlite3
from sqlite3 import Connection
from contextlib import contextmanager
import json
from datetime import datetime


def dict_factory(cursor, row):
    d = {}
    for idx, col in enumerate(cursor.description):
        d[col[0]] = row[idx]
    return d


# App setup
app = FastAPI()
conn = sqlite3.connect("./data/database")
conn.row_factory = dict_factory
datetime_format = "%Y-%m-%d %H:%M:%S"


# context manager for creating db requests closing the cursor
@contextmanager
def query_db(conn: Connection, q: str, params={}):
    c = conn.cursor()
    c.execute(q, params)
    conn.commit()
    yield c.fetchall()
    c.close()


@app.on_event("startup")
async def startup_event():
    query = """
            CREATE TABLE IF NOT EXISTS processed_packages (id INTEGER PRIMARY KEY AUTOINCREMENT,
                                                           perishable INTEGER NOT NULL,
                                                           hazardous INTEGER NOT NULL,
                                                           error INTEGER,
                                                           error_type TEXT,
                                                           error_message TEXT,
                                                           processed_datetime TEXT
                                                           )
            """
    # create process_packages table. Database return is not neccessary so we pass
    with query_db(conn, query, {}) as _:
        pass


def handleProcessedPackage(request_json):
    perishable = 1 if request_json["package"]["perishable"] else 0
    hazardous = 1 if request_json["package"]["hazardous"] else 0
    error = 1 if request_json["package"]["error"] else 0
    error_type = request_json["package"]["error_type"]
    error_message = request_json["package"]["error_message"]
    proccesed_datetime = datetime.strptime(
        request_json["processed_datetime"], datetime_format
    )

    query = """
            INSERT INTO processed_packages (perishable, hazardous, error, error_type, error_message, processed_datetime) VALUES (?,?,?,?,?,?)
    """
    params = (
        perishable,
        hazardous,
        error,
        error_type,
        error_message,
        proccesed_datetime.strftime(datetime_format),
    )

    with query_db(conn, query, params) as _:
        pass


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        try:
            data = await websocket.receive_text()
            request_json = json.loads(data)
        except Exception as e:
            await websocket.send_text(f"Error processing request: {e}")
            continue

        match request_json["type"]:
            case "package_processed":
                try:
                    handleProcessedPackage(request_json)
                except Exception as e:
                    await websocket.send_text(f"Error inserting in database: {e}")
                    continue

                await websocket.send_text("done")
                continue


@app.get("/processed_packages")
async def get_processed_packages(request: Request):
    query = """
        SELECT * FROM processed_packages
    """

    with query_db(conn, query) as r:
        rows = r

    return {"packages": rows}
