from fastapi import FastAPI, Request, HTTPException, BackgroundTasks
from json import JSONDecodeError, dumps
import requests
from time import time_ns
from fastapi.middleware.cors import CORSMiddleware
import logging
from datetime import datetime

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE"],
    allow_headers=["Authorization", "Content-Type"],
)

data = {}
taskRunnig = False
url = "http://czechrockets.euweb.cz/set.php"
dd = datetime.now().strftime("%d/%m/%Y %H:%M:%S")

logging.basicConfig(
    level=logging.INFO, format="%(message)s", filename="fastApi.json", filemode="a"
)


def dbUpdate():
    global taskRunnig, data, url
    try:
        response = requests.post(url, json=data, headers={"X-Password": "air"})
        print("dbUpdate: ", end="")
        print(response)
    except:
        print("dbUpdate: error")
    taskRunnig = False


def logData():
    logging.info(dumps(data) + ",")


@app.get("/")
def read_root():
    data["timestamp_gs"] = datetime.now().strftime("%H:%M:%S")
    return data


@app.post("/json_update")
async def receive_data(request: Request, bg: BackgroundTasks):
    global data, taskRunnig, t0
    content_type = request.headers.get("Content-Type")

    if content_type is None:
        raise HTTPException(status_code=400, detail="No Content-Type provided")
    elif content_type == "application/json":
        try:
            data = await request.json()
            logData()
        except JSONDecodeError:
            raise HTTPException(status_code=400, detail="Invalid JSON data")
    else:
        raise HTTPException(status_code=400, detail="Content-Type not supported")

    if not taskRunnig:
        bg.add_task(dbUpdate)
        taskRunnig = True
    return {"message": "Data received successfully!"}
