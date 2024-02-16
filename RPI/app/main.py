from fastapi import FastAPI, Request, HTTPException, BackgroundTasks
from json import JSONDecodeError
import requests
from time import time_ns
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE"],
    allow_headers=["Authorization", "Content-Type"]
)

data = {}
taskRunnig = False
url = "http://czechrockets.euweb.cz/set.php"
t0 = 0

def dbUpdate():
    global taskRunnig, data, url
    try:
        response = requests.post(url, json=data, headers={"X-Password": "air"})
        print("dbUpdate: ", end="")
        print(response)
    except:
        print("dbUpdate: error")
    taskRunnig = False

@app.get("/")
def read_root():
    return data

@app.post("/json_update")
async def receive_data(request: Request, bg: BackgroundTasks):
    global data, taskRunnig, t0
    content_type = request.headers.get('Content-Type')
    
    if content_type is None:
        raise HTTPException(status_code=400, detail='No Content-Type provided')
    elif content_type == 'application/json':
        try:
            data = await request.json()
        except JSONDecodeError:
            raise HTTPException(status_code=400, detail='Invalid JSON data')
    else:
        raise HTTPException(status_code=400, detail='Content-Type not supported')
    
    if not taskRunnig:
        bg.add_task(dbUpdate)
        taskRunnig = True
    print("elapsed time = " + str((time_ns()-t0)/1e6))
    t0 = time_ns()
    return {"message": "Data received successfully!"}
    
    
