from fastapi import FastAPI, Request, HTTPException
from json import JSONDecodeError
import requests
import json

app = FastAPI()

data = {}

@app.get("/")
def read_root():
    return data

@app.post("/json_update")
async def receive_data(request: Request):
    content_type = request.headers.get('Content-Type')
    
    if content_type is None:
        raise HTTPException(status_code=400, detail='No Content-Type provided')
    elif content_type == 'application/json':
        try:
            global data
            data = await request.json()
            url = "http://czechrockets.euweb.cz/set.php"
            response = requests.post(url, json=data, headers={"X-Password": "air"})
            return {"message": "Data received successfully!"}
        except JSONDecodeError:
            raise HTTPException(status_code=400, detail='Invalid JSON data')
    else:
        raise HTTPException(status_code=400, detail='Content-Type not supported')
    
    
