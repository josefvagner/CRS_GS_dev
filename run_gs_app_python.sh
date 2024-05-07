#!/bin/bash
cd /home/gs/Desktop/CRS_GS_dev/RPI
source .venv/bin/activate
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload