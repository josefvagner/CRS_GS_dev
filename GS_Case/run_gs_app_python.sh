#!/bin/bash
cd /home/pi/avionics/cansat/SW/GSW/GS_Case
source .venv/bin/activate
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload