# get latest python 
FROM python:3.11-slim

# set working directory
WORKDIR /code

# copy requirements file
COPY ./requirements.txt /code/requirements.txt

# install required modules
RUN pip install --no-cache-dir --upgrade -r /code/requirements.txt

# copy application
COPY ./app /code/app

# run application
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "80", "--reload"]