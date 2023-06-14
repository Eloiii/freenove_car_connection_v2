# syntax=docker/dockerfile:1
FROM python:3.11.4

ENV DEBIAN_FRONTEND=noninteractive
LABEL authors="Lony Riffard, Eloi Charra"

WORKDIR /app
RUN apt update && apt-get install -y build-essential libcap-dev libglib2.0-0 libsm6 libxrender-dev libxext6 libgl1-mesa-glx libegl1 && rm -rf /var/lib/apt/lists/*

COPY requirements.txt requirements.txt
RUN /usr/local/bin/python -m pip install --upgrade pip
RUN pip3 install -r requirements.txt

COPY . .

CMD ["python3", "-m" , "flask", "run", "--host=0.0.0.0"]
