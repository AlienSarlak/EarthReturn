FROM python:3.12.5-alpine3.19

ENV DEBIAN_FRONTEND=noninteractive
RUN apk update && apk add --no-cache \
    git \
    curl \
    build-base

RUN python3 -m pip install --upgrade pip

WORKDIR /app

COPY . /app
# COPY requirements.txt /app/
# RUN pip install -r requirements.txt

# Startup command
# CMD ["python3", "src/rocket_sim.py"]

CMD ["/bin/sh"]
