FROM ubuntu:24.04

RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    openscad \
    meshlab \
    vim
RUN pip install onshape-to-robot --break-system-packages

WORKDIR /workspace

# These will be passed at runtime instead of build time
ENV ONSHAPE_ACCESS_KEY=""
ENV ONSHAPE_SECRET_KEY=""
ENV DOCUMENT_ID=""

CMD ["python3", "run_onshape_conversion.py"]
