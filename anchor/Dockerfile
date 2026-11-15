FROM python:3.12-slim
RUN apt-get update && apt-get install -y --no-install-recommends libusb-1.0-0 libportaudio2 pipewire-alsa libasound2-plugins && rm -rf /var/lib/apt/lists/* \
 && pip install --no-cache-dir pyserial==3.5 pyusb==1.2.1 numpy==2.1.3 sounddevice==0.5.1
WORKDIR /app
COPY anchor.py clocks.py usbserial.py gpioline.py tca9538.py enstream.py mesh.py mesh.html locate.py ranging.py ./
ENV PYTHONUNBUFFERED=1
CMD ["python", "anchor.py"]
