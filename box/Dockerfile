FROM python:3.13-alpine
WORKDIR /app
COPY taploc.py server.py index.html ./
ENV PORT=8000
CMD ["python", "server.py"]
