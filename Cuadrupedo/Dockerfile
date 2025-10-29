FROM python:3.11-slim

# Metadata.
LABEL description="Simulación de robot cuadrúpedo Minitaur con PyBullet"
LABEL version="1.0"

# Variables de entorno
ENV PYTHONUNBUFFERED=1
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0

# Instalar dependencias del sistema para PyBullet y OpenGL
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    g++ \
    libgl1 \
    libgl1-mesa-dri \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libgomp1 \
    mesa-utils \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copiar requirements e instalar dependencias Python
COPY requirements.txt .
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Copiar el código
COPY minitaur.py .

# Comando por defecto: ejecutar marcha automática
CMD ["python", "minitaur.py", "1"]
