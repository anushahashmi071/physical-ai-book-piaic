# Use smaller Python base image
FROM python:3.9-slim

WORKDIR /app

# Install only essential system dependencies and clean up
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    g++ \
    curl \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /var/cache/apt/*

# Copy requirements first to leverage Docker cache
COPY requirements.txt .

# Install Python dependencies without cache and exclude heavy optional deps
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt && \
    pip cache purge

# Download embedding model separately (will be cached)
RUN python -c "from sentence_transformers import SentenceTransformer; SentenceTransformer('all-MiniLM-L6-v2')" || true

# Create non-root user for security
RUN groupadd -r raguser && useradd -r -g raguser raguser

# Create directories and set permissions
RUN mkdir -p /app/logs /app/data && \
    chown -R raguser:raguser /app

# Switch to non-root user
USER raguser

# Copy application code (exclude large data files)
COPY --chown=raguser:raguser main.py logging_config.py validation.py metrics.py ./
COPY --chown=raguser:raguser config/ ./config/
COPY --chown=raguser:raguser models/ ./models/
COPY --chown=raguser:raguser routes/ ./routes/
COPY --chown=raguser:raguser services/ ./services/
COPY --chown=raguser:raguser utils/ ./utils/

# Expose port (use PORT env variable, default 8000)
ARG PORT=8000
ENV PORT=${PORT}
EXPOSE ${PORT}

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:${PORT}/health || exit 1

# Run the application with fewer workers to reduce memory
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000", "--workers", "2"]
