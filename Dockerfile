# MyCobot MCP Server Docker Image
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the MCP server
COPY mycobot_mcp_server.py .

# Create non-root user for security
RUN useradd --create-home --shell /bin/bash mcpuser && \
    chown -R mcpuser:mcpuser /app
USER mcpuser

# Set environment variables
ENV PYTHONUNBUFFERED=1

# Expose stdin/stdout for MCP communication
ENTRYPOINT ["python", "mycobot_mcp_server.py"]

# Default arguments (can be overridden)
CMD ["--api-host", "host.docker.internal", "--api-port", "8080"]