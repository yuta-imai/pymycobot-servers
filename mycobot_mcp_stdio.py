#!/usr/bin/env python3
"""
MyCobot MCP STDIO Server

STDIO transport version of MyCobot MCP Server.
Maintains compatibility with existing pipe-based MCP clients.
"""

import asyncio
import argparse
import logging
import sys

from mcp.server.stdio import stdio_server
from mcp.server.models import InitializationOptions
from mcp.server.lowlevel.server import NotificationOptions

from mycobot_mcp_lib import MyCobotMCPServer

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def run_stdio_server(api_host: str = "localhost", api_port: int = 8080):
    """Run the MCP server using STDIO transport."""
    mcp_server = MyCobotMCPServer(api_host, api_port)
    
    # Initialize API session
    if not await mcp_server.initialize_api_session():
        logger.error("Failed to connect to API server")
        sys.exit(1)
    
    try:
        async with stdio_server() as (read_stream, write_stream):
            await mcp_server.server.run(
                read_stream,
                write_stream,
                InitializationOptions(
                    server_name="mycobot-controller",
                    server_version="1.0.0",
                    capabilities=mcp_server.server.get_capabilities(
                        notification_options=NotificationOptions(),
                        experimental_capabilities={}
                    )
                )
            )
    except KeyboardInterrupt:
        logger.info("Server interrupted by user")
    except Exception as e:
        logger.error(f"Server error: {e}")
        raise
    finally:
        await mcp_server.cleanup()


def main():
    """Main function to run the STDIO MCP server."""
    parser = argparse.ArgumentParser(description='MyCobot MCP STDIO Server')
    parser.add_argument('--api-host', type=str, default='localhost',
                       help='MyCobot API server host (default: localhost)')
    parser.add_argument('--api-port', type=int, default=8080,
                       help='MyCobot API server port (default: 8080)')
    
    args = parser.parse_args()
    
    logger.info("Starting MyCobot MCP server with STDIO transport")
    logger.info(f"API server: {args.api_host}:{args.api_port}")
    
    try:
        asyncio.run(run_stdio_server(args.api_host, args.api_port))
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
    except Exception as e:
        logger.error(f"Server failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()