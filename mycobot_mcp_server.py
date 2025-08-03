"""
MyCobot MCP Server

Model Context Protocol (MCP) server for MyCobot robot control.
Provides tools for external systems to control the robot through REST API.

This version maintains backward compatibility while using the new library structure.
Now includes ChatGPT Remote MCP compatibility with search and fetch tools.
"""

import asyncio
import argparse
import logging
import sys
from typing import Optional

from mcp.server.stdio import stdio_server
from mcp.server.sse import SseServerTransport
from mcp.server.models import InitializationOptions
from mcp.server.lowlevel.server import NotificationOptions
from aiohttp import web, web_request

from mycobot_mcp_lib import MyCobotMCPServer

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# Global MCP server instance
mcp_server_instance: Optional[MyCobotMCPServer] = None


async def run_stdio_server(api_host: str = "localhost", api_port: int = 8080):
    """Run the MCP server using STDIO transport."""
    global mcp_server_instance
    mcp_server_instance = MyCobotMCPServer(api_host, api_port)
    
    # Initialize API session
    if not await mcp_server_instance.initialize_api_session():
        logger.error("Failed to connect to API server")
        sys.exit(1)
    
    try:
        async with stdio_server() as (read_stream, write_stream):
            await mcp_server_instance.server.run(
                read_stream,
                write_stream,
                InitializationOptions(
                    server_name="mycobot-controller",
                    server_version="1.0.0",
                    capabilities=mcp_server_instance.server.get_capabilities(
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
        if mcp_server_instance:
            await mcp_server_instance.cleanup()


async def create_sse_transport(request: web_request.Request) -> web.Response:
    """Create SSE transport for HTTP MCP connection."""
    global mcp_server_instance
    if not mcp_server_instance:
        return web.Response(status=500, text="MCP server not initialized")
    
    try:
        transport = SseServerTransport("/message", request)
        
        async def _run_server():
            await mcp_server_instance.server.run(
                transport,
                InitializationOptions(
                    server_name="mycobot-controller",
                    server_version="1.0.0",
                    capabilities=mcp_server_instance.server.get_capabilities(
                        notification_options=NotificationOptions(),
                        experimental_capabilities={}
                    )
                )
            )
        
        # Start the server in a background task
        asyncio.create_task(_run_server())
        return await transport.start()
    except Exception as e:
        logger.error(f"SSE transport error: {e}")
        return web.Response(status=500, text=f"Transport error: {e}")


async def run_http_server(host: str = "localhost", port: int = 8081, 
                         api_host: str = "localhost", api_port: int = 8080):
    """Run the MCP server using HTTP transport."""
    global mcp_server_instance
    mcp_server_instance = MyCobotMCPServer(api_host, api_port)
    
    # Initialize API session
    if not await mcp_server_instance.initialize_api_session():
        logger.error("Failed to connect to API server")
        sys.exit(1)
    
    app = web.Application()
    
    # Add CORS headers
    async def add_cors_headers(request, handler):
        try:
            response = await handler(request)
            response.headers["Access-Control-Allow-Origin"] = "*"
            response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
            response.headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization"
            response.headers["Access-Control-Allow-Credentials"] = "true"
            return response
        except Exception as e:
            logger.error(f"CORS middleware error: {e}")
            return web.Response(status=500, text=f"Middleware error: {e}")
    
    app.middlewares.append(add_cors_headers)
    
    # SSE endpoint for MCP communication
    app.router.add_get("/sse", create_sse_transport)
    
    # Options handler for CORS preflight
    async def options_handler(request):
        return web.Response(
            headers={
                "Access-Control-Allow-Origin": "*",
                "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
                "Access-Control-Allow-Headers": "Content-Type, Authorization",
                "Access-Control-Allow-Credentials": "true"
            }
        )
    
    app.router.add_options("/sse", options_handler)
    app.router.add_options("/{path:.*}", options_handler)
    
    # Health check endpoint
    async def health_handler(request):
        try:
            robot_status = await mcp_server_instance.get_robot_status()
            return web.json_response({
                "status": "healthy", 
                "transport": "http",
                "mcp_server": "mycobot-controller",
                "version": "1.0.0",
                "robot_connected": True,
                "robot_moving": robot_status.get("is_moving", False),
                "capabilities": ["search", "fetch", "robot_control"],
                "description": "MyCobot MCP Server (backward compatible version)"
            })
        except Exception as e:
            return web.json_response({
                "status": "degraded",
                "transport": "http",
                "error": str(e)
            }, status=503)
    
    app.router.add_get("/health", health_handler)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()
    
    logger.info(f"HTTP MCP server started on {host}:{port}")
    logger.info(f"SSE endpoint: http://{host}:{port}/sse")
    logger.info(f"Health check: http://{host}:{port}/health")
    logger.info("Now includes ChatGPT Remote MCP compatibility!")
    
    try:
        await asyncio.Event().wait()
    except KeyboardInterrupt:
        logger.info("HTTP server interrupted by user")
    finally:
        await runner.cleanup()
        if mcp_server_instance:
            await mcp_server_instance.cleanup()


def main():
    """Main function to run the MCP server."""
    parser = argparse.ArgumentParser(
        description='MyCobot MCP Server (Backward Compatible with Remote MCP Support)'
    )
    parser.add_argument('--api-host', type=str, default='localhost',
                       help='MyCobot API server host (default: localhost)')
    parser.add_argument('--api-port', type=int, default=8080,
                       help='MyCobot API server port (default: 8080)')
    parser.add_argument('--transport', type=str, choices=['stdio', 'http'], default='stdio',
                       help='Transport protocol: stdio or http (default: stdio)')
    parser.add_argument('--http-host', type=str, default='localhost',
                       help='HTTP server host for http transport (default: localhost)')
    parser.add_argument('--http-port', type=int, default=8081,
                       help='HTTP server port for http transport (default: 8081)')
    
    args = parser.parse_args()
    
    logger.info("=" * 60)
    logger.info("MyCobot MCP Server (Enhanced with Remote MCP)")
    logger.info("=" * 60)
    logger.info(f"API Server: {args.api_host}:{args.api_port}")
    logger.info(f"Transport: {args.transport}")
    if args.transport == 'http':
        logger.info(f"HTTP Server: {args.http_host}:{args.http_port}")
    logger.info("New Features: search/fetch tools for ChatGPT compatibility")
    logger.info("=" * 60)
    
    try:
        if args.transport == 'stdio':
            logger.info("Starting MCP server with STDIO transport")
            asyncio.run(run_stdio_server(args.api_host, args.api_port))
        elif args.transport == 'http':
            logger.info(f"Starting MCP server with HTTP transport on {args.http_host}:{args.http_port}")
            asyncio.run(run_http_server(args.http_host, args.http_port, args.api_host, args.api_port))
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
    except Exception as e:
        logger.error(f"Server failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()