#!/usr/bin/env python3
"""
MyCobot MCP HTTP Server

HTTP transport version of MyCobot MCP Server for Remote MCP.
Designed for ChatGPT and external web-based integrations.
"""

import asyncio
import argparse
import logging
import sys

from aiohttp import web, web_request
from mcp.server.sse import SseServerTransport
from mcp.server.models import InitializationOptions
from mcp.server.lowlevel.server import NotificationOptions

from mycobot_mcp_lib import MyCobotMCPServer

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def create_sse_transport(request: web_request.Request, mcp_server: MyCobotMCPServer) -> web.Response:
    """Create SSE transport for HTTP MCP connection."""
    try:
        # Create basic SSE response for now
        response = web.StreamResponse(
            status=200,
            headers={
                'Content-Type': 'text/event-stream',
                'Cache-Control': 'no-cache',
                'Connection': 'keep-alive',
                'Access-Control-Allow-Origin': '*',
                'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
                'Access-Control-Allow-Headers': 'Content-Type, Authorization',
            }
        )
        
        await response.prepare(request)
        
        # Send initial connection message
        await response.write(b'data: {"type": "ping"}\n\n')
        
        # Keep connection alive
        async def keep_alive():
            try:
                while True:
                    await asyncio.sleep(30)
                    await response.write(b'data: {"type": "ping"}\n\n')
            except Exception as e:
                logger.debug(f"SSE keep-alive ended: {e}")
        
        asyncio.create_task(keep_alive())
        return response
        
    except Exception as e:
        logger.error(f"SSE transport error: {e}")
        return web.Response(status=500, text=f"Transport error: {e}")


async def run_http_server(host: str = "0.0.0.0", port: int = 8081, 
                         api_host: str = "localhost", api_port: int = 8080):
    """Run the MCP server using HTTP transport."""
    mcp_server = MyCobotMCPServer(api_host, api_port)
    
    # Initialize API session
    if not await mcp_server.initialize_api_session():
        logger.error("Failed to connect to API server")
        sys.exit(1)
    
    app = web.Application()
    
    # Add CORS headers for Remote MCP compatibility
    @web.middleware
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
            raise
    
    app.middlewares.append(add_cors_headers)
    
    # SSE endpoint for MCP communication
    async def sse_handler(request):
        return await create_sse_transport(request, mcp_server)
    
    app.router.add_get("/sse", sse_handler)
    app.router.add_post("/sse", sse_handler)
    
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
            # Test API connection
            robot_status = await mcp_server.get_robot_status()
            return web.json_response({
                "status": "healthy", 
                "transport": "http",
                "mcp_server": "mycobot-controller",
                "version": "1.0.0",
                "robot_connected": True,
                "robot_moving": robot_status.get("is_moving", False),
                "endpoints": {
                    "sse": f"http://{host}:{port}/sse",
                    "health": f"http://{host}:{port}/health"
                },
                "capabilities": ["search", "fetch", "robot_control"],
                "description": "MyCobot MCP Server with Remote MCP support for ChatGPT"
            })
        except Exception as e:
            logger.error(f"Health check failed: {e}")
            return web.json_response({
                "status": "degraded",
                "transport": "http", 
                "mcp_server": "mycobot-controller",
                "version": "1.0.0",
                "robot_connected": False,
                "error": str(e),
                "endpoints": {
                    "sse": f"http://{host}:{port}/sse",
                    "health": f"http://{host}:{port}/health"
                }
            }, status=503)
    
    app.router.add_get("/health", health_handler)
    
    # MCP specification endpoint for Remote MCP discovery
    async def mcp_spec_handler(request):
        return web.json_response({
            "mcp_version": "1.0.0",
            "server_name": "mycobot-controller",
            "server_version": "1.0.0",
            "description": "MyCobot robot control server with ChatGPT Remote MCP compatibility",
            "endpoints": {
                "sse": f"http://{host}:{port}/sse"
            },
            "required_tools": ["search", "fetch"],
            "additional_tools": [
                "get_joint_angle", "move_joint", "get_all_joint_angles", 
                "move_all_joints", "home_position", "stop_robot", 
                "jog_joint", "wait_for_completion", "get_robot_status"
            ],
            "prompts": ["robot_status", "joint_info", "basic_movements"],
            "capabilities": {
                "search": "Search through robot resources and capabilities",
                "fetch": "Fetch detailed information about specific robot components",
                "robot_control": "Full 6-DOF robot arm control",
                "real_time_status": "Live robot position and status information"
            },
            "compatible_with": ["ChatGPT", "OpenAI", "Remote MCP clients"]
        })
    
    app.router.add_get("/mcp", mcp_spec_handler)
    app.router.add_get("/.well-known/mcp", mcp_spec_handler)
    
    # Root endpoint - redirect to MCP spec for discoverability
    async def root_handler(request):
        return web.json_response({
            "message": "MyCobot Remote MCP Server",
            "version": "1.0.0",
            "mcp_endpoint": f"http://{host}:{port}/sse",
            "health_check": f"http://{host}:{port}/health",
            "specification": f"http://{host}:{port}/mcp"
        })
    
    app.router.add_get("/", root_handler)
    app.router.add_post("/", root_handler)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()
    
    logger.info("=" * 60)
    logger.info("MyCobot Remote MCP Server Started!")
    logger.info("=" * 60)
    logger.info(f"🌐 HTTP Server: http://{host}:{port}")
    logger.info(f"🔌 SSE Endpoint: http://{host}:{port}/sse")
    logger.info(f"❤️  Health Check: http://{host}:{port}/health")
    logger.info(f"📋 MCP Spec: http://{host}:{port}/mcp")
    logger.info(f"🤖 API Server: {mcp_server.api_base_url}")
    logger.info("=" * 60)
    logger.info("🎯 Ready for ChatGPT Remote MCP connections!")
    logger.info("💡 Add this URL to ChatGPT: http://{host}:{port}/sse".format(host=host, port=port))
    logger.info("=" * 60)
    
    try:
        # Keep the server running
        await asyncio.Event().wait()
    except KeyboardInterrupt:
        logger.info("HTTP server interrupted by user")
    finally:
        await runner.cleanup()
        await mcp_server.cleanup()


def main():
    """Main function to run the HTTP MCP server."""
    parser = argparse.ArgumentParser(description='MyCobot MCP HTTP Server for Remote MCP')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                       help='HTTP server host (default: 0.0.0.0 for external access)')
    parser.add_argument('--port', type=int, default=8081,
                       help='HTTP server port (default: 8081)')
    parser.add_argument('--api-host', type=str, default='localhost',
                       help='MyCobot API server host (default: localhost)')
    parser.add_argument('--api-port', type=int, default=8080,
                       help='MyCobot API server port (default: 8080)')
    
    args = parser.parse_args()
    
    logger.info("Starting MyCobot Remote MCP HTTP Server")
    logger.info(f"HTTP Server: {args.host}:{args.port}")
    logger.info(f"API Server: {args.api_host}:{args.api_port}")
    
    try:
        asyncio.run(run_http_server(args.host, args.port, args.api_host, args.api_port))
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
    except Exception as e:
        logger.error(f"Server failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()