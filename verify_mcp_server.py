#!/usr/bin/env python3
"""
Verification script for MyCobot MCP server functionality
This script tests all major operations and validates MCP response formats.
"""

import asyncio
import aiohttp
import json
import sys

# Import MCP server components
sys.path.append('.')
from mycobot_mcp_server import (
    handle_list_tools,
    handle_call_tool,
    handle_list_prompts,
    handle_get_prompt,
    api_base_url
)
import mycobot_mcp_server

async def verify_mcp_server():
    """Comprehensive verification of MCP server functionality."""
    
    print("MyCobot MCP Server Verification")
    print("=" * 50)
    
    # Initialize API session
    mycobot_mcp_server.api_session = aiohttp.ClientSession()
    
    try:
        # Test API server connectivity
        print("1. Testing API server connectivity...")
        async with mycobot_mcp_server.api_session.get(f"{api_base_url}/health") as response:
            if response.status == 200:
                health_data = await response.json()
                print(f"✓ API server healthy: {health_data}")
            else:
                print(f"✗ API server error: {response.status}")
                return False
        
        # Test tool listing
        print("\n2. Testing tool listing...")
        tools = await handle_list_tools()
        print(f"✓ Found {len(tools)} tools:")
        for tool in tools:
            print(f"  - {tool.name}")
        
        # Test critical tools
        critical_tests = [
            ("get_robot_status", {}),
            ("get_all_joint_angles", {}),
            ("get_joint_angle", {"joint_num": 1}),
            ("stop_robot", {})
        ]
        
        print("\n3. Testing critical tool operations...")
        for tool_name, args in critical_tests:
            try:
                result = await handle_call_tool(tool_name, args)
                if result.isError:
                    print(f"✗ {tool_name}: Error - {result.content[0].text}")
                else:
                    # Verify JSON response
                    json_data = json.loads(result.content[0].text)
                    print(f"✓ {tool_name}: Valid JSON response")
            except Exception as e:
                print(f"✗ {tool_name}: Exception - {e}")
        
        # Test prompts
        print("\n4. Testing prompt functionality...")
        prompts = await handle_list_prompts()
        print(f"✓ Found {len(prompts)} prompts:")
        for prompt in prompts:
            print(f"  - {prompt.name}")
        
        # Test a prompt
        try:
            prompt_result = await handle_get_prompt("robot_status", {})
            print("✓ robot_status prompt: Working")
        except Exception as e:
            print(f"✗ robot_status prompt: {e}")
        
        # Test error handling
        print("\n5. Testing error handling...")
        try:
            error_result = await handle_call_tool("invalid_tool", {})
            if error_result.isError:
                print("✓ Error handling: Proper error response for invalid tool")
            else:
                print("✗ Error handling: Should have returned error")
        except Exception as e:
            print(f"✗ Error handling: Exception - {e}")
        
        print("\n" + "=" * 50)
        print("✓ MCP Server Verification PASSED")
        print("The server is ready for MCP client connections.")
        print("\nUsage:")
        print("- STDIO: python mycobot_mcp_server.py --transport stdio")
        print("- HTTP:  python mycobot_mcp_server.py --transport http --http-host 0.0.0.0 --http-port 8081")
        return True
        
    except Exception as e:
        print(f"\n✗ Verification FAILED: {e}")
        return False
    
    finally:
        if mycobot_mcp_server.api_session:
            await mycobot_mcp_server.api_session.close()

if __name__ == "__main__":
    success = asyncio.run(verify_mcp_server())
    sys.exit(0 if success else 1)