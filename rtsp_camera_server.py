"""
RTSP Camera Streaming Server for Raspberry Pi

This module provides an RTSP streaming server that captures video from
the Raspberry Pi camera module (/dev/video0) and streams it over RTSP.
"""

import cv2
import threading
import socket
import time
import logging
from typing import Optional, Tuple
import subprocess
import os
import signal
import sys
import argparse


class RTSPCameraServer:
    """RTSP streaming server for Raspberry Pi camera module."""
    
    def __init__(self, 
                 camera_device: str = "/dev/video0",
                 rtsp_port: int = 8554,
                 stream_name: str = "camera",
                 width: int = 640,
                 height: int = 480,
                 fps: int =5,
                 bitrate: int = 1000000,
                 bind_address: str = "0.0.0.0"):
        """
        Initialize RTSP camera server.
        
        Args:
            camera_device: Camera device path (default: /dev/video0)
            rtsp_port: RTSP server port (default: 8554)
            stream_name: Stream name for RTSP URL (default: camera)
            width: Video width in pixels
            height: Video height in pixels
            fps: Frames per second
            bitrate: Video bitrate in bps
            bind_address: Address to bind server to (default: 0.0.0.0 for external access)
        """
        self.camera_device = camera_device
        self.rtsp_port = rtsp_port
        self.stream_name = stream_name
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate
        self.bind_address = bind_address
        
        self.cap = None
        self.streaming = False
        self.rtsp_process = None
        
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # RTSP URL
        self.rtsp_url = f"rtsp://{self.bind_address}:{rtsp_port}/{stream_name}"
        
    def _check_camera_available(self) -> bool:
        """Check if camera device is available."""
        return os.path.exists(self.camera_device)
    
    def _initialize_camera(self) -> bool:
        """Initialize camera capture."""
        try:
            self.cap = cv2.VideoCapture(self.camera_device)
            if not self.cap.isOpened():
                self.logger.error(f"Failed to open camera device: {self.camera_device}")
                return False
            
            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # Verify settings
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            
            self.logger.info(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps}fps")
            return True
            
        except Exception as e:
            self.logger.error(f"Error initializing camera: {e}")
            return False
    
    def _start_ffmpeg_rtsp_server(self) -> bool:
        """Start FFmpeg RTSP server process."""
        try:
            # FFmpeg command for RTSP streaming
            ffmpeg_cmd = [
                'ffmpeg',
                '-f', 'rawvideo',
                '-vcodec', 'rawvideo',
                '-s', f'{self.width}x{self.height}',
                '-pix_fmt', 'bgr24',
                '-r', str(self.fps),
                '-i', '-',
                '-c:v', 'libx264',
                '-preset', 'ultrafast',
                '-tune', 'zerolatency',
                '-b:v', str(self.bitrate),
                '-maxrate', str(self.bitrate),
                '-bufsize', str(self.bitrate * 2),
                '-pix_fmt', 'yuv420p',
                '-g', str(self.fps * 2),
                '-f', 'rtsp',
                f'rtsp://{self.bind_address}:{self.rtsp_port}/{self.stream_name}'
            ]
            
            self.logger.info(f"Starting FFmpeg RTSP server: {' '.join(ffmpeg_cmd)}")
            
            self.rtsp_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            time.sleep(2)  # Allow FFmpeg to start
            
            if self.rtsp_process.poll() is None:
                self.logger.info(f"RTSP server started on {self.rtsp_url}")
                return True
            else:
                self.logger.error("FFmpeg process failed to start")
                return False
                
        except Exception as e:
            self.logger.error(f"Error starting FFmpeg RTSP server: {e}")
            return False
    
    def _stream_frames(self):
        """Stream camera frames to FFmpeg process."""
        frame_interval = 1.0 / self.fps
        last_frame_time = 0
        
        while self.streaming:
            current_time = time.time()
            
            # Control frame rate
            if current_time - last_frame_time < frame_interval:
                time.sleep(0.001)
                continue
            
            ret, frame = self.cap.read()
            if not ret:
                self.logger.warning("Failed to read frame from camera")
                time.sleep(0.1)
                continue
            
            try:
                # Send frame to FFmpeg
                if self.rtsp_process and self.rtsp_process.stdin:
                    self.rtsp_process.stdin.write(frame.tobytes())
                    self.rtsp_process.stdin.flush()
                    last_frame_time = current_time
                else:
                    self.logger.error("FFmpeg process stdin not available")
                    break
                    
            except Exception as e:
                self.logger.error(f"Error writing frame to FFmpeg: {e}")
                break
    
    def start_streaming(self) -> bool:
        """Start RTSP streaming server."""
        if self.streaming:
            self.logger.warning("Streaming already active")
            return True
        
        # Check camera availability
        if not self._check_camera_available():
            self.logger.error(f"Camera device not found: {self.camera_device}")
            return False
        
        # Initialize camera
        if not self._initialize_camera():
            return False
        
        # Start FFmpeg RTSP server
        if not self._start_ffmpeg_rtsp_server():
            return False
        
        # Start streaming
        self.streaming = True
        self.stream_thread = threading.Thread(target=self._stream_frames, daemon=True)
        self.stream_thread.start()
        
        self.logger.info(f"RTSP streaming started: {self.rtsp_url}")
        return True
    
    def stop_streaming(self):
        """Stop RTSP streaming server."""
        self.logger.info("Stopping RTSP streaming...")
        
        self.streaming = False
        
        # Stop stream thread
        if hasattr(self, 'stream_thread') and self.stream_thread.is_alive():
            self.stream_thread.join(timeout=2)
        
        # Close camera
        if self.cap:
            self.cap.release()
            self.cap = None
        
        # Stop FFmpeg process
        if self.rtsp_process:
            try:
                self.rtsp_process.stdin.close()
                self.rtsp_process.terminate()
                self.rtsp_process.wait(timeout=5)
            except Exception as e:
                self.logger.warning(f"Error stopping FFmpeg process: {e}")
                try:
                    self.rtsp_process.kill()
                except:
                    pass
            finally:
                self.rtsp_process = None
        
        self.logger.info("RTSP streaming stopped")
    
    def is_streaming(self) -> bool:
        """Check if streaming is active."""
        return self.streaming and (self.rtsp_process is not None and self.rtsp_process.poll() is None)
    
    def get_stream_url(self) -> str:
        """Get RTSP stream URL."""
        return self.rtsp_url
    
    def get_stream_info(self) -> dict:
        """Get streaming information."""
        return {
            'url': self.rtsp_url,
            'resolution': f'{self.width}x{self.height}',
            'fps': self.fps,
            'bitrate': self.bitrate,
            'streaming': self.is_streaming()
        }


class SimpleRTSPServer:
    """Simplified RTSP server using GStreamer pipeline."""
    
    def __init__(self, 
                 camera_device: str = "/dev/video0",
                 rtsp_port: int = 8554,
                 width: int = 640,
                 height: int = 480,
                 fps: int = 30):
        """
        Initialize simple RTSP server using GStreamer.
        
        Args:
            camera_device: Camera device path
            rtsp_port: RTSP server port
            width: Video width
            height: Video height
            fps: Frames per second
        """
        self.camera_device = camera_device
        self.rtsp_port = rtsp_port
        self.width = width
        self.height = height
        self.fps = fps
        self.gst_process = None
        
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
    
    def start_gstreamer_rtsp(self) -> bool:
        """Start GStreamer RTSP server."""
        try:
            # GStreamer pipeline for RTSP streaming
            gst_cmd = [
                'gst-launch-1.0',
                '-v',
                f'v4l2src device={self.camera_device}',
                '!', f'video/x-raw,width={self.width},height={self.height},framerate={self.fps}/1',
                '!', 'videoconvert',
                '!', 'x264enc tune=zerolatency bitrate=1000 speed-preset=superfast',
                '!', 'rtph264pay config-interval=1 pt=96',
                '!', f'udpsink host=127.0.0.1 port={self.rtsp_port}'
            ]
            
            self.logger.info(f"Starting GStreamer pipeline: {' '.join(gst_cmd)}")
            
            self.gst_process = subprocess.Popen(
                gst_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            time.sleep(2)
            
            if self.gst_process.poll() is None:
                self.logger.info(f"GStreamer RTSP server started on port {self.rtsp_port}")
                return True
            else:
                self.logger.error("GStreamer process failed to start")
                return False
                
        except Exception as e:
            self.logger.error(f"Error starting GStreamer RTSP server: {e}")
            return False
    
    def stop_gstreamer_rtsp(self):
        """Stop GStreamer RTSP server."""
        if self.gst_process:
            try:
                self.gst_process.terminate()
                self.gst_process.wait(timeout=5)
            except:
                self.gst_process.kill()
            finally:
                self.gst_process = None
        self.logger.info("GStreamer RTSP server stopped")


def signal_handler(signum, frame):
    """Handle termination signals."""
    print("\nReceived signal to stop streaming...")
    sys.exit(0)


# Example usage
if __name__ == "__main__":
    # Setup command line argument parsing
    parser = argparse.ArgumentParser(description='RTSP Camera Streaming Server')
    parser.add_argument('--fps', type=int, default=30, 
                       help='Frame rate for video capture (default: 30)')
    parser.add_argument('--device', type=str, default='/dev/video0',
                       help='Camera device path (default: /dev/video0)')
    parser.add_argument('--port', type=int, default=8554,
                       help='RTSP server port (default: 8554)')
    parser.add_argument('--width', type=int, default=640,
                       help='Video width in pixels (default: 640)')
    parser.add_argument('--height', type=int, default=480,
                       help='Video height in pixels (default: 480)')
    parser.add_argument('--stream-name', type=str, default='camera',
                       help='Stream name for RTSP URL (default: camera)')
    parser.add_argument('--bind-address', type=str, default='0.0.0.0',
                       help='Address to bind server to (default: 0.0.0.0 for external access)')
    
    args = parser.parse_args()
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create RTSP server instance with command line arguments
    rtsp_server = RTSPCameraServer(
        camera_device=args.device,
        rtsp_port=args.port,
        stream_name=args.stream_name,
        width=args.width,
        height=args.height,
        fps=args.fps,
        bind_address=args.bind_address
    )
    
    try:
        # Start streaming
        print("Starting RTSP camera server...")
        if rtsp_server.start_streaming():
            print(f"RTSP stream available at: {rtsp_server.get_stream_url()}")
            print("Stream info:", rtsp_server.get_stream_info())
            print("Press Ctrl+C to stop...")
            
            # Keep server running
            while rtsp_server.is_streaming():
                time.sleep(1)
        else:
            print("Failed to start RTSP streaming")
            
    except KeyboardInterrupt:
        print("\nStopping RTSP server...")
    finally:
        rtsp_server.stop_streaming()
        print("RTSP server stopped")