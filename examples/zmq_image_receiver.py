#!/usr/bin/env python3

"""
Demo script that receives images using ZMQ without depending on ROS.
It expects messages sent by the 'network_bridge' package:
- Reads the network_bridge custom header.
- Decompresses data if necessary.
- Unpacks the CDR-serialized sensor_msgs/Image.
- Uses OpenCV to visualize it.

Required dependencies:
pip install pyzmq zstandard opencv-python numpy
"""

import argparse
import struct
import zmq
import zstandard as zstd
import cv2
import numpy as np

class CDRDeserializer:
    """
    A simple deserializer for ROS 2 FastDDS/CDR encoded streams,
    specifically tailored for the sensor_msgs/msg/Image layout.
    """
    def __init__(self, data):
        self.data = data
        self.offset = 0
        self.endian = '<' # Little endian by default

        # Parse encapsulation header
        if len(data) >= 4:
            encap = data[self.offset : self.offset+4]
            self.offset += 4
            # encap[1] indicates endianness: 0=Big Endian, 1=Little Endian
            if encap[1] == 0:
                self.endian = '>'
            elif encap[1] == 1:
                self.endian = '<'

    def align(self, size):
        r = self.offset % size
        if r > 0:
            self.offset += (size - r)

    def read_uint32(self):
        self.align(4)
        val = struct.unpack(self.endian + 'I', self.data[self.offset:self.offset+4])[0]
        self.offset += 4
        return val

    def read_int32(self):
        self.align(4)
        val = struct.unpack(self.endian + 'i', self.data[self.offset:self.offset+4])[0]
        self.offset += 4
        return val

    def read_uint8(self):
        val = struct.unpack(self.endian + 'B', self.data[self.offset:self.offset+1])[0]
        self.offset += 1
        return val

    def read_string(self):
        length = self.read_uint32()
        if length == 0:
            return ""
        # The length includes the null terminator
        val = self.data[self.offset:self.offset+length-1]
        self.offset += length
        return val.decode('utf-8', errors='replace')
        
    def read_byte_array(self):
        length = self.read_uint32()
        self.align(1)  # Byte sequence, so 1-byte aligned (i.e. no extra padding)
        val = self.data[self.offset:self.offset+length]
        self.offset += length
        return val


def parse_network_bridge_header(data):
    """
    Parses the header introduced by network_bridge.
    Format is sequential unaligned: 
    [8 bytes double timestamp][topic string \0][msg_type string \0]
    
    Returns: (timestamp, topic, msg_type, actual_payload)
    """
    if len(data) < 8:
        raise ValueError("Data too short to contain header.")
        
    # Read double timestamp (8 bytes)
    time_val = struct.unpack('d', data[:8])[0]
    
    offset = 8
    
    # Read topic string
    topic_end = data.find(b'\x00', offset)
    if topic_end == -1:
        raise ValueError("Malformed header: missing topic null terminator.")
    topic = data[offset:topic_end].decode('utf-8')
    offset = topic_end + 1
    
    # Read msg_type string
    msg_type_end = data.find(b'\x00', offset)
    if msg_type_end == -1:
        raise ValueError("Malformed header: missing msg_type null terminator.")
    msg_type = data[offset:msg_type_end].decode('utf-8')
    offset = msg_type_end + 1
    
    payload = data[offset:]

    print(f"Received message on topic '{topic}' with type '{msg_type}'")
    
    return time_val, topic, msg_type, payload


def main():
    parser = argparse.ArgumentParser(description="Standalone ZMQ Image Receiver")
    parser.add_argument("--host", default="127.0.0.1", help="Host IP of the network bridge ZMQ server")
    parser.add_argument("--port", type=int, default=5555, help="Port of the network bridge ZMQ server")
    args = parser.parse_args()

    # Create ZMQ context and PULL socket
    # According to network_bridge test and source, the server binds to a PUSH socket
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    endpoint = f"tcp://{args.host}:{args.port}"
    print(f"Connecting to {endpoint} via ZMQ PULL...")
    socket.connect(endpoint)

    dctx = zstd.ZstdDecompressor()

    print("Listening for messages...")
    
    while True:
        try:
            # Receive raw compressed ZMQ frame
            raw_data = socket.recv()
            
            # Decompress data using Zstandard
            decompressed_data = dctx.decompress(raw_data, max_output_size=100*1024*1024)
            
            # Parse custom network bridge header
            recv_time, topic, msg_type, payload = parse_network_bridge_header(decompressed_data)
            
            # Filter solely for image messages
            if msg_type == "sensor_msgs/msg/Image":
                # print(f"Received Image on topic '{topic}', bridge_timestamp: {recv_time}")
                
                # Deserialization without ROS dependencies
                deserializer = CDRDeserializer(payload)
                
                # The layout sequence of sensor_msgs/msg/Image in ROS 2 CDR:
                _sec = deserializer.read_int32()            # std_msgs/Header/stamp/sec
                _nanosec = deserializer.read_uint32()       # std_msgs/Header/stamp/nanosec
                _frame_id = deserializer.read_string()      # std_msgs/Header/frame_id
                
                height = deserializer.read_uint32()         # height
                width = deserializer.read_uint32()          # width
                encoding = deserializer.read_string()       # encoding
                _is_bigendian = deserializer.read_uint8()   # is_bigendian
                _step = deserializer.read_uint32()          # step
                img_data = deserializer.read_byte_array()   # actual image data array
                
                # We interpret it as unsigned 8-bit array for OpenCV
                img_np = np.frombuffer(img_data, dtype=np.uint8)
                
                if encoding == "rgb8":
                    img_np = img_np.reshape((height, width, 3))
                    img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)  # Convert for cv2 (uses BGR)
                elif encoding == "bgr8":
                    img_np = img_np.reshape((height, width, 3))       # Already BGR
                elif encoding == "mono8":
                    img_np = img_np.reshape((height, width))          # Grayscale
                elif encoding == "bgra8":
                    img_np = img_np.reshape((height, width, 4))
                elif encoding == "rgba8":
                    img_np = img_np.reshape((height, width, 4))
                    img_np = cv2.cvtColor(img_np, cv2.COLOR_RGBA2BGRA)
                else:
                    print(f"Skipping visualization for unsupported encoding: {encoding}")
                    continue
                
                # Create a dynamic window naming schema per topic
                window_name = f"Image received - {topic}"
                cv2.imshow(window_name, img_np)
                
                # 1 ms wait + handles window events. Press 'q' to quit.
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Exiting...")
                    break
                    
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error processing message: {e}")
            
    cv2.destroyAllWindows()
    socket.close()
    context.term()

if __name__ == "__main__":
    main()
