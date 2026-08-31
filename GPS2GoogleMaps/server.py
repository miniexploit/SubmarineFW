import asyncio
import websockets
import serial
import threading
import http.server
import socketserver

SERIAL_PORT = '/dev/cu.usbmodem101'
BAUD_RATE = 115200
WEB_SERVER_PORT = 8000
WEBSOCKET_PORT = 8765

main_loop = None

connected_clients = set()

def serial_reader():
    print('Reading...')
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Successfully opened serial port {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"FATAL: Error opening serial port {SERIAL_PORT}: {e}")
        print("Please check the port name and ensure the ESP32 is connected.")
        return

    while True:
        try:
            line = ser.readline().decode('utf-8').strip()

            if line.startswith('MAP_DATA,'):
                coords = line.split(',', 1)[1]
                
                print(f"Received Coordinates for Map: {coords}")
                
                # Send "lat,lng"
                asyncio.run_coroutine_threadsafe(
                    broadcast_message(coords),
                    main_loop
                )

            elif line: # if not begins with MAP_DATA, a stray message
                print(f"Received from ESP32 (Debug): {line}")


        except serial.SerialException:
            print("Serial port disconnected. Exiting serial reader thread")
            break
        except Exception as e:
            print(f"An error occurred in the serial reader: {e}")

async def broadcast_message(message):
    if connected_clients:
        tasks = [client.send(message) for client in connected_clients]
        await asyncio.gather(*tasks)


async def websocket_handler(websocket):
    print("Web client connected")
    connected_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        print("Web client disconnected")
        connected_clients.remove(websocket)

async def start_websocket_server():
    server = await websockets.serve(websocket_handler, "localhost", WEBSOCKET_PORT)
    print(f"WebSocket server started on ws://localhost:{WEBSOCKET_PORT}")
    await server.wait_closed()

def start_web_server():
    handler = http.server.SimpleHTTPRequestHandler
    with socketserver.TCPServer(("", WEB_SERVER_PORT), handler) as httpd:
        print(f"Web server started. Open http://localhost:{WEB_SERVER_PORT}/map.html in your browser")
        httpd.serve_forever()

if __name__ == "__main__":
    main_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(main_loop)

    threading.Thread(target=start_web_server, daemon=True).start()
    threading.Thread(target=serial_reader, daemon=True).start()

    try:
        main_loop.run_until_complete(start_websocket_server())
    except KeyboardInterrupt:
        print("Servers shutting down")

