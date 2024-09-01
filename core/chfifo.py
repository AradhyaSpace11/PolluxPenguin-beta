import os

fifo_path = '/tmp/gpt_obcoords_fifo'

if not os.path.exists(fifo_path):
    os.mkfifo(fifo_path)

print("Listening for messages on /tmp/gpt_obcoords_fifo...")

try:
    while True:
        with open(fifo_path, 'r') as fifo:
            # Read a line (message) from the FIFO
            message = fifo.readline().strip()
            if message:
                print(f"Received message: {message}")
except KeyboardInterrupt:
    print("Stopped listening for messages.")
except Exception as e:
    print(f"An error occurred: {e}")
