import os

fifo_path = '/tmp/gpt_comin_fifo'

# Ensure the FIFO file exists
if not os.path.exists(fifo_path):
    os.mkfifo(fifo_path)

print("Listening for messages...")

# Continuously read from the FIFO
with open(fifo_path, 'r') as fifo:
    while True:
        message = fifo.read().strip()  # Read and strip any extra whitespace/newlines
        if message:
            print(f"Received message: {message}")
