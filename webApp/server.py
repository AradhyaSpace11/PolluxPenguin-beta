from flask import Flask, request
import logging
import os

app = Flask(__name__)

# Disable default Flask logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

@app.route('/append', methods=['POST'])
def append_to_fifo():
    data = request.data.decode('utf-8')
    
    # Write the received data to the FIFO file
    fifo_path = '/tmp/gpt_comin_fifo'
    
    try:
        with open(fifo_path, 'w') as fifo:
            fifo.write(data)
        print(f"latest text: {data}")
    except Exception as e:
        print(f"Failed to write to FIFO: {e}")
        return 'Failed to write to FIFO', 500
    
    return 'Message written to FIFO', 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=3000)
