import os
import time
from openai import OpenAI # type: ignore
import sys

# Replace with your actual API key and assistant ID
OPENAI_API_KEY = os.getenv('OPENAI_API_KEY')
ASSISTANT_ID = os.getenv('ASSISTANT_ID')

# Check if environment variables are set
if not OPENAI_API_KEY:
    print("Error: OPENAI_API_KEY environment variable is not set.")
    sys.exit(1)

if not ASSISTANT_ID:
    print("Error: OPENAI_ASSISTANT_ID environment variable is not set.")
    sys.exit(1)

# Initialize the OpenAI client
client = OpenAI(api_key=OPENAI_API_KEY)

# Create named pipes (FIFO) if they don't exist
command_fifo_path = '/tmp/gpt_command_fifo'
abort_fifo_path = '/tmp/gpt_abort_fifo'
status_fifo_path = '/tmp/gpt_status_fifo'
obcoords_fifo_path = '/tmp/gpt_obcoords_fifo'

for fifo_path in [command_fifo_path, abort_fifo_path,status_fifo_path,obcoords_fifo_path]:
    if not os.path.exists(fifo_path):
        os.mkfifo(fifo_path)

def create_thread_with_message(message):
    try:
        thread = client.beta.threads.create(
            messages=[
                {
                    "role": "user",
                    "content": message,
                }
            ]
        )
        return thread
    except Exception as e:
        print(f"Error creating thread: {e}")
        sys.exit(1)

def run_assistant(thread_id):
    try:
        run = client.beta.threads.runs.create(thread_id=thread_id, assistant_id=ASSISTANT_ID)
        while run.status not in ["completed", "failed"]:
            run = client.beta.threads.runs.retrieve(thread_id=thread_id, run_id=run.id)
            time.sleep(1)
        if run.status == "failed":
            print(f"Run failed: {run.last_error}")
            return None
        return run
    except Exception as e:
        print(f"Error running assistant: {e}")
        return None

def get_latest_message(thread_id):
    try:
        messages = client.beta.threads.messages.list(thread_id=thread_id)
        return messages.data[0].content[0].text.value
    except Exception as e:
        print(f"Error getting latest message: {e}")
        return None
    
def get_current_status():
    try:
        with open(status_fifo_path, 'r') as fifo:
            status_line = fifo.readline().strip()
            return status_line  # Return the status line as a string
    except Exception as e:
        print(f"Error reading status: {e}")
        return None
    
def get_coords():
    try:
        with open(obcoords_fifo_path, 'r') as fifo:
            coords_line = fifo.readline().strip()
            return coords_line  # Return the coordinates line as a string
    except Exception as e:
        print(f"Error reading coordinates: {e}")
        return "no object detected"

def send_abort_command(command):
    if not os.path.exists(abort_fifo_path):
        print(f"Error: FIFO {abort_fifo_path} does not exist.")
        return

    with open(abort_fifo_path, 'w') as fifo:
        fifo.write(command + '\n')

def main():


    while True:



        user_prompt = input("Commander: ")

        if user_prompt.lower() == "exit":
            print("Exiting the drone command interface. Goodbye!")
            break
        
        global abor
        abor = user_prompt

        if abor.lower() == "stop":
            send_abort_command('stop')
            abor = 'nothing'
            
        stat = get_current_status()
        objcoords = get_coords()
    
        status_pre_instruction = "you are a drone. your current latitude, longitude and altitude are  "+ stat + "\n"
        pre_instruction_vision_description = "Here are some objects and their coordinates that your camera can see. when asked what you can see, tell these. And if told to go to these objects, simply use GOTO with the coordinates of the objects" + objcoords
        thread = create_thread_with_message(status_pre_instruction + pre_instruction_vision_description)

        try:

            full_prompt = user_prompt

            # Add user message to the thread
            client.beta.threads.messages.create(
                thread_id=thread.id,
                role="user",
                content=full_prompt
            )

            # Run the assistant
            run = run_assistant(thread.id)
            if run is None:
                continue

            # Get the latest message (assistant's response)
            instruction = get_latest_message(thread.id)

            if instruction:
                print("Drone Pilot: " + instruction)
                
                
            with open(command_fifo_path, 'w') as fifo:
                fifo.write(instruction + "\n")
                continue
            
                    
                
        except Exception as e:
            print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
