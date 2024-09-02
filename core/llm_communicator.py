from pathlib import Path
import os
import time
import sys  
import google.generativeai as genai # type: ignore


GOOGLE_API_KEY = os.getenv('GOOGLE_API_KEY')

genai.configure(api_key=GOOGLE_API_KEY)

model = genai.GenerativeModel('gemini-1.5-flash')

fifo_paths = {
    "command": '/tmp/gpt_command_fifo',
    "abort": '/tmp/gpt_abort_fifo',
    "status": '/tmp/gpt_statusa_fifo',
    "obcoords": '/tmp/gpt_obcoords_fifo',
}
imgcont_fifo_path = '/tmp/gpt_imgcont_fifo'
imgcontnew_fifo_path = '/tmp/gpt_imgcontnew_fifo'
text_file_path = "./img_text.txt"
sys_instructions_path = "./sysin.txt"

with open(sys_instructions_path, "r") as file:
    sys_cont = file.read()

safetysettings = [
    {"category": "HARM_CATEGORY_HARASSMENT", "threshold": "BLOCK_NONE"},
    {"category": "HARM_CATEGORY_HATE_SPEECH", "threshold": "BLOCK_NONE"},
    {"category": "HARM_CATEGORY_SEXUALLY_EXPLICIT", "threshold": "BLOCK_NONE"},
    {"category": "HARM_CATEGORY_DANGEROUS_CONTENT", "threshold": "BLOCK_NONE"}
]

def fiforead(fifo_path):
    """Check if the FIFO exists, create it if not, and read from it."""
    if not os.path.exists(fifo_path):
        os.mkfifo(fifo_path)
    
    # Open the FIFO in non-blocking mode
    try:
        with open(fifo_path, 'r', os.O_NONBLOCK) as fifo:
            return fifo.read()
    except Exception as e:
        print(f"Error reading from FIFO {fifo_path}: {e}")
        return ""

def fifowrite(fifo_path, whatToWrite):
    """Check if the FIFO exists, create it if not, and write to it."""
    if not os.path.exists(fifo_path):
        os.mkfifo(fifo_path)
    
    # Open the FIFO in non-blocking mode
    try:
        with open(fifo_path, 'w', os.O_NONBLOCK) as fifo:
            fifo.write(whatToWrite)
    except Exception as e:
        print(f"Error writing to FIFO {fifo_path}: {e}")

def read_img_text(file_path):
    try:
        with open(file_path, 'r') as file:
            content = file.read().strip()
        return content
    except FileNotFoundError:
        print("File not found.")
        return ""

def clear_img_text(file_path):
    with open(file_path, 'w') as file:
        pass

def send_abort_command(command):
    try:
        with open(fifo_paths["abort"], 'w') as fifo:
            fifo.write(command + '\n')
    except Exception as e:
        print(f"Error sending abort command: {e}")

def get_current_status():
    try:
        with open(fifo_paths["status"], 'r') as fifo:
            return fifo.readline().strip()
    except Exception as e:
        print(f"Error reading status: {e}")
        return None

def get_coords():
    try:
        with open(fifo_paths["obcoords"], 'r') as fifo:
            return fifo.readline().strip()
    except Exception as e:
        print(f"Error reading coordinates: {e}")
        return "no object detected"

def send_command(instruction):
    try:
        with open(fifo_paths["command"], 'w') as fifo:
            fifo.write(instruction + "\n")
        print(f"Drone: {instruction}")
    except Exception as e:
        print(f"Error writing to command FIFO: {e}")

def process_image_description(chat_session, description, user_prompt):
    full_prompt = f"Based on this image description: '{description}', and the user's instruction: '{user_prompt}', what should the drone do next? Respond with only the exact command the drone should execute, such as 'F100', 'LAND', or 'SEE'."
    
    response = chat_session.send_message(full_prompt, safety_settings=safetysettings)
    return response.text.strip()

def output_instruction(instruction):
    try:
        with open(fifo_paths["out"], 'w') as fifo:
            fifo.write(instruction + "\n")
    except Exception as e:
        print(f"Error writing instruction: {e}")

def handle_gpt_commands(chat_session):
    global abor
    abor = 'nothing'
    
    while True:
        user_prompt = input("Commander: ")
        #user_prompt = fiforead(fifo_paths["comin"])

        if user_prompt.lower() == "exit":
            #output_instruction("Exiting the drone command interface. Goodbye!")
            print("Exiting the drone command interface. Goodbye!")
            break

        if user_prompt.lower() == "stop":
            send_abort_command('stop')
            time.sleep(2)
            abor = 'nothing'
            send_abort_command(abor)
            continue

        stat = get_current_status()
        objcoords = get_coords()

        status_pre_instruction = f"You are a drone. Your current latitude, longitude, and altitude are {stat}.\n"
        #pre_instruction_vision_description = f"Here are some objects and their coordinates that your camera can see: {objcoords}. When asked what you can see, tell these. And if told to go to these objects, simply use GOTO with the coordinates of the objects."

        #full_prompt = status_pre_instruction + pre_instruction_vision_description + "\n" + user_prompt
        full_prompt = status_pre_instruction +"\n" + user_prompt

        response = chat_session.send_message(full_prompt, safety_settings=safetysettings)
        instruction = response.text.strip()
        
        if instruction:
            send_command(instruction)
            #output_instruction(instruction)
            
            
            
            while "SEE" in instruction:
                flg = fiforead(imgcontnew_fifo_path)
                if flg == "i" :
                    #output_instruction("Processing image flag...")
                    print("Processing image flag...")
                    time.sleep(7)  # Wait for the image description to be written
                    image_description = read_img_text(text_file_path)
                    if image_description:
                        next_instruction = process_image_description(chat_session, image_description, user_prompt)
                        send_command(next_instruction)
                        clear_img_text(text_file_path)
                        flg = "h"
                        break
                    
                        
                    else:
                        print("No image description found.")
                        #output_instruction("No image description found.")
                        break
                        

def main():
    chat_session = model.start_chat(history=[])

    try:
        respo1 = chat_session.send_message(sys_cont, safety_settings=safetysettings)
    except Exception as e:
        print(f"Error sending initial system instructions: {e}")
        sys.exit(1)

    handle_gpt_commands(chat_session)

if __name__ == "__main__":
    main()
    
