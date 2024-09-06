import openai
import base64
import os
import glob
import PIL.Image
import requests


# Set up your OpenAI API key
api_key = ""

# Define the paths for FIFOs
img_fifo_path = '/tmp/gpt_imgcont_fifo'
comin_fifo_path = '/tmp/gpt_comin_fifo'
imflag_fifo_path = '/tmp/gpt_imflag_fifo'
seecont_fifo_path = '/tmp/gpt_seecont_fifo'
folder_path = "./captures"
text_file_path = "./img_text.txt"  # Path to store the text content

# Function to write to a FIFO
def fifowrite(fifo_path, whatToWrite):
    """Write to a FIFO."""
    if not os.path.exists(fifo_path):
        os.mkfifo(fifo_path)
    
    with open(fifo_path, 'w') as fifo:
        fifo.write(whatToWrite)

# Function to read from a FIFO
def fiforead(fifo_path):
    """Read from a FIFO."""
    if not os.path.exists(fifo_path):
        os.mkfifo(fifo_path)
    
    with open(fifo_path, 'r') as fifo:
        return fifo.readline().strip()

# Function to get the latest image from the folder
def get_latest_image(folder_path):
    """Get the latest image file in the specified folder."""
    image_extensions = ('*.png', '*.jpg', '*.jpeg', '*.bmp', '*.gif')
    image_files = []

    for ext in image_extensions:
        image_files.extend(glob.glob(os.path.join(folder_path, ext)))

    if not image_files:
        return None

    return max(image_files, key=os.path.getmtime)

# Function to encode image in base64
def encode_image(image_path):
    """Encode image to base64."""
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

# Set up the request headers
headers = {
    "Authorization": f"Bearer {api_key}",
    "Content-Type": "application/json"
}

# Main loop to monitor for image flags and process images
while True:
    flag = fiforead(img_fifo_path)
    if flag == "i":  # When 'i' flag is received, indicating image processing
        # Read context from FIFO
        see_context = fiforead(seecont_fifo_path)
        
        # Get the latest image from the folder
        latest_image = get_latest_image(folder_path)
        
        if latest_image:
            # Encode the image to base64
            encoded_image = encode_image(latest_image)

            # Prepare the prompt text and image for the API request
            prompt_text = f"{see_context}"
            
            # Prepare the API payload with the image and prompt
            payload = {
                "model": "gpt-4o-mini",  # Replace with the correct model version if needed
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt_text},  # Use the prompt_text variable
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{encoded_image}"
                                }
                            }
                        ]
                    }
                ]
            }
            
            response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)

            response_data = response.json()

            # Print the result
            
            
            
            
            if 'choices' in response_data:
                image_description = response_data['choices'][0]['message']['content']
                with open(text_file_path, 'w') as text_file:
                    text_file.write(image_description)
            else:
                with open(text_file_path, 'w') as text_file:
                    text_file.write("Error: No description found.")



