import PIL.Image
import google.generativeai as genai # type: ignore
import os
import glob

# Define the paths for FIFOs
img_fifo_path = '/tmp/gpt_imgcont_fifo'
comin_fifo_path = '/tmp/gpt_comin_fifo'
imflag_fifo_path = '/tmp/gpt_imflag_fifo'
seecont_fifo_path = '/tmp/gpt_seecont_fifo'
folder_path = "./captures"
text_file_path = "./img_text.txt"  # Path to store the text content

GOOGLE_API_KEY = os.getenv('GOOGLE_API_KEY')

def fifowrite(fifo_path, whatToWrite):
    """Write to a FIFO."""
    if not os.path.exists(fifo_path):
        os.mkfifo(fifo_path)
    
    with open(fifo_path, 'w') as fifo:
        fifo.write(whatToWrite)

def fiforead(fifo_path):
    """Read from a FIFO."""
    if not os.path.exists(fifo_path):
        os.mkfifo(fifo_path)
    
    with open(fifo_path, 'r') as fifo:
        return fifo.readline().strip()

def get_latest_image(folder_path):
    """Get the latest image file in the specified folder."""
    image_extensions = ('*.png', '*.jpg', '*.jpeg', '*.bmp', '*.gif')
    image_files = []

    for ext in image_extensions:
        image_files.extend(glob.glob(os.path.join(folder_path, ext)))

    if not image_files:
        return None

    return max(image_files, key=os.path.getmtime)

# Initialize the Generative AI model
genai.configure(api_key=GOOGLE_API_KEY)
model = genai.GenerativeModel('gemini-1.5-pro')

while True:
    flag = fiforead(img_fifo_path)
    if flag == "i":
        see_context = fiforead(seecont_fifo_path)
        latest_image = get_latest_image(folder_path)

        if latest_image:
            img = PIL.Image.open(latest_image)

            # Form the prompt with the context
            prompt_text = f"{see_context}"
            response = model.generate_content([prompt_text, img])

            # Extract the text content from the response
            text_content = "Image description:" + response.candidates[0].content.parts[0].text

            # Save text_content to a file
            with open(text_file_path, 'w') as text_file:
                text_file.write(text_content)

            # Print the extracted text for debugging/logging purposes
            print(text_content)
        else:
            print("No image found.")
