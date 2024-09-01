import os
from pathlib import Path
from tkinter import Tk, Canvas, Entry, Button, PhotoImage, Text, filedialog


# Paths for the FIFOs
comin_fifo_path = '/tmp/gpt_comin_fifo'
out_fifo_path = '/tmp/gpt_out_fifo'
status_fifo_path = '/tmp/gpt_statusa_fifo'

def select_file():
    global selected_photo
    selected_photo = filedialog.askopenfilename(
        title="Select a Photo",
        filetypes=[("Image Files", "*.png;*.jpg;*.jpeg;*.gif"), ("All Files", "*.*")]
    )
    if selected_photo:
        print(f"Selected file: {selected_photo}")

# Ensure the status FIFO exists
if not os.path.exists(status_fifo_path):
    os.mkfifo(status_fifo_path)

def update_status():
    try:
        with open(status_fifo_path, 'r', os.O_NONBLOCK) as fifo:
            status = fifo.readline().strip()
            entry_3.delete("1.0", 'end')  # Clear the current status
            entry_3.insert("1.0", status)  # Insert the new status
    except Exception as e:
        print(f"Error reading from status FIFO: {e}")
        entry_3.delete("1.0", 'end')
        entry_3.insert("1.0", "Error receiving status.")
    
    # Schedule the next status update
    window.after(1000, update_status)  # Update every 1 second

# Ensure the FIFOs exist
if not os.path.exists(comin_fifo_path):
    os.mkfifo(comin_fifo_path)

if not os.path.exists(out_fifo_path):
    os.mkfifo(out_fifo_path)

# Helper function to write to comin FIFO
def send_to_comin_fifo(message):
    try:
        with open(comin_fifo_path, 'w', os.O_NONBLOCK) as fifo:
            fifo.write(message + '\n')
    except Exception as e:
        print(f"Error sending to comin FIFO: {e}")

# Helper function to read from out FIFO
def read_from_out_fifo():
    try:
        with open(out_fifo_path, 'r', os.O_NONBLOCK) as fifo:
            message = fifo.readline().strip()
            return message
    except Exception as e:
        print(f"Error reading from out FIFO: {e}")
        return "Error receiving output."

# Function to handle input and update output
def display_logs():
    entry_2.delete("1.0", "end")  # Clear the text area
    # Display all logs in the large text area
    for log in logs:
        entry_2.insert("end", log)

def handle_input(event=None):
    user_input = entry_1.get()
    send_to_comin_fifo(user_input)
    entry_1.delete(0, 'end')
    
    output = read_from_out_fifo()
    entry_2.delete("1.0", "end")
    entry_2.insert("1.0", output)
    
    # Store the input and output in the logs list
    logs.append(f"Input: {user_input}\nOutput: {output}\n")

# Tkinter setup
OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH = OUTPUT_PATH / Path(r"/home/aradhya/build/assets/frame0")

def relative_to_assets(path: str) -> Path:
    return ASSETS_PATH / Path(path)

window = Tk()
window.title("PolluxPenguin")

# List to store the logs
logs = []

# Start the status update loop

window.geometry("1600x1000")
window.configure(bg = "#FFFFFF")

canvas = Canvas(
    window,
    bg = "#FFFFFF",
    height = 1000,
    width = 1600,
    bd = 0,
    highlightthickness=0,
    relief = "ridge"
)

canvas.place(x = 0, y = 0)
image_image_1 = PhotoImage(
    file=relative_to_assets("image_1.png"))
image_1 = canvas.create_image(
    800.0,
    500.0,
    image=image_image_1
)

button_image_1 = PhotoImage(
    file=relative_to_assets("button_1.png"))
button_1 = Button(
    image=button_image_1,
    borderwidth=0,
    highlightthickness=0,
    command=handle_input,  # Send input when button is pressed
    relief="flat"
)
button_1.place(
    x=1480.0,
    y=874.0,
    width=86.0,
    height=88.0
)

entry_image_1 = PhotoImage(
    file=relative_to_assets("entry_1.png"))
entry_bg_1 = canvas.create_image(
    933.0,
    918.0,
    image=entry_image_1
)
entry_1 = Entry(
    bd=0,
    bg="#FFFFFF",
    fg="#000716",
    highlightthickness=0
)
entry_1.place(
    x=408.0,
    y=874.0,
    width=1050.0,
    height=86.0
)

# Bind Enter key to handle input
entry_1.bind('<Return>', handle_input)

button_image_2 = PhotoImage(
    file=relative_to_assets("button_2.png"))
button_2 = Button(
    image=button_image_2,
    borderwidth=0,
    highlightthickness=0,
    command=display_logs,  # Display logs when button is clicked
    relief="flat"
)

button_2.place(
    x=34.0,
    y=37.0,
    width=218.0,
    height=88.0
)

button_image_3 = PhotoImage(
    file=relative_to_assets("button_3.png"))
button_3 = Button(
    image=button_image_3,
    borderwidth=0,
    highlightthickness=0,
    command=select_file,  # Call the file selector when button is clicked
    relief="flat"
)
button_3.place(
    x=280.0,
    y=874.0,
    width=100.0,
    height=88.0
)


button_image_4 = PhotoImage(
    file=relative_to_assets("button_4.png"))
button_4 = Button(
    image=button_image_4,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: print("button_4 clicked"),
    relief="flat"
)
button_4.place(
    x=34.0,
    y=276.0,
    width=218.0,
    height=88.0
)

button_image_5 = PhotoImage(
    file=relative_to_assets("button_5.png"))
button_5 = Button(
    image=button_image_5,
    borderwidth=0,
    highlightthickness=0,
    command=lambda: print("button_5 clicked"),
    relief="flat"
)
button_5.place(
    x=34.0,
    y=157.0,
    width=218.0,
    height=88.0
)

entry_image_2 = PhotoImage(
    file=relative_to_assets("entry_2.png"))
entry_bg_2 = canvas.create_image(
    923.0,
    439.0,
    image=entry_image_2
)
entry_2 = Text(
    bd=0,
    bg="#1F1F1F",
    fg="#FFFFFF",
    highlightthickness=0,
    font=("Lexend", 14),
    wrap="word", 
)
entry_2.place(
    x=280.0,
    y=36.0,
    width=1286.0,
    height=804.0
)

entry_image_3 = PhotoImage(
    file=relative_to_assets("entry_3.png"))
entry_bg_3 = canvas.create_image(
    143.0,
    685.0,
    image=entry_image_3
)
entry_3 = Text(
    bd=0,
    bg="#222324",
    fg="#ff0000",
    highlightthickness=0
)
entry_3.place(
    x=34.0,
    y=406.0,
    width=218.0,
    height=556.0
)
update_status()
window.resizable(False, False)
window.mainloop()
