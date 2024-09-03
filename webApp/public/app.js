document.addEventListener('DOMContentLoaded', () => {
    const outputDisplay = document.querySelector('.output_display');
    const inputTextbox = document.querySelector('.input_textbox');

    // Function to fetch and display the content of outputtext.txt
    function loadOutputText() {
        fetch('/outputtext')
            .then(response => response.text())
            .then(data => {
                outputDisplay.value = data;
            })
            .catch(error => console.error('Error fetching output text:', error));
    }

    // Function to handle input text submission
    function submitInputText() {
        const inputText = inputTextbox.value;

        fetch('/inputtext', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ inputText }),
        })
        .then(response => response.text())
        .then(message => {
            console.log(message);
            inputTextbox.value = ''; // Clear the input textbox
            loadOutputText(); // Refresh the output display
        })
        .catch(error => console.error('Error submitting input text:', error));
    }

    // Load the initial output text
    loadOutputText();

    // Handle the Enter key press in the input textbox
    inputTextbox.addEventListener('keydown', (event) => {
        if (event.key === 'Enter') {
            event.preventDefault();
            submitInputText();
        }
    });
});
