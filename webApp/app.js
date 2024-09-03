document.addEventListener('DOMContentLoaded', function() {
    const inputTextBox = document.querySelector('.input_textbox');

    // Listen for the 'Enter' keypress in the input textbox
    inputTextBox.addEventListener('keypress', function(event) {
        if (event.key === 'Enter') {
            event.preventDefault();  // Prevent the default action of the Enter key (e.g., line break)
            sendMessage();
        }
    });

    // Function to send the message to the Flask server
    function sendMessage() {
        const message = inputTextBox.value.trim();  // Get the trimmed message text

        if (message === "") return;  // Don't send empty messages

        // Clear the input textbox immediately
        inputTextBox.value = '';

        fetch('http://localhost:3000/append', {
            method: 'POST',
            headers: {
                'Content-Type': 'text/plain',
            },
            body: message,
        })
        .then(response => response.text())
        .then(data => {
            console.log('Success:', data);  // Log success message
        })
        .catch((error) => {
            console.error('Error:', error);  // Log error message
        });
    }
});
