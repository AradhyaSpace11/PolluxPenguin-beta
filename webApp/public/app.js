document.addEventListener('DOMContentLoaded', () => {
    const outputDisplay = document.querySelector('.output_display');
    const inputTextbox = document.querySelector('.input_textbox');
    const startMicButton = document.getElementById('startMic');
    const statusBox = document.querySelector('.status_box');

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

    inputTextbox.addEventListener('keydown', (event) => {
        if (event.key === 'Enter') {
            event.preventDefault();
            submitInputText();
        }
    });

    // from here this is voice to text functionalty which is not supported brave browser actually
    if ('SpeechRecognition' in window || 'webkitSpeechRecognition' in window) {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        recognition = new SpeechRecognition();
        recognition.continuous = false;
        recognition.interimResults = true;

        let silenceTimer;
        const silenceThreshold = 2000;

        recognition.onstart = function() {
            statusBox.value = 'Listening...';
            startMicButton.textContent = 'Stop';
        };

        recognition.onresult = function(event) {
            clearTimeout(silenceTimer);
            const result = event.results[event.results.length - 1];
            const transcript = result[0].transcript;
            inputTextbox.value = transcript;

            if (result.isFinal) {
                silenceTimer = setTimeout(() => {
                    recognition.stop();
                }, silenceThreshold);
            }
        };

        recognition.onend = function() {
            statusBox.value = 'Voice recognition stopped.';
            startMicButton.textContent = 'Voice';
            recognition.isStarted = false;
        };

        recognition.onerror = function(event) {
            statusBox.value = 'Error occurred in recognition: ' + event.error;
        };

        startMicButton.addEventListener('click', function() {
            if (recognition.isStarted) {
                recognition.stop();
            } else {
                recognition.start();
                recognition.isStarted = true;
            }
        });
    } else {
        statusBox.value = 'Web Speech API is not supported in this browser.';
        startMicButton.disabled = true;
    }
});