document.addEventListener('DOMContentLoaded', () => {
    const outputDisplay = document.querySelector('.output_display');
    const inputTextbox = document.querySelector('.input_textbox');
    const startMicButton = document.getElementById('startMic');
    const stopButton = document.querySelector('.stop');
    const rtlButton = document.querySelector('.rtl');
    const statusBox = document.querySelector('.status_box');

    let isVoiceInput = false; // Flag to track voice input
    let recognition;

    // Function to fetch and display the content of outputtext.txt
    function loadOutputText() {
        fetch('/outputtext')
            .then(response => response.text())
            .then(data => {
                outputDisplay.value = data;
            })
            .catch(error => console.error('Error fetching output text:', error));
    }

    // Automatically refresh the output display every 2 seconds
    setInterval(loadOutputText, 500); // Refresh every 2 seconds (2000 milliseconds)

    // Function to handle input text submission
    function submitInputText(text) {
        const inputText = text || inputTextbox.value;

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
            loadOutputText(); // Refresh the output display immediately after submitting
        })
        .catch(error => console.error('Error submitting input text:', error));
    }

    // Handle the Enter key press in the input textbox
    inputTextbox.addEventListener('keydown', (event) => {
        if (event.key === 'Enter') {
            event.preventDefault();
            submitInputText();
        }
    });

    // Add event listeners for the 'stop' and 'rtl' buttons
    stopButton.addEventListener('click', () => {
        submitInputText('stop'); // Send 'stop' when stop button is pressed
    });

    rtlButton.addEventListener('click', () => {
        submitInputText('rtl'); // Send 'rtl' when rtl button is pressed
    });

    // Voice recognition feature
    if ('SpeechRecognition' in window || 'webkitSpeechRecognition' in window) {
        const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        recognition = new SpeechRecognition();
        recognition.continuous = false;
        recognition.interimResults = true;
        recognition.isStarted = false; // Initialize isStarted flag

        let silenceTimer;
        const silenceThreshold = 500;

        recognition.onstart = function() {
            statusBox.value = 'Listening...';
            startMicButton.textContent = 'Stop';
            isVoiceInput = true; // Set flag for voice input
            recognition.isStarted = true; // Set recognition as started
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
            if (isVoiceInput) {
                submitInputText(); // Automatically submit after voice input ends
                isVoiceInput = false; // Reset flag
            }
            recognition.isStarted = false; // Ensure it can be restarted
        };

        recognition.onerror = function(event) {
            statusBox.value = 'Error occurred in recognition: ' + event.error;
            recognition.isStarted = false; // Reset in case of error
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
