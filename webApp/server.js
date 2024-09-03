const express = require('express');
const fs = require('fs');
const path = require('path');

const app = express();
const PORT = 3000;

app.use(express.static(path.join(__dirname, 'public')));
app.use(express.json());

// Serve the index.html file when accessing the root URL
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// Route to get the output text
app.get('/outputtext', (req, res) => {
    fs.readFile(path.join(__dirname, 'outputtext.txt'), 'utf8', (err, data) => {
        if (err) {
            console.error(err);
            return res.status(500).send('Error reading file');
        }
        res.send(data);
    });
});

// Route to handle input text submission
app.post('/inputtext', (req, res) => {
    const inputText = req.body.inputText;
    
    // Write to inputtext.txt
    fs.writeFile(path.join(__dirname, 'inputtext.txt'), inputText, (err) => {
        if (err) {
            console.error(err);
            return res.status(500).send('Error writing file');
        }

        // Clear the outputtext.txt
        fs.writeFile(path.join(__dirname, 'outputtext.txt'), '', (err) => {
            if (err) {
                console.error(err);
                return res.status(500).send('Error clearing output file');
            }
            res.send('Text received and output cleared');
        });
    });
});

app.listen(PORT, () => {
    console.log(`Server running on http://localhost:${PORT}`);
});
