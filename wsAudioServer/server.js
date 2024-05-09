const path = require("path");
const express = require("express");
const WebSocket = require("ws");
const { SerialPort } = require('serialport')
const { ReadlineParser } = require('@serialport/parser-readline')
const app = express();
const { spawn } = require('child_process');



const WS_PORT = process.env.WS_PORT || 40040;
const HTTP_PORT = process.env.HTTP_PORT || 8080;

const serialPortPath = '/dev/ttyUSB0';
const baudRate = 115200;
const serialPort = new SerialPort({ path: serialPortPath, baudRate: baudRate }, (err) => {
  if (err) {
    console.error(`Error opening serial port ${serialPortPath} at baud rate ${baudRate}:`, err);
  } else {
    console.log(`Serial port ${serialPortPath} opened at baud rate ${baudRate}`);
  }
});

const parser = serialPort.pipe(new ReadlineParser({ delimiter: '\r\n' }))
const wsServer = new WebSocket.Server({ port: WS_PORT }, () =>
  console.log(`WS server is listening at ws:localhost:${WS_PORT}`)
);

let connectedClients = [];
let arecordProcess = null;
let isPaused = false;

wsServer.on("connection", (ws, req) => {
   const clientId = Math.random().toString(36).substring(7);
  console.log(`Client connected: ${clientId}`);

  connectedClients.push({ id: clientId, socket: ws });

  ws.on("message", (message) => {
     connectedClients.forEach((client, i) => {
      if (client.socket.readyState === WebSocket.OPEN) {
       
       
         if (message === "BTN15") {
            resumeMicrophoneStream();
            
         } else if (message === "BTN16") {
            pauseMicrophoneStream();
    
         } else if (message === "BTN01") {
            const btdataToSend = Buffer.from([0xFE, 0x01, 0x02, 0xFD]); //menu
            const okdataToSend = Buffer.from([0xFE, 0x01, 0x00, 0xFD]); //ok1
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            serialPort.write(okdataToSend);
            resumeMicrophoneStream();
            
    
         } else if (message === "BTN02") {
            const btdataToSend = Buffer.from([0xFE, 0x01, 0x01, 0xFD]); //f1
            const okdataToSend = Buffer.from([0xFE, 0x01, 0x00, 0xFD]); //ok1
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            serialPort.write(okdataToSend);
            resumeMicrophoneStream();
    
         } else if (message === "BTN03") {
            const btdataToSend = Buffer.from([0xFE, 0x02, 0x80, 0xFD]); //f2
            const okdataToSend = Buffer.from([0xFE, 0x02, 0x00, 0xFD]); //ok2
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            serialPort.write(okdataToSend);
            resumeMicrophoneStream();
    
         } else if (message === "BTN04") {
            const btdataToSend = Buffer.from([0xFE, 0x02, 0x40, 0xFD]); //f3
            const okdataToSend = Buffer.from([0xFE, 0x02, 0x00, 0xFD]); //ok2
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            serialPort.write(okdataToSend);
            resumeMicrophoneStream();
    
         } else if (message === "BTN05") {
            const btdataToSend = Buffer.from([0xFE, 0x02, 0x08, 0xFD]); //mode
            const okdataToSend = Buffer.from([0xFE, 0x02, 0x00, 0xFD]); //ok2
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            serialPort.write(okdataToSend);
            resumeMicrophoneStream();
    
         } else if (message === "BTN06") {
            const btdataToSend = Buffer.from([0xFE, 0x02, 0x04, 0xFD]); //ts
            const okdataToSend = Buffer.from([0xFE, 0x02, 0x00, 0xFD]); //ok2
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            serialPort.write(okdataToSend);
            resumeMicrophoneStream();
    
         } else if (message === "BTN07") {
            const btdataToSend = Buffer.from([0xFE, 0x02, 0x20, 0xFD]); //disp
            const okdataToSend = Buffer.from([0xFE, 0x02, 0x00, 0xFD]); //ok2
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            serialPort.write(okdataToSend);
            resumeMicrophoneStream();
    
         } else if (message === "BTN08") {
            const btdataToSend = Buffer.from([0xFE, 0x02, 0x10, 0xFD]); //lock
            const okdataToSend = Buffer.from([0xFE, 0x02, 0x00, 0xFD]); //ok2
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            serialPort.write(okdataToSend);
            resumeMicrophoneStream();
    
         } else if (message === "BTN09") {
            const btdataToSend = Buffer.from([0xFE, 0x02, 0x02, 0xFD]); //up
            const okdataToSend = Buffer.from([0xFE, 0x02, 0x00, 0xFD]); //ok2
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            serialPort.write(okdataToSend);
            resumeMicrophoneStream();
    
         } else if (message === "BTN10") {
            const btdataToSend = Buffer.from([0xFE, 0x02, 0x01, 0xFD]); //down
            const okdataToSend = Buffer.from([0xFE, 0x02, 0x00, 0xFD]); //ok2
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            serialPort.write(okdataToSend);
            resumeMicrophoneStream();
    
         } else if (message === "BTN11") {
            const btdataToSend = Buffer.from([0xFE, 0x06, 0x7A, 0xFD]); //off  
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            resumeMicrophoneStream();
           
    
         } else if (message === "BTN12") {
            const btdataToSend = Buffer.from([0xFE, 0x01, 0x08, 0xFD]); //att
            const okdataToSend = Buffer.from([0xFE, 0x01, 0x00, 0xFD]); //ok1
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            serialPort.write(okdataToSend);
            resumeMicrophoneStream();
    
         } else if (message === "BTN13") {
            const btdataToSend = Buffer.from([0xFE, 0x01, 0x04, 0xFD]); //tune
            const okdataToSend = Buffer.from([0xFE, 0x01, 0x00, 0xFD]); //ok1
            pauseMicrophoneStream();
            serialPort.write(btdataToSend);
            serialPort.write(okdataToSend);
            resumeMicrophoneStream();
    
         } 
        
      } else {
        connectedClients.splice(i, 1);
      }
    });
  });
});


let receivedData = '';


    serialPort.on('data', (data) => {
    receivedData += data.toString();
    const lines = receivedData.split('\n');
    receivedData = lines.pop();

    lines.forEach((line) => {
        connectedClients.forEach((client) => {
            if (client.socket.readyState === WebSocket.OPEN) {
                client.socket.send(line);
              
            }
        });
        console.log(line);
    });
});





serialPort.on('error', (err) => {
  console.error('Serial port parser error:', err);
  });


startMicrophoneStream();


/*
function startMicrophoneStream() {
   const arecordProcess = spawn('arecord', ['-r','16000','-D', 'plughw:2', '-f','cd','-c', '1']);
  
       
    
    arecordProcess.stdout.on('data', (data) => {
        
        if (!isPaused) {
        connectedClients.forEach((client) => {
            if (client.socket.readyState === WebSocket.OPEN) {
         
                client.socket.send(data);
            }
        });
        }
    });
   
    arecordProcess.stderr.on('data', (data) => {
        console.error(`arecord error: ${data}`);
    });
    
    arecordProcess.on('close', (code) => {
        console.log(`arecord process ended with code ${code}`);
       
    });
}
*/

function startMicrophoneStream() {
    if (arecordProcess === null) {
        startArecordProcess();
    }

    function startArecordProcess() {
        arecordProcess = spawn('arecord', ['-r','16000','-D', 'plughw:2', '-f','cd','-c', '1']);

        arecordProcess.stdout.on('data', (data) => {
            if (!isPaused) {
                connectedClients.forEach((client) => {
                    if (client.socket.readyState === WebSocket.OPEN) {
                        client.socket.send(data);
                    }
                });
            }
        });

        arecordProcess.stderr.on('data', (data) => {
            console.error(`arecord error: ${data}`);
        });

        arecordProcess.on('close', (code) => {
            console.log(`arecord process ended with code ${code}`);
        });
    }

  
    function restartArecord() {
        if (arecordProcess !== null) {
            console.log('Stopping arecord process...');
            arecordProcess.kill('SIGINT'); 
            arecordProcess.on('exit', () => {
                startArecordProcess(); 
            });
        }
    }

   
    wsServer.on("connection", (ws, req) => {
        restartArecord();
    });
}

function pauseMicrophoneStream() {
    isPaused = true;
}

function resumeMicrophoneStream() {
    isPaused = false;
}

// HTTP stuff
app.use("/image", express.static("image"));
app.use("/js", express.static("js"));
app.get("/audio", (req, res) =>
  res.sendFile(path.resolve(__dirname, "./audio_client.html"))
);

// Start the HTTP server
app.listen(HTTP_PORT, () =>
  console.log(`HTTP server listening at http:localhost:${HTTP_PORT}`)
);

 
      
        

      
        
