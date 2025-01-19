// Set up the canvases and contexts
const wheelCanvas1 = document.getElementById('wheel1');
const wheelCanvas2 = document.getElementById('wheel2');
const wheelCtx1 = wheelCanvas1.getContext('2d');
const wheelCtx2 = wheelCanvas2.getContext('2d');

const signalCanvas = document.getElementById('signal');
const signalCtx = signalCanvas.getContext('2d');

const onlineDisplay = document.getElementById("online");
const systemDisplay = document.getElementById("system");

const output = document.getElementById('output');

// Set canvas dimensions
const wheelRadius = 100;
wheelCanvas1.width = wheelCanvas2.width = 300;
wheelCanvas1.height = wheelCanvas2.height = 300;

// Initial rotation angles
let rotation1 = 0;
let rotation2 = 0;

online = false;
signal = 0;


function log(commandText) {
    const newCommand = document.createElement('div');
    newCommand.textContent = `> ${commandText}`;
    output.appendChild(newCommand);
    output.scrollTop = output.scrollHeight; // Aut
}

function logError(errorText) {
    const newCommand = document.createElement('div');
    newCommand.textContent = `> ${errorText}`;
    newCommand.style.color = "#ff0000"
    output.appendChild(newCommand);
    output.scrollTop = output.scrollHeight; // Aut

}

function drawSignal(ctx, signal) {
    ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height); // Clear the canvas
    ctx.lineWidth = 20;

    if (signal >= 1) {
        ctx.strokeStyle = '#ff0000';
        ctx.beginPath()
        ctx.moveTo(ctx.canvas.width/10, (ctx.canvas.height*6)/7);
        ctx.lineTo(ctx.canvas.width/10, (ctx.canvas.height*5)/7);
        ctx.stroke(); 
    }
    if (signal >= 2) {
        ctx.strokeStyle = '#ffa500';

        ctx.beginPath()
        ctx.moveTo((ctx.canvas.width*3)/10, (ctx.canvas.height*6)/7);
        ctx.lineTo((ctx.canvas.width*3)/10, (ctx.canvas.height*4)/7);
        ctx.stroke(); 
    }
    if (signal >= 3) {
        ctx.strokeStyle = '#ffa500';

        ctx.beginPath()
        ctx.moveTo((ctx.canvas.width*5)/10, (ctx.canvas.height*6)/7);
        ctx.lineTo((ctx.canvas.width*5)/10, (ctx.canvas.height*3)/7);
        ctx.stroke(); 
    }
    if (signal >= 4) {
        ctx.strokeStyle = '#00ff00';

        ctx.beginPath()
        ctx.moveTo((ctx.canvas.width*7)/10, (ctx.canvas.height*6)/7);
        ctx.lineTo((ctx.canvas.width*7)/10, (ctx.canvas.height*2)/7);
        ctx.stroke(); 
    }
    if (signal >= 5) {
        ctx.strokeStyle = '#f00ff00';

        ctx.beginPath()
        ctx.moveTo((ctx.canvas.width*9)/10, (ctx.canvas.height*6)/7);
        ctx.lineTo((ctx.canvas.width*9)/10, (ctx.canvas.height*1)/7);
        ctx.stroke(); 
    }



    ctx.restore();
}

function drawWheel(ctx, rotation) {
    ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height); // Clear the canvas

    ctx.save();
    ctx.translate(ctx.canvas.width / 2, ctx.canvas.height / 2); // Move to the center of the canvas
    ctx.rotate(rotation); // Apply rotation

    // Draw the outer circle (wheel)
    ctx.beginPath();
    ctx.arc(0, 0, wheelRadius, 0, 2 * Math.PI);
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 10;
    ctx.stroke();

    // Draw the center of the wheel
    ctx.beginPath();
    ctx.arc(0, 0, 20, 0, 2 * Math.PI);
    ctx.fillStyle = '#00ff00';
    ctx.fill();

    ctx.moveTo(-wheelRadius, 0);
    ctx.lineTo(wheelRadius, 0);
    ctx.stroke(); 
    ctx.moveTo(0, -wheelRadius);
    ctx.lineTo(0, wheelRadius);
    ctx.stroke(); 

    ctx.restore();
}

drawWheel(wheelCtx1, 0);
drawWheel(wheelCtx2, 0);


slider1 = document.getElementById("slider1")
slider1Value = document.getElementById("slider1Value")

slider2 = document.getElementById("slider2")
slider2Value = document.getElementById("slider2Value")

resetSliders()

slider1Prev = 0
slider2Prev = 0
encoder1Prev = 0
encoder2Prev = 0

async function sendSpeed() {

    if (slider1Prev != slider1.value) {
        log(`Sending Motor 1 Speed: ${slider1.value} RPS`)
    }

    if (slider2Prev != slider2.value) {
        log(`Sending Motor 2 Speed: ${slider2.value} RPS`)
    }

    slider1Value.innerHTML = slider1.value;
    slider2Value.innerHTML = slider2.value;

    fetch(`http://192.168.234.45:5000/speed?rps1=${slider1.value}&rps2=${slider2.value}`)
        .then(response => response.json())
        .then(data => {
            if (!online) {
                log("Connected to server!")
            }
            online = true
            if (data[0] != encoder1Prev || data[1] != encoder2Prev) {
                log(`Encoder data received: ${data}`);
            }
            encoder1Prev = data[0];
            encoder2Prev = data[1];

            rotation1 = (parseFloat(data[0])/2740) * 2 * Math.PI
            rotation2 = (parseFloat(data[1])/2740) * 2 * Math.PI
            drawWheel(wheelCtx1, rotation1);
            drawWheel(wheelCtx2, rotation2);
        })
        .catch(error => {
            console.error('Error with the GET request:', error);
            // if (online) {
                logError("Couldn't reach server...")
            // }
            online = false;
        });

    slider1Prev = slider1.value;
    slider2Prev = slider2.value;
}

function resetSliders() {
    slider1.value = 0;
    slider1Value.innerHTML = 0;
    slider2.value = 0;
    slider2Value.innerHTML = 0;
}

setInterval(sendSpeed, 50);


function updateOnlineDisplay() {
    if (online) {
        signal = Math.min(signal+1, 5)
        drawSignal(signalCtx, signal);
        onlineDisplay.classList.remove("neonRed");
        onlineDisplay.innerHTML = "COULDN'T ACQUIRE SIGNAL"
        systemDisplay.classList.remove("neonRed");
        systemDisplay.innerHTML = "SYSTEM NOT RESPONDING"

        onlineDisplay.classList.add("neonGreen");
        onlineDisplay.innerHTML = "SIGNAL ACQUIRED"
        systemDisplay.classList.add("neonGreen");
        systemDisplay.innerHTML = "ALL SYSTEMS ONLINE"
    } else {
        signal = Math.max(signal-1, 1)
        drawSignal(signalCtx, signal);
        onlineDisplay.classList.remove("neonGreen");
        onlineDisplay.innerHTML = "SIGNAL ACQUIRED"
        systemDisplay.classList.remove("neonGreen");
        systemDisplay.innerHTML = "ALL SYSTEMS ONLINE"

        onlineDisplay.classList.add("neonRed");
        onlineDisplay.innerHTML = "COULDN'T ACQUIRE SIGNAL"
        systemDisplay.classList.add("neonRed");
        systemDisplay.innerHTML = "SYSTEM NOT RESPONDING"
    }
    if (onlineDisplay.style.display == "none") {
        onlineDisplay.style.display = "block";
        systemDisplay.style.display = "block";
    } else {
        onlineDisplay.style.display = "none";
        systemDisplay.style.display = "none";
    }
    
}


setInterval(updateOnlineDisplay, 500);



