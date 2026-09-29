// Constant Variables
const connectionBtn = document.getElementById("connection-status-btn");
const circleIcon = document.getElementById("circle-icon");
const connectionText = document.getElementById("connection-status-text");
const fillElement = document.getElementById("battery-fill");
const textElement = document.getElementById("battery-text");
const startBtn = document.getElementById("start-btn");
const stopBtn = document.getElementById("stop-btn");
const irOne = document.getElementById("ir-one");
const irTwo = document.getElementById("ir-two");
const irThree = document.getElementById("ir-three");
const angleData = document.getElementById("angle-value");
const distanceData = document.getElementById("distanceData");
const headingData= document.getElementById("heading-value");
const tiltData = document.getElementById("tilt-value");
const accData = document.getElementById("accelerationData");
const flData = document.getElementById("front-left-value");
const rlData = document.getElementById("rear-left-value");
const frData = document.getElementById("front-right-value");
const rrData = document.getElementById("rear-right-value");

// Mutable Variables
let batteryInterval = null;
let sensorInterval = null;
let isConnected = false;
let connectionStartTime = null;
let connectionTimer = null;

// Functions

// Retrieve data functions
function get_local_time() {
    const localTime = new Date(); // To retrieve today's date and time 
    document.getElementById('time-container').textContent = localTime.toLocaleTimeString(); // Convert today's time only to a string 
}

function get_battery_health() { // To retrieve battery health from the PICO W
    fetch('/api/battery')
        .then(response => response.text())
        .then(voltage => {
            let minVoltage = 3.0;
            let maxVoltage = 5.0;
            let percentage = ((parseFloat(voltage) - minVoltage) / (maxVoltage - minVoltage)) * 100;
            updateBatteryUI(percentage);
        })
        .catch(err => console.log("Failed to fetch battery data"));
} 

function get_sensor_data() {
    fetch('/api/data')
        .then(response => response.json())
        .then(data => {

            // Ultrasonic + Servo
            angleData.textContent = data.angle;
            distanceData.textContent = data.distance;

            // IMU
            headingData.textContent = data.heading;
            tiltData.textContent = data.tilt;
            accData.textContent = data.acceleration;

            // Wheels
            flData.textContent = data.front_left;
            rlData.textContent = data.rear_left;
            frData.textContent = data.front_right;
            rrData.textContent = data.rear_right;

            // IR
            irOne.classList.toggle("disabled", !data.ir1);
            irTwo.classList.toggle("disabled", !data.ir2);
            irThree.classList.toggle("disabled", !data.ir3);

            // Battery
            updateBatteryUI(data.battery);
        })
        .catch(err => {
            console.log("Failed to retrieve sensor data");
        });
}

// UI toggles functions
function updateBatteryUI(percentage) {
    const clampedPct = Math.max(0, Math.min(100, percentage));
    const maxWidth = 37;
    const newWidth = (clampedPct / 100) * maxWidth;
    
    fillElement.setAttribute("width", newWidth);
    if (clampedPct < 20) {
        fillElement.style.fill = "#dc3545";
    } else {
        fillElement.style.fill = "#28a745";
    }
    
    textElement.textContent = Math.round(clampedPct) + "%";
}

function startBatteryMonitoring() {
    if (batteryInterval) return; 
    
    get_battery_health();
    batteryInterval = setInterval(get_battery_health, 5000);
}

function stopBatteryMonitoring() {
    if (batteryInterval) {
        clearInterval(batteryInterval);
        batteryInterval = null;
    }

    if (textElement) textElement.textContent = "XX%";
    if (fillElement) fillElement.setAttribute("width", "37");
}

function startCar() {
    if (!isConnected) {
        return;
    }

    startBtn.classList.remove("active");
    stopBtn.classList.add("active");

    startSensorMonitoring();
}


function stopCar() {
    if (!isConnected) {
        return;
    }

    stopBtn.classList.remove("active");
    startBtn.classList.add("active");

    stopSensorMonitoring();
}

function disableCar() {
    startBtn.classList.remove("active");
    stopBtn.classList.remove("active");
}

function setCarControls(isEnabled) {
    if (startBtn) {
        startBtn.disabled = !isEnabled;
        startBtn.classList.toggle("disabled", !isEnabled);
    }
    
    if (stopBtn) {
        stopBtn.disabled = !isEnabled;
        stopBtn.classList.toggle("disabled", !isEnabled);
    }
    
    if (!isEnabled) {
        disableCar();
    }
}

function startSensorMonitoring() {
    if (sensorInterval) return;

    get_sensor_data();

    sensorInterval = setInterval(get_sensor_data, 500);
}


function stopSensorMonitoring() {
    if (sensorInterval) {
        clearInterval(sensorInterval);
        sensorInterval = null;
    }
}

// Callout Functions
get_local_time();
setInterval(get_local_time, 1000)

connectionBtn.addEventListener("click", function() {
    const isClicked = !circleIcon.classList.contains("disabled") && !connectionBtn.classList.contains("disabled");

    if (isClicked) {
        isConnected = false;
        circleIcon.classList.add("disabled");
        connectionBtn.classList.add("disabled");
        connectionText.textContent = "NOT CONNECTED";
        stopBatteryMonitoring();
        setCarControls(false);
    } else {
        isConnected = true;
        circleIcon.classList.remove("disabled");
        connectionBtn.classList.remove("disabled");
        connectionText.textContent = "CONNECTED";
        startBtn.classList.add("active");
        connectionStartTime = new Date();
        sessionStorage.setItem("connectionStartTime", connectionStartTime.toISOString());
        startBatteryMonitoring();
        setCarControls(true);
    }
});

if (startBtn) startBtn.addEventListener("click", startCar);
if (stopBtn) stopBtn.addEventListener("click", stopCar);