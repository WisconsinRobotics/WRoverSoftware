const xcoordBox = document.getElementById('xcoord');
const ycoordBox = document.getElementById('ycoord');
const xvelocityBox = document.getElementById('xvelocity');
const yvelocityBox = document.getElementById('yvelocity');
const accelBox = document.getElementById('accel');
const distanceBox = document.getElementById('distance');
// const compassBox = document.getElementById('compass');
const compassNeedle = document.getElementById('needle');


let xcoord = 0;
let ycoord = 0;
let xvelocity = 0;
let yvelocity = 0;
let accel = 0;
let distance = 0;
let compass = 0;

setInterval(() => {

    xcoordBox.innerHTML = "X: " + xcoord.toFixed(2);
    ycoordBox.innerHTML = "Y: " + ycoord.toFixed(2);
    xvelocityBox.innerHTML = "X: " + xvelocity.toFixed(2) + " m/s";
    yvelocityBox.innerHTML = "Y: " + yvelocity.toFixed(2) + " m/s";
    accelBox.innerHTML = accel.toFixed(2) + " m/s^2";
    distanceBox.innerHTML = distance.toFixed(2) + " m";
    // compassBox.innerHTML = compass.toFixed(2) + " °";
    const degrees = parseFloat(compass) || 0;
    const clamped = ((degrees % 360) + 360) % 360; // Normalize to [0, 360)
    needle.style.transform = `rotate(${clamped}deg)`;
}, 500);
const tempBoxes = [];
const positionBoxes = [];
const currentBoxes = [];
const adc2Boxes = [];

for (let i = 1; i <= 7; i++) {
    tempBoxes.push(document.getElementById(`temp${i}`));
    positionBoxes.push(document.getElementById(`position${i}`));
    currentBoxes.push(document.getElementById(`current${i}`));
    adc2Boxes.push(document.getElementById(`adc${i}`));
}
                  