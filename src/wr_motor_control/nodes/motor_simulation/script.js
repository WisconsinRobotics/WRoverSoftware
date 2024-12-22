// global variables for the robot and its wheels
const body_width = 75;
const body_height = 100;

const wheel_width = 20;
const wheel_height = 40;

const start_pos_x = 150;
const start_pos_y = 150;

let body_x = start_pos_x;
let body_y = start_pos_y;
let body_angle = 0;
let v = 0;

const trailPositions = [];
const trailLength = 750; 

let reset = 1;

let rotational_velocity = 0;
let translational_velocity = [0,0];

let gamepad_index = null;

// F: front, B: back, L:keft, R:Right
const wheels = {
    FL_wheel_speed: 0,
    FL_wheel_angle: 0,

    FR_wheel_speed: 0,
    FR_wheel_angle: 0,

    BL_wheel_speed: 0,
    BL_wheel_angle: 0,

    BR_wheel_speed: 0,
    BR_wheel_angle: 0
};

// start the animation
function init() {
    setupSliders();
    window.requestAnimationFrame(draw);
}

// update values on every frame
document.addEventListener("DOMContentLoaded", function () {
    const speedInfo = document.getElementById("speed-info");
    const angleInfo = document.getElementById("angle-info");
    const positionInfo = document.getElementById("position-info");

    // update html 
    function updateInfo() {
        speedInfo.textContent = v.toFixed(2);
        angleInfo.textContent = body_angle.toFixed(2);
        positionInfo.textContent = `(${body_x.toFixed(2)}, ${body_y.toFixed(2)})`;

    }

    function updateWheelVelocities(){
        wheel_vectors = get_wheel_vectors(rotational_velocity, translational_velocity);
        let speeds = [get_wheel_speed(wheel_vectors[0]),get_wheel_speed(wheel_vectors[1]),
                      get_wheel_speed(wheel_vectors[2]),get_wheel_speed(wheel_vectors[3])]

        //Normalize speeds
        let max = 0
        for (let i = 0; i < speeds.length; i++){
            if(max<speeds[i]){
                max = speeds[i]
            }
        }
        if (max > 1) {
            for (let i = 0; i < speeds.length; i++){
                speeds[i] /= max;
            }
        }

        wheels.FL_wheel_speed = speeds[0];
        wheels.FL_wheel_angle = get_wheel_angle(wheel_vectors[0]);
        
        wheels.FR_wheel_speed = speeds[1];
        wheels.FR_wheel_angle = get_wheel_angle(wheel_vectors[1]);

        wheels.BL_wheel_speed = speeds[2];
        wheels.BL_wheel_angle = get_wheel_angle(wheel_vectors[2]);
        
        wheels.BR_wheel_speed = speeds[3];
        wheels.BR_wheel_angle = get_wheel_angle(wheel_vectors[3]);
    }
    // loop always
    function continuousUpdate() {
        updateInfo();
        if (gamepad_index != null){
            axes = getAxisValues(navigator.getGamepads()[gamepad_index])
            document.getElementById("ljx").innerHTML = axes[0]
            document.getElementById("ljy").innerHTML = axes[1]
            document.getElementById("rjx").innerHTML = axes[2]
            document.getElementById("rjy").innerHTML = axes[3]
            STR_AXIS = axes[0]
            FWD_AXIS = axes[1]
            RCW = axes[2]
            rotational_velocity = RCW * 0.5
            translational_velocity[0] = STR_AXIS * 5
            translational_velocity[1] = FWD_AXIS * 5
            console.log(body_x)
            console.log(body_y)
        }
        updateWheelVelocities();
        requestAnimationFrame(continuousUpdate); // Repeat at the next frame
    }

    // Start loop
    continuousUpdate();
});


// Setup
function setupSliders() {
    document.getElementById("FL_angle").addEventListener("input", (e) => wheels.FL_wheel_angle = parseFloat(e.target.value));
    document.getElementById("FL_speed").addEventListener("input", (e) => wheels.FL_wheel_speed = parseFloat(e.target.value));
    document.getElementById("FR_angle").addEventListener("input", (e) => wheels.FR_wheel_angle = parseFloat(e.target.value));
    document.getElementById("FR_speed").addEventListener("input", (e) => wheels.FR_wheel_speed = parseFloat(e.target.value));
    document.getElementById("BL_angle").addEventListener("input", (e) => wheels.BL_wheel_angle = parseFloat(e.target.value));
    document.getElementById("BL_speed").addEventListener("input", (e) => wheels.BL_wheel_speed = parseFloat(e.target.value));
    document.getElementById("BR_angle").addEventListener("input", (e) => wheels.BR_wheel_angle = parseFloat(e.target.value));
    document.getElementById("BR_speed").addEventListener("input", (e) => wheels.BR_wheel_speed = parseFloat(e.target.value));
    document.getElementById("rotational_velocity").addEventListener("input", (e) => rotational_velocity = parseFloat(e.target.value));
    document.getElementById("translational_velocity").addEventListener("input", (e) => translational_velocity = parseFloat(e.target.value));
    window.addEventListener("gamepadconnected", (event) => {
        handleConnectDisconnect(event, true)
    })
    window.addEventListener("gamepaddisconnected", (event) => {
        handleConnectDisconnect(event, true)
    })

}

// Handle Gamepad connect disconnect
function handleConnectDisconnect(event, connected) {
    if(!connected){
        document.getElementById("gamepad-connected-info").innerHTML = "False"
        document.getElementById("gamepad-name-info").innerHTML = "---"
    }else{
        document.getElementById("gamepad-connected-info").innerHTML = "True"
        const gamepad = event.gamepad
        document.getElementById("gamepad-name-info").innerHTML = gamepad.id
        gamepad_index = gamepad.index
    }
}

// Get axis values
function getAxisValues(gamepad) {
    let axisVals = [0,0,0,0]
    for (let i = 0; i < gamepad.axes.length; i++){
        if (gamepad.axes[i] > 0.06 || gamepad.axes[i] < -0.06){
            axisVals[i] = gamepad.axes[i]
        }
    }
    return axisVals
}

// calculations
function reset_velocities(){
    rotational_velocity = 0;
    translational_velocity = [0,0];
    document.getElementById("translational_velocity").value = 0;
    document.getElementById("rotational_velocity").value = 0;
    body_angle = 0;
}

function get_wheel_vectors(rotational_velocity, vehicle_translation){
    // x component is vehicle_translation +/- rot_vel*body_width/2
    // y component is vehicle_translation +/- rot_vel*body_height/2
    // + or - depends on what wheel it is

    // see "derivation of inverse kinematics for swerve" figure 5
    // possible x components 
    A = vehicle_translation[0] - ((rotational_velocity)*(body_height/2));
    B = vehicle_translation[0] + ((rotational_velocity)*(body_height/2));

    // possible y components
    C = vehicle_translation[1] - ((rotational_velocity)*(body_width/2));
    D = vehicle_translation[1] + ((rotational_velocity)*(body_width/2));

    FL_vector = [B, D];
    FR_vector = [B, C];
    BL_vector = [A, D];
    BR_vector = [A, C];

    // returns a list containing vectors [front left, front right, back left, back right]
    return [FL_vector, FR_vector, BL_vector, BR_vector];

}

function get_wheel_speed(wheel_vector){
    // sqrt(x_component^2 + y_component^2)
    return Math.sqrt(wheel_vector[0]*wheel_vector[0] + wheel_vector[1]*wheel_vector[1]);
}

function get_wheel_angle(wheel_vector){
    // arctan2(x_component, y_component)
    // zero is straight ahead (need to verify what angle range in radians is)
    return Math.atan2(wheel_vector[0], wheel_vector[1]);
}

// TODO: complete the function to simulate rover
function move_rover(wheels_group) {
    // const FL_velocity = wheels_group.FL_wheel_speed * Math.cos(wheels_group.FL_wheel_angle);
    // const FR_velocity = wheels_group.FR_wheel_speed * Math.cos(wheels_group.FR_wheel_angle);
    // const BL_velocity = wheels_group.BL_wheel_speed * Math.cos(wheels_group.BL_wheel_angle);
    // const BR_velocity = wheels_group.BR_wheel_speed * Math.cos(wheels_group.BR_wheel_angle);

    const FL_velocity = wheels_group.FL_wheel_speed
    const FR_velocity = wheels_group.FR_wheel_speed
    const BL_velocity = wheels_group.BL_wheel_speed
    const BR_velocity = wheels_group.BR_wheel_speed
    // velocity and angle
    v = (FL_velocity + FR_velocity + BL_velocity + BR_velocity) / 4;
    let w = (FR_velocity - FL_velocity + BR_velocity - BL_velocity) / 20; 

    // update pos and angle
    body_angle += (rotational_velocity * 0.05);
    // body_x += (translational_velocity[0] * Math.cos(body_angle) - translational_velocity[1] * Math.sin(body_angle)) * 0.5;
    // body_y += (translational_velocity[0] * Math.sin(body_angle) + translational_velocity[1] * Math.cos(body_angle)) * 0.5;
    body_x += (translational_velocity[0]);
    body_y += (translational_velocity[1]);

    /* below is quality of life stuff, keep to make things easier, change it as necessary */

    // bound it so that it comes back
    if(body_x > canvas.width) {
        body_x = 0;
    }
    if(body_x < 0) {
        body_x = canvas.width;
    }
    if(body_y > canvas.height) {
        body_y = 0;
    }
    if(body_y < 0) {
        body_y = canvas.height;
    }

    // add trail dot
    trailPositions.push({ x: body_x, y: body_y });

    // crunch trail
    if (trailPositions.length > trailLength) {
        trailPositions.shift();
    }

}

// draw wheels
function draw_wheel(context, width, height, x_translation, y_translation, color, angle, speed, label_displacement) {
    // draw and rotate wheel
    context.save();
    context.translate(x_translation, y_translation);
    context.translate(width / 2, height / 2);
    context.rotate(angle - body_angle);
    context.fillStyle = color;
    context.fillRect(-width / 2, -height / 2, width, height);

    // labels
    context.fillStyle = "black";
    context.font = "12px Arial";
    context.textAlign = "center"; 
    context.fillText("Angle: " + angle.toFixed(2), label_displacement, height / 2); // First line
    context.fillText("Speed: " + speed.toFixed(2), label_displacement, height / 2 + 15); // Second line

    context.restore();
}

function draw() {
    // setup the canvas
    const canvas = document.getElementById("canvas");
    const ctx = canvas.getContext("2d");

    // // move rover so we can see it
    if(reset == 1) {
        body_x = start_pos_x;
        body_y = start_pos_y;
        reset = 0;
    }

    // // clear and save for current iteration
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    // ctx.save();

    // // calculate next position of the robot based on the directions of the wheels and their speeds
    move_rover(wheels);

    // // path out trail
    ctx.beginPath();
    ctx.strokeStyle = "#00FF00";
    ctx.lineWidth = 2;

    // // make link and stroke
    for (let i = 0; i < trailPositions.length; i++) {
        const pos = trailPositions[i];
        if (i === 0) {
            ctx.moveTo(pos.x, pos.y);
        } else {
            ctx.lineTo(pos.x, pos.y);
        }
    }
    ctx.stroke();

    // // create body
    ctx.save();
    ctx.translate(body_x, body_y);
    ctx.rotate(body_angle);
    ctx.fillStyle = "#FF0000";
    ctx.fillRect(-body_width / 2, -body_height / 2, body_width, body_height);
    // ctx.resetore();

    // // add the label
    ctx.fillStyle = "black";
    ctx.font = "12px Arial";
    ctx.textAlign = "center"; 
    ctx.fillText("Front", 0, body_height / 2 - 5);

    // // front right wheel
    ctx.save();
    draw_wheel(
        ctx, 
        wheel_width, 
        wheel_height, 
        (-wheel_width / 2) - (body_width / 2), 
        (-wheel_height / 2) + (body_height / 2), 
        "#0000FF", 
        wheels.FR_wheel_angle, 
        wheels.FR_wheel_speed,
        -50
    );
    ctx.restore();

    // // front left wheel
    ctx.save();
    draw_wheel(
        ctx, 
        wheel_width, 
        wheel_height, 
        (-wheel_width / 2) + (body_width / 2), 
        (-wheel_height / 2) + (body_height / 2), 
        "#0000FF", 
        wheels.FL_wheel_angle, 
        wheels.FL_wheel_speed,
        50
    );
    ctx.restore();

    // // back left wheel
    ctx.save();
    draw_wheel(
        ctx, 
        wheel_width, 
        wheel_height, 
        (-wheel_width / 2) + (body_width / 2), 
        (-wheel_height / 2) - (body_height / 2), 
        "#0000FF", 
        wheels.BL_wheel_angle, 
        wheels.BL_wheel_speed,
        50
    );
    ctx.restore();

    // // back right wheel
    ctx.save();
    draw_wheel(
        ctx, 
        wheel_width, 
        wheel_height, 
        -(wheel_width / 2) - (body_width / 2), 
        -(wheel_height / 2) - (body_height / 2), 
        "#0000FF", 
        wheels.BR_wheel_angle, 
        wheels.BR_wheel_speed,
        -50
    );
    ctx.restore();  

    // // restore back to original
    ctx.restore();

    // draw
    window.requestAnimationFrame(draw);
}


init();
