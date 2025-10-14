// Gate and obstacle drawing functions (ported from gates.py)
/*function drawFlag(ctx, x, y, scale = 1) {
    ctx.save();
    ctx.translate(x, y);
    ctx.scale(scale, scale);
    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.moveTo(0, 66); ctx.lineTo(6, 60); ctx.lineTo(12, 66); ctx.closePath(); ctx.fill();
    ctx.beginPath();
    ctx.moveTo(6, 0); ctx.lineTo(15, 6); ctx.lineTo(15, 36); ctx.lineTo(6, 54); ctx.closePath(); ctx.fill();
    ctx.fillStyle = '#f00';
    ctx.beginPath(); ctx.moveTo(6, 12); ctx.lineTo(6, 0); ctx.lineTo(15, 6); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(6, 24); ctx.lineTo(6, 12); ctx.lineTo(15, 18); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(6, 36); ctx.lineTo(6, 24); ctx.lineTo(15, 30); ctx.closePath(); ctx.fill();
    ctx.restore();
}*/

function drawFlag2(ctx, x, y, scale = 1) {
    ctx.save();
    ctx.translate(x, y);
    ctx.scale(3, 3);
    
    //solid base tri
    ctx.fillStyle = '#fff';
    //pygame.draw.polygon(surface,WHITE,[(0*s,22*s),(2*s,20*s),(4*s,22*s)]) #solid base tri
    ctx.beginPath(); ctx.moveTo(0,22); ctx.lineTo(2,20); ctx.lineTo(4,22); ctx.closePath(); ctx.fill();
    //solid body
    ctx.fillStyle = '#fff';
    //pygame.draw.polygon(surface,WHITE,[(2*s,0*s),(5*s,2*s),(5*s,12*s),(2*s,18*s)]) #solid body
    ctx.beginPath(); ctx.moveTo(2,0); ctx.lineTo(5,2); ctx.lineTo(5,12); ctx.lineTo(2,18); ctx.closePath(); ctx.fill();    
    //top tri
    ctx.fillStyle = '#f00';
    //pygame.draw.polygon(surface,RED,[(2*s,4*s),(2*s,0*s),(5*s,2*s)])#top tri
    ctx.beginPath(); ctx.moveTo(2,4); ctx.lineTo(2,0); ctx.lineTo(5,2); ctx.closePath(); ctx.fill();   
    //mid tri
    ctx.fillStyle = '#f00';
    //pygame.draw.polygon(surface,RED,[(2*s,8*s),(2*s,4*s),(5*s,6*s)])#mid tri
    ctx.beginPath(); ctx.moveTo(2,8); ctx.lineTo(2,4); ctx.lineTo(5,6); ctx.closePath(); ctx.fill();
    //bottom tri
    ctx.fillStyle = '#f00';
    //pygame.draw.polygon(surface,RED,[(2*s,12*s),(2*s,8*s),(5*s,10*s)])#bottom tri
    ctx.beginPath(); ctx.moveTo(2,12); ctx.lineTo(2,8); ctx.lineTo(5,10); ctx.closePath(); ctx.fill();
    //outline
    //pygame.draw.aalines(surface,WHITE,False,[(2*s,20*s),(2*s,0*s),(5*s,2*s),(5*s,12*s),(2*s,18*s)])#outline
    ctx.strokeStyle = '#fff';
    ctx.beginPath(); ctx.moveTo(2,20); ctx.lineTo(2,0); ctx.lineTo(5,2); ctx.lineTo(5,12); ctx.lineTo(2,18); ctx.closePath(); ctx.stroke();
    ctx.restore();
}

function drawXGate(ctx, x, y, scale = 1) {
    ctx.save();
    ctx.translate(x, y);
    ctx.scale(scale, scale);
    ctx.fillStyle = '#fff';
    ctx.beginPath(); ctx.moveTo(0, 144); ctx.lineTo(16, 108); ctx.lineTo(40, 108); ctx.lineTo(24, 144); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(128, 144); ctx.lineTo(104, 144); ctx.lineTo(88, 108); ctx.lineTo(112, 108); ctx.closePath(); ctx.fill();
    ctx.fillStyle = '#f00';
    ctx.beginPath(); ctx.moveTo(16, 108); ctx.lineTo(32, 72); ctx.lineTo(64, 72); ctx.lineTo(64, 98); ctx.lineTo(40, 108); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(112, 108); ctx.lineTo(88, 108); ctx.lineTo(64, 98); ctx.lineTo(64, 72); ctx.lineTo(96, 72); ctx.closePath(); ctx.fill();
    ctx.fillStyle = '#f00';
    ctx.beginPath(); ctx.moveTo(48, 36); ctx.lineTo(64, 0); ctx.lineTo(80, 36); ctx.closePath(); ctx.fill();
    ctx.restore();
}

function drawXGate2(ctx, x, y, scale=1) {
    ctx.save();
    ctx.translate(x,y);
    ctx.scale(6,6);
    //left leg
    ctx.fillStyle = '#fff';
    //    (0*s,24*s), (4*s,18*s), (10*s,18*s), (6*s,24*s)
    ctx.beginPath(); ctx.moveTo(0,24); ctx.lineTo(4,18); ctx.lineTo(10,18); ctx.lineTo(6,24); ctx.closePath(); ctx.fill();
    
    //left upper
    ctx.fillStyle = '#f00';
    //    (4*s,18*s), (8*s,12*s), (16*s,12*s), (16*s,16*s), (10*s,18*s)
    ctx.beginPath(); ctx.moveTo(4,18); ctx.lineTo(8,12); ctx.lineTo(16,12); ctx.lineTo(16,16); ctx.lineTo(10,18); ctx.closePath(); ctx.fill();

    //right leg
    ctx.fillStyle = '#fff';
    //    (32*s,24*s), (26*s,24*s), (22*s,18*s), (28*s,18*s)
    ctx.beginPath(); ctx.moveTo(32,24); ctx.lineTo(26,24); ctx.lineTo(22,18); ctx.lineTo(28,18); ctx.closePath(); ctx.fill();

    //right upper
    ctx.fillStyle = '#f00';
    //    (28*s,18*s), (22*s,18*s), (16*s,16*s), (16*s,12*s), (24*s,12*s)
    ctx.beginPath(); ctx.moveTo(28,18); ctx.lineTo(22,18); ctx.lineTo(16,16); ctx.lineTo(16,12); ctx.lineTo(24,12); ctx.closePath(); ctx.fill();

    //top tri
    ctx.fillStyle = '#f00';
    //(12*s,6*s), (16*s,0*s), (20*s,6*s)]
    ctx.beginPath(); ctx.moveTo(12,6); ctx.lineTo(16,0); ctx.lineTo(20,6); ctx.closePath(); ctx.fill();
    
    ctx.restore();
}

function drawGate(ctx, x, y, scale = 1) {
    ctx.save();
    ctx.translate(x, y);
    ctx.scale(scale, scale);
    ctx.fillStyle = '#fff';
    ctx.beginPath(); ctx.moveTo(0, 48); ctx.lineTo(0, 24); ctx.lineTo(8, 12); ctx.lineTo(24, 0); ctx.lineTo(32, 0); ctx.lineTo(32,12);
    ctx.lineTo(16, 18); ctx.lineTo(8, 30); ctx.lineTo(8, 48); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(32, 0); ctx.lineTo(40, 0); ctx.lineTo(56, 12); ctx.lineTo(64, 24); ctx.lineTo(64, 48);
    ctx.lineTo(56, 48); ctx.lineTo(56, 30); ctx.lineTo(48, 18); ctx.lineTo(32, 12); ctx.closePath(); ctx.fill();
    ctx.fillStyle = '#f00';
    //ctx.beginPath(); ctx.moveTo(12, 6); ctx.lineTo(18, 2); ctx.lineTo(17, 6); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(32, 9); ctx.lineTo(26, 3); ctx.lineTo(38, 3); ctx.closePath(); ctx.fill();
    //(8*s,1.5*s), (6.5*s,0.5*s), (9.5*s,0.5*s)
    ctx.fillStyle = '#f0f';
    //ctx.beginPath(); ctx.moveTo(39, 6); ctx.lineTo(30, 2); ctx.lineTo(31, 6); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(12, 12); ctx.lineTo(24, 3); ctx.lineTo(22, 12); ctx.closePath(); ctx.fill();
    //(3*s,2*s), (6*s,0.5*s), (5.5*s,2*s)
    //ctx.beginPath(); ctx.moveTo(7, 24); ctx.lineTo(7, 18); ctx.lineTo(2, 24); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(52, 12); ctx.lineTo(40, 3); ctx.lineTo(42, 12); ctx.closePath(); ctx.fill();
    //(13*s,2*s), (10*s,0.5*s), (10.5*s,2*s)
    //ctx.beginPath(); ctx.moveTo(41, 24); ctx.lineTo(41, 18); ctx.lineTo(46, 24); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(10, 24); ctx.lineTo(10, 15); ctx.lineTo(4, 24); ctx.closePath(); ctx.fill();
    //(2.5*s,4*s), (2.5*s,2.5*s), (1*s,4*s)
    //ctx.beginPath(); ctx.moveTo(3, 36); ctx.lineTo(1, 42); ctx.lineTo(1, 30); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(54, 24); ctx.lineTo(54, 15); ctx.lineTo(60, 24); ctx.closePath(); ctx.fill();
    //(13.5*s,4*s), (13.5*s,2.5*s), (15*s,4*s)
    //ctx.beginPath(); ctx.moveTo(45, 36); ctx.lineTo(47, 42); ctx.lineTo(47, 30); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(6, 36); ctx.lineTo(2, 42); ctx.lineTo(2, 30); ctx.closePath(); ctx.fill();
    //(1.5*s,6*s), (0.5*s,7*s), (0.5*s,5*s)
    //
    ctx.beginPath(); ctx.moveTo(58, 36); ctx.lineTo(62, 42); ctx.lineTo(62, 30); ctx.closePath(); ctx.fill();
    //(14.5*s,6*s), (15.5*s,7*s), (15.5*s,5*s)
    ctx.restore();
}

function drawGate2(ctx, x, y, scale=1) {
    ctx.save();
    ctx.translate(x,y);
    ctx.scale(6,6);
    //left
    ctx.fillStyle = '#fff';
    //    (0*s,8*s), (0*s,4*s), (2*s,2*s), (6*s,0*s), (8*s,0*s), (8*s,2*s),
    //    (4*s,3*s), (2*s,5*s), (2*s,8*s)
    ctx.beginPath(); ctx.moveTo(0,8); ctx.lineTo(0,4); ctx.lineTo(2,2); ctx.lineTo(6,0); ctx.lineTo(8,0); ctx.lineTo(8,2);
    ctx.lineTo(4,3); ctx.lineTo(2,5); ctx.lineTo(2,8); ctx.closePath(); ctx.fill();

    //right
    //    (8*s,0*s), (10*s,0*s), (14*s,2*s), (16*s,4*s), (16*s,8*s),
    //    (14*s,8*s), (14*s,5*s), (12*s,3*s), (8*s,2*s)
    ctx.beginPath(); ctx.moveTo(8,0); ctx.lineTo(10,0); ctx.lineTo(14,2); ctx.lineTo(16,4); ctx.lineTo(16,8);
    ctx.lineTo(14,8); ctx.lineTo(14,5); ctx.lineTo(12,3); ctx.lineTo(8,2); ctx.closePath(); ctx.fill();
    
    //centre
    ctx.fillStyle = '#f00';
    //    (8*s,1.5*s), (6.5*s,0.5*s), (9.5*s,0.5*s)
    ctx.beginPath(); ctx.moveTo(8,1.5); ctx.lineTo(6.5,0.5); ctx.lineTo(9.5,0.5); ctx.closePath(); ctx.fill();
    //left
    ctx.fillStyle = '#f0f';
    //    (3*s,2*s), (6*s,0.5*s), (5.5*s,2*s)
    ctx.beginPath(); ctx.moveTo(3,2); ctx.lineTo(6,0.5); ctx.lineTo(5.5,2); ctx.closePath(); ctx.fill();
    //right
    //    (13*s,2*s), (10*s,0.5*s), (10.5*s,2*s)
    ctx.beginPath(); ctx.moveTo(13,2); ctx.lineTo(10,0.5); ctx.lineTo(10.5,2); ctx.closePath(); ctx.fill();
    //further left
    //    (2.5*s,4*s), (2.5*s,2.5*s), (1*s,4*s)
    ctx.beginPath(); ctx.moveTo(2.5,4); ctx.lineTo(2.5,2.5); ctx.lineTo(1,4); ctx.closePath(); ctx.fill();
    //further right
    //    (13.5*s,4*s), (13.5*s,2.5*s), (15*s,4*s)
    ctx.beginPath(); ctx.moveTo(13.5,4); ctx.lineTo(13.5,2.5); ctx.lineTo(15,4); ctx.closePath(); ctx.fill();
    //left vertical
    //    (1.5*s,6*s), (0.5*s,7*s), (0.5*s,5*s)
    ctx.beginPath(); ctx.moveTo(1.5,6); ctx.lineTo(0.5,7); ctx.lineTo(0.5,5); ctx.closePath(); ctx.fill();
    //right vertical
    //    (14.5*s,6*s), (15.5*s,7*s), (15.5*s,5*s)
    ctx.beginPath(); ctx.moveTo(14.5,6); ctx.lineTo(15.5,7); ctx.lineTo(15.5,5); ctx.closePath(); ctx.fill();

    ctx.restore();
}

/*function drawTunnel(ctx, x, y, scale = 1) {
    ctx.save();
    ctx.translate(x, y);
    ctx.scale(scale, scale);
    ctx.fillStyle = '#fff';
    ctx.beginPath(); ctx.moveTo(0, 40); ctx.lineTo(0, 8); ctx.lineTo(76, 8); ctx.lineTo(68, 16); ctx.lineTo(68, 40); ctx.closePath(); ctx.fill();
    ctx.fillStyle = '#f00';
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(92, 0); ctx.lineTo(76, 8); ctx.lineTo(0, 8); ctx.closePath(); ctx.fill();
    ctx.fillStyle = '#f0f';
    ctx.beginPath(); ctx.moveTo(68, 40); ctx.lineTo(68, 16); ctx.lineTo(76, 8); ctx.lineTo(92, 0); ctx.lineTo(100, 0); ctx.lineTo(100, 8); ctx.lineTo(76, 20); ctx.lineTo(76, 40); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.moveTo(100, 0); ctx.lineTo(108, 0); ctx.lineTo(124, 8); ctx.lineTo(132, 16); ctx.lineTo(132, 40); ctx.lineTo(124, 40); ctx.lineTo(124, 20); ctx.lineTo(100, 8); ctx.closePath(); ctx.fill();
    ctx.restore();
}*/

function drawTunnel2(ctx,x,y,scale=1) {
    ctx.save();
    ctx.translate(x,y)
    ctx.scale(4,4);
    //tunnel
    ctx.fillStyle = '#fff';
    ctx.beginPath(); ctx.moveTo(0,10); ctx.lineTo(0,2); ctx.lineTo(19,2); ctx.lineTo(17,4); ctx.lineTo(17,10); ctx.closePath(); ctx.fill();
    //red
    ctx.fillStyle = '#f00';
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(23,0); ctx.lineTo(19,2); ctx.lineTo(0,2); ctx.closePath(); ctx.fill();
    //opening left
    ctx.fillStyle = '#f0f';
    ctx.beginPath(); ctx.moveTo(17,10); ctx.lineTo(17,4); ctx.lineTo(19,2); ctx.lineTo(23,0); ctx.lineTo(25,0);
    ctx.lineTo(25,2); ctx.lineTo(19,5); ctx.lineTo(19,10); ctx.closePath(); ctx.fill();
    //opening right
    ctx.fillStyle = '#f0f';
    ctx.beginPath(); ctx.moveTo(25,0); ctx.lineTo(27,0); ctx.lineTo(31,2); ctx.lineTo(33,4); ctx.lineTo(33,10);
    ctx.lineTo(31,10); ctx.lineTo(31,5); ctx.lineTo(25,2); ctx.closePath(); ctx.fill();
    ctx.restore();
}

/*function drawStartFinish(ctx, x, y, scale = 1) {
    ctx.save();
    ctx.translate(x, y);
    ctx.scale(scale, scale);
    ctx.fillStyle = '#fff';
    ctx.fillRect(0, 0, 64, 16);
    ctx.fillStyle = '#f0f';
    for (let i = 0; i < 4; i++) {
        ctx.fillRect(i * 16, 0, 8, 8);
        ctx.fillRect(i * 16 + 8, 8, 8, 8);
    }
    ctx.restore();
}*/

function drawStartFinish2(ctx, x, y, scale=1) {
    size=4; //size of square, where spacing between same colours is spacing*2
    ctx.save();
    ctx.translate(x,y);
    ctx.scale(2,2);
    ctx.fillStyle = '#fff';
    //(0*s,0*s), (32*s,0*s), (32*s,8*s), (0*s,8*s)
    ctx.fillRect(0,0,32,8);
    ctx.fillStyle = '#f0f';
    let dx=size*2; //offset for bottom row - NOTE scale is in the context
    for (let x=0; x<4; x++) //count of how many coloured squares fit in 32 pixels i.e. 32/(size*2)
    {
        //top line
        ctx.fillRect(x*dx,0,size,size);
        //bottom line x offset
        ctx.fillRect(x*dx+size,size,size,size);
    }
    ctx.restore();
}

// CourseSequencer class (ported from coursesequencer.py)
class CourseSequencer {
    constructor() {
        this.sequence = []; // {rect: {x, y, size}, jump: {x, y}}
        this.next = 0;
    }

    startLap() {
        this.next = 0;
    }

    addWaypoint(x, y, size, jumpx, jumpy) {
        // x, y are center; size is half-side (square)
        this.sequence.push({
            rect: {x: x, y: y, size: size},
            jump: {x: jumpx, y: jumpy}
        });
    }

    testWaypoint(x, y) {
        // x, y are drone screen coordinates (pixels)
        if (this.next >= this.sequence.length) return {isHit: false, isLapComplete: false, jump: {x:0, y:0}};
        const wp = this.sequence[this.next];
        const r = wp.rect;
        const isHit = (x >= r.x - r.size && x <= r.x + r.size && y >= r.y - r.size && y <= r.y + r.size);
        let isLapComplete = false;
        let jump = {x:0, y:0};
        if (isHit) {
            jump = wp.jump;
            this.next++;
            if (this.next >= this.sequence.length) {
                isLapComplete = true;
            }
        }
        return {isHit, isLapComplete, jump};
    }

    getNextRect() {
        if (this.next >= this.sequence.length) return null;
        return this.sequence[this.next].rect;
    }
}
// PID Controller class (ported from PIDController.py)
class PIDController {
    constructor(Kp, Ki, Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.x0 = 0; // previous input value
        this.e0 = 0; // previous error value
        this.I = 0; // Integral accumulator
        this.D = 0; // Derivative accumulator
        this.u = 0; // last output value
    }

    // Run the PID calculation
    process(x, y, deltaT) {
        const e = y - x; // current error
        this.I += e * deltaT;
        this.D = (e - this.e0) / deltaT;
        this.u = this.Kp * e + this.Ki * this.I + this.Kd * this.D;
        this.e0 = e;
        this.x0 = x;
        return this.u;
    }
}
// Drone physics, controls, and main game loop for browser-based drone racer
// This is the first step: physics, controls, and rendering loop only

const canvas = document.getElementById('droneCanvas');
const ctx = canvas.getContext('2d');

// Constants (from droneracer.py)
const screenScale = 100;
const SCREEN_WIDTH = 640;
const SCREEN_HEIGHT = 480;
const gravity = 9.81;
const mass = 0.4;
const Izz = 0.0012;
const Cla = 0.8;
const Cld = 0.003;
const Cd = 0.5;
const maxThrust = 4.5;

// Drone state
let drone = {
    x: SCREEN_WIDTH / 2 / screenScale,
    y: SCREEN_HEIGHT / screenScale,
    vX: 0,
    vY: 0,
    aX: 0,
    aY: 0,
    theta: 0, // roll angle (rads)
    thetaDot: 0, // roll acceleration
    thrust: 0,
    aileron: 0,
    throttle: 0,
    isAltLock: false,
    altLockHeight: 0
};

// Input state
let input = {
    throttleUp: false,
    throttleDown: false,
    aileronLeft: false,
    aileronRight: false
};

//lap timing and game state
let lapTimer = 0;
let completedLaps = [];
let isRaceRunning = false;
let animationTimer = 0;

//PID Controllers
heightPID = new PIDController(0.4,0.025,0.35) //5,1.0,0.5 #5,0.5,0.5
rollVelPID = new PIDController(0.03,0.04,0.000005) //0.03,0.04,0.000005
rollAnglePID = new PIDController(0.06,0.01,0.005)
throttleNudgePID = new PIDController(0.4,0,0)
aileronNudgePID = new PIDController(0.1,0.02,0.005)

function handleKeyDown(e) {
    switch (e.key) {
        case 'ArrowUp', 'p':
            input.throttleUp = true;
            break;
        case 'ArrowDown', 'l':
            input.throttleDown = true;
            break;
        case 'ArrowLeft', 'q':
            input.aileronLeft = true;
            break;
        case 'ArrowRight', 'w':
            input.aileronRight = true;
            break;
        case 'h':
            drone.altLockHeight = drone.y;
            drone.isAltLock = !drone.isAltLock;
            break;
    }
}

function handleKeyUp(e) {
    switch (e.key) {
        case 'ArrowUp', 'p':
            input.throttleUp = false;
            break;
        case 'ArrowDown', 'l':
            input.throttleDown = false;
            break;
        case 'ArrowLeft', 'q':
            input.aileronLeft = false;
            break;
        case 'ArrowRight', 'w':
            input.aileronRight = false;
            break;
    }
}

document.addEventListener('keydown', handleKeyDown);
document.addEventListener('keyup', handleKeyUp);


function updateDrone(dt) {
    // Controls
    if (input.throttleUp) {
        drone.throttle = Math.min(drone.throttle + 0.05, 1.0);
    } else if (input.throttleDown) {
        drone.throttle = Math.max(drone.throttle - 0.05, -1.0);
    } else {
        drone.throttle = 0;
    }
    if (input.aileronLeft) {
        drone.aileron = Math.max(drone.aileron - 0.05, -1.0);
    } else if (input.aileronRight) {
        drone.aileron = Math.min(drone.aileron + 0.05, 1.0);
    } else {
        drone.aileron = 0;
    }

    // Thrust (no PID yet)
    //xxx drone.thrust = (mass * gravity) + (maxThrust / 2) * drone.throttle;
    //xxx drone.thrust = Math.max(0, Math.min(maxThrust, drone.thrust));
    //use a PID to link throttle position to a desired vertical speed (vY)
    drone.thrust = (mass*gravity) + (maxThrust/2)*throttleNudgePID.process(drone.vY,drone.throttle,dt);
    drone.thrust = Math.max(0,drone.thrust);
    drone.thrust = Math.min(maxThrust,drone.thrust);

    // Angular dynamics (closer to Python)
    //XXX let a2 = drone.aileron * Cla;
    a2 = aileronNudgePID.process(drone.vX,drone.aileron*Cla,dt);
    let L = a2 - Cld * drone.thetaDot; //L (torque) = roll power (torque) minus drag
    let pDot = L / Izz; //A=F/m
    drone.thetaDot = pDot * dt; // integrate pDot to get thetaDot
    drone.theta += drone.thetaDot * dt; // integrate thetaDot to get theta 

    // Speed and drag (closer to Python)
    let vsq = drone.vX * drone.vX + drone.vY * drone.vY;
    let v = vsq > 0 ? Math.sqrt(vsq) : 0;
    let dx = v > 0 ? drone.vX / v : 0;
    let dy = v > 0 ? drone.vY / v : 0;
    let fX = drone.thrust * Math.sin(drone.theta) - Cd * vsq * dx;
    let fY = -mass * gravity + drone.thrust * Math.cos(drone.theta) - Cd * vsq * dy;
    drone.aX = fX / mass;
    drone.aY = fY / mass;
    drone.vX += drone.aX * dt;
    drone.vY += drone.aY * dt;
    drone.x += drone.vX * dt;
    drone.y += drone.vY * dt;

    // Bounds and ground collision (closer to Python)
    if (drone.x < 25 / screenScale) {
        drone.x = 25 / screenScale;
        drone.vX = -drone.vX;
        drone.theta = -drone.theta;
    }
    if (drone.x > (SCREEN_WIDTH - 25) / screenScale) {
        drone.x = (SCREEN_WIDTH - 25) / screenScale;
        drone.vX = -drone.vX;
        drone.theta = -drone.theta;
    }
    if (drone.y <= 11 / screenScale) {
        drone.y = 11 / screenScale;
        drone.theta = 0;
        drone.vX = 0.4 * drone.vX;
        drone.vY = -0.4 * drone.vY;
    }
    //collision detection
    const GREEN = {r:0, g:255, b:0, a:255};
    const RED = {r:255,g:0,b:0,a:255};
    const sX = Math.floor(drone.x * screenScale);
    const sY = Math.floor(SCREEN_HEIGHT - drone.y * screenScale);
    // read pixel color at canvas pixel coords
    let lowPixel = getPixelColourAtCanvas(canvas, sX, sY+8); //screen.get_at((sX,sY+8))
    let leftPixel = getPixelColourAtCanvas(canvas,sX-20,sY); //screen.get_at((sX-20,sY))
    let rightPixel = getPixelColourAtCanvas(canvas,sX+20,sY); //screen.get_at((sX+20,sY))
    if (colourMatch(lowPixel,GREEN) || colourMatch(lowPixel,RED)) { // crash down
        drone.theta = 0;
        drone.vX = 0.4 * drone.vX;
        drone.vY = -0.4 * drone.vY;
        drone.y += drone.vY*2.5/30; //WAS pY=  stop it going through the ground
    }
    if (colourMatch(leftPixel,RED)) { //crash left (this is useful, the constant vX PID makes it spin out of control!)
        drone.vX = -1.2 * Math.abs(drone.vX); //was 0.4
    }
    if (colourMatch(rightPixel,RED)) { //crash right
        drone.vX = -1.2 * Math.abs(drone.vX);
    }
}

function getPixelColourAtCanvas(canvas, clientX, clientY) {
  const rect = canvas.getBoundingClientRect();
  const x=Math.floor(clientX);
  const y=Math.floor(clientY);

  const ctx = canvas.getContext('2d');
  // This can throw if the canvas is tainted by cross-origin images
  const data = ctx.getImageData(x, y, 1, 1).data;
  return { r: data[0], g: data[1], b: data[2], a: data[3] }; // a is 0-255
}


function colourMatch(c1,c2) {
    const threshold=60; //tolerance for matching (to allow for anti-aliasing)
    //console.log(c1,c2);
    if (c1==undefined || c2==undefined) return false;
    return (Math.abs(c1.r-c2.r)<threshold)
        && (Math.abs(c1.g-c2.g)<threshold)
        && (Math.abs(c1.b-c2.b)<threshold);
}

/*function drawDrone(x, y, a) {
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-a); // negative for canvas y-down
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-18, 2);
    ctx.lineTo(18, 2);
    ctx.moveTo(-7, -8);
    ctx.lineTo(7, -8);
    ctx.moveTo(-10, 8);
    ctx.lineTo(10, 8);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(0, 0, 8, 0, Math.PI * 2);
    ctx.fillStyle = '#00ffcc';
    ctx.fill();
    ctx.stroke();
    ctx.restore();
}*/

function drawDrone2(x,y,a) {
    pts = [
        [-9,1], [-9,2], [9,2], [9,1], /*central slice*/
        [-7,-4], [-4,0], [-3,1], [3,1], [4,0], [7,-4], [2,0], [-2,0], /*feet*/
        [-2,2], [-1,3], [1,3], [2,2], /*top*/
        [-3,6], [3,6], /*antennae*/
        [-11,1], [-11,4], [-9,4], /*l motor*/
        [9,4], [11,4], [11,1], /*r motor*/
        [-14,5], [-10,5], [-6,5], /*l prop*/
        [6,5], [10,5], [14,5] /*r prop*/
    ];
    verts = [
        [1,2], [0,3], /*central slice*/
        [4,5], [5,6], [11,4], /* l leg*/
        [7,8], [8,9], [9,10], [10,11], /*r leg*/
        [12,13], [13,14], [14,15], /*top*/
        [13,16], /* l antenna*/
        [14,17], /* r anetnna*/
        [18,19], [19,20], [20,0], [0,18], /* l motor*/
        [3,21], [21,22], [22,23], [23,3], /*r motor*/
        [24,25], [25,26], /*l props*/
        [27,28], [28,29] /*r props*/
    ];
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(a); // negative for canvas y-down
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;

    //drawing
    s=screenScale/100;
    //ctx.beginPath();
    for (i=0; i<verts.length; i++) {
        v=verts[i];
        p0=[pts[v[0]][0],-pts[v[0]][1]];
        p1=[pts[v[1]][0],-pts[v[1]][1]];
        ctx.moveTo(s*p0[0], s*p0[1]);
        ctx.lineTo(s*p1[0], s*p1[1]);
        //ctx.stroke();
    }
    ctx.stroke();

    //ctx.moveTo(0,0);
    //ctx.lineTo(-90,10);
    //ctx.moveTo(-90,10);
    //ctx.lineTo(-90,20);
    //ctx.stroke();
    ctx.restore();
}

function clearCanvas() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
}

// --- Course and game state ---
const course = new CourseSequencer();
// Add waypoints (from droneracer.py)
course.addWaypoint(331,166,20,-30,0);
course.addWaypoint(60,190,40,0,0);
course.addWaypoint(330,298,20,0,0);
course.addWaypoint(459,465,20,-30,0);
course.addWaypoint(238,465,20,-30,0);
course.addWaypoint(336,375,30,28,140);
course.addWaypoint(478,244,20,0,0);
course.addWaypoint(586,175,40,0,0); // start finish
drone.x = 5.65; // puts you on the grass under the start finish grid ready to go
drone.y = 2.38;
course.next = 7; // sets the waypoint to the start finish straight where timing starts



function drawWaypoint(rect, state) {
    if (!rect) return;
    let cx = rect.x;
    let cy = rect.y;
    let s = rect.size;
    let grow = state > 16 ? 32 - state : state;
    let size = s * grow / 24.0;
    ctx.save();
    ctx.strokeStyle = '#ff0';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(cx, cy - size); ctx.lineTo(cx, cy + size);
    ctx.moveTo(cx - size, cy); ctx.lineTo(cx + size, cy);
    ctx.moveTo(cx - size, cy - size); ctx.lineTo(cx + size, cy + size);
    ctx.moveTo(cx + size, cy - size); ctx.lineTo(cx - size, cy + size);
    ctx.stroke();
    ctx.restore();
}

function drawTiming(lapTime, laps) {
    // Draw lap time and last 3 laps
    laps = laps || [];
    ctx.save();
    ctx.fillStyle = '#fff';
    ctx.font = '16px monospace';
    let previousTimes = '';
    for (let i = laps.length - 1; i >= Math.max(0, laps.length - 3); i--) {
        previousTimes += `${i + 1}: ${laps[i].toFixed(3)}   `;
    }
    let secs = lapTime / 30.0;
    ctx.fillText(`Lap Time: ${secs.toFixed(3)}   ${previousTimes}`, 20, 30);
    ctx.restore();
}

function drawCourseObstacles() {
    // Draw obstacles and gates (positions from droneracer.py)
    drawFlag2(ctx, 80, 120, 1);
    ctx.fillStyle = '#0f0'; ctx.fillRect(70, 188, 30, 10);
    drawFlag2(ctx, 500, 120, 1);
    ctx.fillStyle = '#0f0'; ctx.fillRect(490, 188, 30, 10);
    ctx.fillRect(610, 188, 40, 10);
    drawFlag2(ctx, 80, 360, 1);
    drawFlag2(ctx, 520, 360, 1);
    drawGate2(ctx, 420, 200, 1);
    drawXGate2(ctx, 224, 232, 1);
    drawGate2(ctx, 180, 420, 1);
    drawGate2(ctx, 400, 420, 1);
    drawTunnel2(ctx, 220, 130, 1);
    drawStartFinish2(ctx, 534, 185, 1);
    ctx.fillStyle = '#0f0';
    ctx.fillRect(100, 170, 390, 10);
    ctx.fillRect(0, 306, 255, 10);
    ctx.fillRect(360, 250, 260, 10);
    ctx.fillRect(0, 470, 640, 10);
    ctx.fillRect(140, 380, 360, 10); //xgate
}

function gameLoop() {
    clearCanvas();
    drawCourseObstacles();
    drawWaypoint(course.getNextRect(), animationTimer);
    // Convert drone position to screen coordinates
    let sx = drone.x * screenScale;
    let sy = SCREEN_HEIGHT - drone.y * screenScale;
    //drawDrone(sx, sy, drone.theta);
    drawDrone2(sx, sy, drone.theta);
    // Lap/waypoint logic
    let result = course.testWaypoint(sx, sy);
    if (result.isHit) {
        drone.x += result.jump.x / screenScale;
        drone.y += result.jump.y / screenScale;
        isRaceRunning = true;
    }
    if (result.isLapComplete) {
        if (lapTimer > 1) {
            completedLaps.push(lapTimer / 30.0);
        }
        course.startLap();
        lapTimer = 0;
    }
    drawTiming(lapTimer, completedLaps);
    updateDrone(1 / 30); // 30 FPS
    animationTimer = (animationTimer + 1) % 32;
    if (isRaceRunning) lapTimer++;
    requestAnimationFrame(gameLoop);
}

gameLoop();

