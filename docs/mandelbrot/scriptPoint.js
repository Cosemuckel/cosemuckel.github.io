const canvasPoint = document.getElementById('canvasPoint');
const ctxPoint = canvasPoint.getContext('2d');

const containerPoint = document.getElementById('containerPoint');
let point = {x: 0, y: 0};
let isDragging = false;

function resizeCanvasPoint() {
    canvasPoint.width = containerPoint.clientWidth;
    canvasPoint.height = containerPoint.clientHeight;

    point.x = canvasPoint.width / 2;
    point.y = canvasPoint.height / 2;

    drawGrid();
    drawMarkers();
    drawPoint();
}

function drawGrid() {
    const spacing = canvasPoint.width / 10;

    const originX = canvasPoint.width / 2;
    const originY = canvasPoint.height / 2;

    ctxPoint.clearRect(0, 0, canvasPoint.width, canvasPoint.height);

    ctxPoint.fillStyle = 'white';
    ctxPoint.fillRect(0, 0, canvasPoint.width, canvasPoint.height);

    ctxPoint.strokeStyle = '#ddd';
    ctxPoint.lineWidth = 1;

    for (let x = 0; x < canvasPoint.width; x += spacing) {
        ctxPoint.beginPath();
        ctxPoint.moveTo(x, 0);
        ctxPoint.lineTo(x, canvasPoint.height);
        ctxPoint.stroke();
    }

    for (let y = 0; y < canvasPoint.height; y += spacing) {
        ctxPoint.beginPath();
        ctxPoint.moveTo(0, y);
        ctxPoint.lineTo(canvasPoint.width, y);
        ctxPoint.stroke();
    }

    ctxPoint.strokeStyle = 'black';
    ctxPoint.lineWidth = 2;
    ctxPoint.beginPath();
    ctxPoint.moveTo(originX, 0);
    ctxPoint.lineTo(originX, canvasPoint.height);
    ctxPoint.stroke();

    ctxPoint.beginPath();
    ctxPoint.moveTo(0, originY);
    ctxPoint.lineTo(canvasPoint.width, originY);
    ctxPoint.stroke();
}

function drawMarkers() {
    const spacing = canvasPoint.width / 10;

    const originX = canvasPoint.width / 2;
    const originY = canvasPoint.height / 2;

    const oneX = originX + spacing * 2;
    const oneY = originY;

    ctxPoint.fillStyle = 'black';
    ctxPoint.beginPath();   
    ctxPoint.arc(oneX, oneY, 3, 0, 2 * Math.PI);
    ctxPoint.fill();

    const iX = originX;
    const iY = originY - spacing * 2;

    ctxPoint.beginPath();
    ctxPoint.arc(iX, iY, 3, 0, 2 * Math.PI);
    ctxPoint.fill();
}

function drawPoint() {
    ctxPoint.fillStyle = 'black';
    ctxPoint.beginPath();
    ctxPoint.arc(point.x, point.y, 5, 0, 2 * Math.PI);
    ctxPoint.fill();
}

function getMousePos(event) {
    const rect = canvasPoint.getBoundingClientRect();
    return {
        x: event.clientX - rect.left,
        y: event.clientY - rect.top
    };
}

function getTouchPos(event) {
    const rect = canvasPoint.getBoundingClientRect();
    return {
        x: event.touches[0].clientX - rect.left,
        y: event.touches[0].clientY - rect.top
    };
}

function getGridPos(pos) {
    const spacing = canvasPoint.width / 10;

    const originX = canvasPoint.width / 2;
    const originY = canvasPoint.height / 2;

    const dx = pos.x - originX;
    const dy = originY - pos.y;

    const real = dx / spacing / 2;
    const imag = dy / spacing / 2;

    return {real : real, imag : imag};
}

canvasPoint.addEventListener('mouseup', () => {
    isDragging = false;
});

canvasPoint.addEventListener('mousedown', (event) => {
    const pos = getMousePos(event);
    const distance = Math.hypot(pos.x - point.x, pos.y - point.y);
    if (distance < 5) {
        isDragging = true;
    }
});

canvasPoint.addEventListener('mousemove', (event) => {
    if (isDragging) {
        const pos = getMousePos(event);
        point = pos;
        drawGrid();
        drawMarkers();
        drawPoint();

        const gridPos = getGridPos(point);
        document.getElementById('coord').innerText = `Z0 = ${gridPos.real.toFixed(2)}, ${gridPos.imag.toFixed(2)}i`;

        debouncedDrawMandelbrot();
    }
});

canvasPoint.addEventListener('touchstart', (event) => {
    const pos = getTouchPos(event);
    const distance = Math.hypot(pos.x - point.x, pos.y - point.y);
    if (distance < 5) {
        isDragging = true;
    }
});

canvasPoint.addEventListener('touchmove', (event) => {
    if (isDragging) {
        const pos = getTouchPos(event);
        point = pos;
        drawGrid();
        drawMarkers();
        drawPoint();

        const gridPos = getGridPos(point);
        document.getElementById('coord').innerText = `Z0 = ${gridPos.real.toFixed(2)}, ${gridPos.imag.toFixed(2)}i`;

        debouncedDrawMandelbrot();
    }
});

canvasPoint.addEventListener('touchend', () => {
    isDragging = false;
});

function resetPoint() {
    point.x = canvasPoint.width / 2;
    point.y = canvasPoint.height / 2;
    drawGrid();
    drawMarkers();
    drawPoint();

    document.getElementById('coord').innerText = `Z0 = 0.00, 0.00i`;

    debouncedDrawMandelbrot();
}

let pointCanvasUP = false;
function togglePointCanvas(caller) {
    if (!pointCanvasUP) {
        document.getElementById("containerPoint").style.transform = "translateY(-12rem)";
        document.getElementById("options").style.transform = "translateY(-10.5rem)";

        caller.style.transform = "rotate(180deg)";
        pointCanvasUP = true;
    }
    else {
        document.getElementById("containerPoint").style.transform = "translateY(0)";
        document.getElementById("options").style.transform = "translateY(0)";

        caller.style.transform = "rotate(0deg)";
        pointCanvasUP = false;
    }
}

let julia = false;
function toggleJulia(caller) {
    julia = !julia;

    caller.style.transform = julia ? "rotate(180deg)" : "rotate(0deg)";

    debouncedDrawMandelbrot();
}

window.addEventListener('resize', resizeCanvasPoint);