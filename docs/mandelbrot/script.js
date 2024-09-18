const workerCode = `
const MAX_ITERATIONS = 200;
const ESCAPE_RADIUS = 4;

function interpolate(t, colors) {
    let c1, c2;
    for (let i = 0; i < colors.length - 1; i++) {
        if (t >= colors[i].position && t <= colors[i + 1].position) {
            c1 = colors[i];
            c2 = colors[i + 1];
            break;
        }
    }

    const dt = (t - c1.position) / (c2.position - c1.position);
    return [
        c1.color[0] + (c2.color[0] - c1.color[0]) * dt,
        c1.color[1] + (c2.color[1] - c1.color[1]) * dt,
        c1.color[2] + (c2.color[2] - c1.color[2]) * dt
    ];
}

function calculateMandelbrot(data) {
    const { width, height, topLeft, bottomRight, chunkStartX, chunkStartY, chunkSize, colors } = data;
    const buffer = new Uint8ClampedArray(chunkSize * chunkSize * 4);

    for (let y = 0; y < chunkSize; y++) {
        for (let x = 0; x < chunkSize; x++) {
            const cx = topLeft.x + ((chunkStartX + x) / width) * (bottomRight.x - topLeft.x);
            const cy = topLeft.y + ((chunkStartY + y) / height) * (bottomRight.y - topLeft.y);

            let zx = 0, zy = 0, i = 0;

            while (i < MAX_ITERATIONS && zx * zx + zy * zy < ESCAPE_RADIUS) {
                const xtemp = zx * zx - zy * zy + cx;
                zy = 2 * zx * zy + cy;
                zx = xtemp;
                ++i;
            }

            const t = i / MAX_ITERATIONS;
            const [r, g, b] = interpolate(t, colors);
            const offset = (y * chunkSize + x) * 4;
            buffer[offset] = r;
            buffer[offset + 1] = g;
            buffer[offset + 2] = b;
            buffer[offset + 3] = 255;
        }
    }

    return buffer;
}

self.onmessage = function(e) {
    const result = calculateMandelbrot(e.data);
    self.postMessage({
        result,
        chunkStartX: e.data.chunkStartX,
        chunkStartY: e.data.chunkStartY,
        renderID: e.data.renderID
    }, [result.buffer]);
};
`;

const themes = [
    [
    { position: 0.0, color: [0, 7, 100] },
    { position: 0.16, color: [32, 107, 203] },
    { position: 0.42, color: [237, 255, 255] },
    { position: 0.6425, color: [255, 170, 0] },
    { position: 0.8575, color: [0, 2, 0] },
    { position: 0.95, color: [0, 7, 100] },
    { position: 1.0, color: [0, 0, 0] }],
    [
    { position: 0.0, color: [0, 0, 0] },
    { position: 0.3, color: [255, 255, 255] },
    { position: 1.0, color: [0, 0, 0] }],
    [
    { position: 0.0, color: [255, 255, 255] },
    { position: 0.3, color: [0, 0, 0] },
    { position: 0.7, color: [255, 255, 255] },
    { position: 1.0, color: [0, 0, 0] }],
];

let currentTheme = 0;

const workerBlob = new Blob([workerCode], { type: 'application/javascript' });
const workerUrl = URL.createObjectURL(workerBlob);

const CHUNK_SIZE = 128;
const MAX_WORKERS = 1; // Single worker

let topLeft = { x: -2, y: 2 };
let bottomRight = { x: 2, y: -2 };
let canvas, ctx, width, height;
let panning = false;
let lastMouse = { x: 0, y: 0 };
let zoomCenter = { x: 0, y: 0 };
let zoomFactor = 1;
let renderID = 0;
let renderTimeout;
let worker;
let activeWorkers = 0;
let renderedChunks = new Set();
let renderQueue = [];
let aspectRatio;

window.addEventListener('load', () => {
    canvas = document.getElementById('fullScreenCanvas');
    ctx = canvas.getContext('2d');

    // Create a single worker
    worker = new Worker(workerUrl);
    worker.onmessage = handleWorkerMessage;

    width = canvas.width;
    height = canvas.height;

    aspectRatio = width / height;
    topLeft = { x: -2 * aspectRatio, y: 2 };
    bottomRight = { x: 2 * aspectRatio, y: -2 };

    resizeCanvas();
    setupEventListeners();
});

function resizeCanvas() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    width = canvas.width;
    height = canvas.height;

    const aspectRatioOld = aspectRatio;
    aspectRatio = width / height;

    topLeft.x *= aspectRatio / aspectRatioOld;
    bottomRight.x *= aspectRatio / aspectRatioOld;

    renderedChunks.clear();
    drawMandelbrot();
}

function drawMandelbrot() {
    renderID++;
    const currentRenderID = renderID;

    for (let y = 0; y < Math.ceil(height / CHUNK_SIZE); y++) {
        for (let x = 0; x < Math.ceil(width / CHUNK_SIZE); x++) {
            addToRenderQueue(x, y, currentRenderID);
        }
    }

    processRenderQueue();
}

function addToRenderQueue(chunkX, chunkY, currentRenderID) {
    const chunkKey = `${chunkX},${chunkY}`;
    if (!renderedChunks.has(chunkKey)) {
        renderQueue.push({ chunkX, chunkY, currentRenderID });
    }
}

function processRenderQueue() {
    while (activeWorkers < MAX_WORKERS && renderQueue.length > 0) {
        const { chunkX, chunkY, currentRenderID } = renderQueue.shift();
        activeWorkers++;

        worker.postMessage({
            width,
            height,
            topLeft,
            bottomRight,
            chunkStartX: chunkX * CHUNK_SIZE,
            chunkStartY: chunkY * CHUNK_SIZE,
            chunkSize: CHUNK_SIZE,
            renderID: currentRenderID,
            colors: themes[currentTheme]
        });
    }
}

function handleWorkerMessage(e) {
    const { result, chunkStartX, chunkStartY, renderID: messageRenderID } = e.data;
    if (messageRenderID === renderID) {
        const imageData = new ImageData(result, CHUNK_SIZE, CHUNK_SIZE);
        ctx.putImageData(imageData, chunkStartX, chunkStartY);
        renderedChunks.add(`${Math.floor(chunkStartX / CHUNK_SIZE)},${Math.floor(chunkStartY / CHUNK_SIZE)}`);
    }

    activeWorkers--;
    processRenderQueue();
}

function panView(dx, dy) {
    const tempCanvas = document.createElement('canvas');
    tempCanvas.width = width;
    tempCanvas.height = height;
    const tempCtx = tempCanvas.getContext('2d');
    tempCtx.drawImage(canvas, 0, 0);

    ctx.clearRect(0, 0, width, height);
    ctx.drawImage(tempCanvas, dx, dy);

    const newRenderedChunks = new Set();
    const chunksToDraw = new Set();
    const chunkDX = Math.floor(dx / CHUNK_SIZE);
    const chunkDY = Math.floor(dy / CHUNK_SIZE);

    renderedChunks.forEach(chunk => {
        const [x, y] = chunk.split(',').map(Number);
        const newX = x + chunkDX;
        const newY = y + chunkDY;
        if (isChunkVisible(newX, newY)) {
            newRenderedChunks.add(`${newX},${newY}`);
        }
    });

    for (let y = 0; y < Math.ceil(height / CHUNK_SIZE); y++) {
        for (let x = 0; x < Math.ceil(width / CHUNK_SIZE); x++) {
            if (!newRenderedChunks.has(`${x},${y}`)) {
                chunksToDraw.add(`${x},${y}`);
            }
        }
    }

    renderedChunks = newRenderedChunks;

    // We do not initiate new rendering here.
}

function isChunkVisible(chunkX, chunkY) {
    return chunkX >= 0 && chunkX < Math.ceil(width / CHUNK_SIZE) &&
           chunkY >= 0 && chunkY < Math.ceil(height / CHUNK_SIZE);
}

function debouncedDrawMandelbrot() {
    clearTimeout(renderTimeout);
    renderTimeout = setTimeout(() => {
        renderedChunks.clear();
        drawMandelbrot();
    }, 200);
}

function setupEventListeners() {
    window.addEventListener('resize', resizeCanvas);

    canvas.addEventListener('mousedown', event => {
        if (event.button === 0) {
            panning = true;
            lastMouse = { x: event.clientX, y: event.clientY };
        }
    });

    canvas.addEventListener('mousemove', event => {
        if (panning) {
            const dx = event.clientX - lastMouse.x;
            const dy = event.clientY - lastMouse.y;

            panView(dx, dy);

            const worldDx = (dx / width) * (bottomRight.x - topLeft.x);
            const worldDy = (dy / height) * (bottomRight.y - topLeft.y);
            topLeft.x -= worldDx;
            topLeft.y -= worldDy;
            bottomRight.x -= worldDx;
            bottomRight.y -= worldDy;

            lastMouse = { x: event.clientX, y: event.clientY };
        }
    });

    canvas.addEventListener('mouseup', event => {
        if (event.button === 0) {
            panning = false;
            debouncedDrawMandelbrot();
        }
    });

    canvas.addEventListener('wheel', event => {
        event.preventDefault();
        const mouseX = event.clientX;
        const mouseY = event.clientY;

        zoomCenter = { x: mouseX, y: mouseY };

        const zoomIn = event.deltaY < 0;
        const factor = zoomIn ? 0.9 : 1.1;
        zoomFactor *= factor;

        ctx.save();
        ctx.translate(zoomCenter.x, zoomCenter.y);
        ctx.scale(1 / factor, 1 / factor);
        ctx.translate(-zoomCenter.x, -zoomCenter.y);
        ctx.drawImage(canvas, 0, 0);
        ctx.restore();

        const mouseXWorld = topLeft.x + (mouseX / width) * (bottomRight.x - topLeft.x);
        const mouseYWorld = topLeft.y + (mouseY / height) * (bottomRight.y - topLeft.y);

        topLeft.x = mouseXWorld + (topLeft.x - mouseXWorld) * factor;
        topLeft.y = mouseYWorld + (topLeft.y - mouseYWorld) * factor;
        bottomRight.x = mouseXWorld + (bottomRight.x - mouseXWorld) * factor;
        bottomRight.y = mouseYWorld + (bottomRight.y - mouseYWorld) * factor;

        debouncedDrawMandelbrot();
    });
}

function home() {
    topLeft = { x: -2 * aspectRatio, y: 2 };
    bottomRight = { x: 2 * aspectRatio, y: -2 };
    zoomFactor = 1;
    
    renderedChunks.clear();

    debouncedDrawMandelbrot();
}

function theme() {
    currentTheme = (currentTheme + 1) % themes.length;
    renderedChunks.clear();
    drawMandelbrot();
}

function download() {
    const link = document.createElement('a');
    link.download = 'mandelbrot.png';
    link.href = canvas.toDataURL();
    link.click();
}