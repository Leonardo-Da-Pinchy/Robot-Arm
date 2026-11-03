const button = document.querySelector("#check-backend");
const status = document.querySelector("#status");

async function checkBackend() {
  try {
    const response = await fetch(button.dataset.url);
    if (!response.ok) {
      throw new Error("Request failed");
    }
    const data = await response.json();
    status.textContent = data.message;
  } catch {
    status.textContent = "Could not connect to the backend.";
  }
}

button.addEventListener("click", checkBackend);

const canvas = document.getElementById("drawing-canvas");
const context = canvas.getContext("2d");
const clearButton = document.getElementById("clear-drawing");
const drawingStatus = document.getElementById("drawing-status");
const drawing = {
  width: canvas.width,
  height: canvas.height,
  strokes: []
};

let activePointerId = null;
let currentStroke = null;

context.lineWidth = 2;
context.lineCap = "round";
context.lineJoin = "round";

function getPoint(event) {
  const rectangle = canvas.getBoundingClientRect();
  const x = (event.clientX - rectangle.left) * canvas.width / rectangle.width;
  const y = (event.clientY - rectangle.top) * canvas.height / rectangle.height;

  return [
    Math.max(0, Math.min(canvas.width, x)),
    Math.max(0, Math.min(canvas.height, y))
  ];
}

function startDrawing(event) {
  if (activePointerId !== null || event.button !== 0) {
    return;
  }

  activePointerId = event.pointerId;
  canvas.setPointerCapture(event.pointerId);
  const point = getPoint(event);
  currentStroke = [point];
  drawing.strokes.push(currentStroke);

  context.beginPath();
  context.arc(point[0], point[1], 1, 0, Math.PI * 2);
  context.fill();
  drawingStatus.textContent = "Strokes: " + drawing.strokes.length;
}

function draw(event) {
  if (event.pointerId !== activePointerId) {
    return;
  }

  const point = getPoint(event);
  const previousPoint = currentStroke[currentStroke.length - 1];
  if (point[0] === previousPoint[0] && point[1] === previousPoint[1]) {
    return;
  }

  context.beginPath();
  context.moveTo(previousPoint[0], previousPoint[1]);
  context.lineTo(point[0], point[1]);
  context.stroke();
  currentStroke.push(point);
}

function stopDrawing(event) {
  if (event.pointerId !== activePointerId) {
    return;
  }

  if (event.type === "pointerup") {
    draw(event);
  }
  activePointerId = null;
  currentStroke = null;
}

function clearDrawing() {
  if (activePointerId !== null && canvas.hasPointerCapture(activePointerId)) {
    canvas.releasePointerCapture(activePointerId);
  }
  activePointerId = null;
  currentStroke = null;
  drawing.strokes = [];
  context.clearRect(0, 0, canvas.width, canvas.height);
  drawingStatus.textContent = "0 strokes";
}

canvas.addEventListener("pointerdown", startDrawing);
canvas.addEventListener("pointermove", draw);
canvas.addEventListener("pointerup", stopDrawing);
canvas.addEventListener("pointercancel", stopDrawing);
canvas.addEventListener("lostpointercapture", stopDrawing);
clearButton.addEventListener("click", clearDrawing);
