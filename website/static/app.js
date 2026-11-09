const canvas = document.getElementById("drawing-canvas");
const context = canvas.getContext("2d");
const clearButton = document.getElementById("clear-drawing");
const sendButton = document.getElementById("send-drawing");
const strokeCount = document.getElementById("stroke-count");
const drawing = {
  width: canvas.width,
  height: canvas.height,
  strokes: []
};

let activePointerId = null;
let currentStroke = null;

canvas.style.touchAction = "none";
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
  strokeCount.value = drawing.strokes.length;

  context.beginPath();
  context.arc(point[0], point[1], 1, 0, Math.PI * 2);
  context.fill();
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
  strokeCount.value = drawing.strokes.length;
  context.clearRect(0, 0, canvas.width, canvas.height);
}

const output = document.getElementById("payload");

function drawingToSvg() {
  const lines = [
    `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${drawing.width} ${drawing.height}" fill="none" stroke="black" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">`
  ];

  for (const stroke of drawing.strokes) {
    if (stroke.length === 1) {
      const point = stroke[0];
      lines.push(`  <circle cx="${point[0]}" cy="${point[1]}" r="1" fill="black" stroke="none" />`);
      continue;
    }

    const points = [];
    for (const point of stroke) {
      points.push(`${point[0]},${point[1]}`);
    }
    lines.push(`  <polyline points="${points.join(" ")}" />`);
  }

  lines.push("</svg>");
  return lines.join("\n");
}

async function sendDrawing() {
  if (drawing.strokes.length === 0) {
    output.textContent = "Draw something before sending.";
    return;
  }

  sendButton.disabled = true;
  output.textContent = "Sending...";

  try {
    const svg = drawingToSvg();
    const response = await fetch(sendButton.dataset.url, {
      method: "POST",
      headers: {
        "Content-Type": "image/svg+xml"
      },
      body: svg
    });
    output.textContent = await response.text();
  } catch {
    output.textContent = "Could not send the payload.";
  } finally {
    sendButton.disabled = false;
  }
}

canvas.addEventListener("pointerdown", startDrawing);
canvas.addEventListener("pointermove", draw);
canvas.addEventListener("pointerup", stopDrawing);
canvas.addEventListener("pointercancel", stopDrawing);
canvas.addEventListener("lostpointercapture", stopDrawing);
clearButton.addEventListener("click", clearDrawing);
sendButton.addEventListener("click", sendDrawing);
