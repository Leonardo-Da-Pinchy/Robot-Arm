# Flask website

Minimal Flask backend with an HTML, CSS, and JavaScript frontend. Flask serves
both from the same app; no frontend build step is needed.

## Run locally

Requires Python 3.9 or newer. From the repository root:

```powershell
cd website
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m flask --app app run --debug
```

On macOS/Linux, use `.venv/bin/python` in place of
`.\.venv\Scripts\python.exe` for the install and run commands.

Open http://127.0.0.1:5000 and click **Check backend connection**.
Use Ctrl+C to stop the server. Debug mode is for local development only.

Draw on the canvas with a mouse, finger, or pen. **Clear drawing** clears both
the canvas and recorded points. The JavaScript `drawing` object stores the
600 by 400 canvas dimensions and a list of strokes. Each stroke contains
`[x, y]` points, with the origin at the top-left corner. Drawings stay in browser
memory and are cleared when the page reloads.

## Files and routes

- `app.py`: Flask backend. `GET /` serves the page; `GET /api/health` returns JSON.
- `templates/index.html`: frontend page.
- `static/style.css`: page styles.
- `static/app.js`: handles drawing, records stroke points, and checks the backend.
- `requirements.txt`: Python dependency.

This starter does not connect to or move robot hardware.
