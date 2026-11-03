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

## Files and routes

- `app.py`: Flask backend. `GET /` serves the page; `GET /api/health` returns JSON.
- `templates/index.html`: frontend page.
- `static/style.css`: page styles.
- `static/app.js`: calls the backend and displays its response.
- `requirements.txt`: Python dependency.

This starter does not connect to or move robot hardware.
