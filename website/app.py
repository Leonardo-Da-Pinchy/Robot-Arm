from flask import Flask, Response, render_template, request

app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = 1024 * 1024


@app.get("/")
def index():
    return render_template("index.html")


@app.post("/api/drawing")
def receive_payload():
    if request.mimetype != "image/svg+xml":
        return Response("Send SVG source.", status=415, mimetype="text/plain")

    svg = request.get_data()
    if not svg.strip():
        return Response("The drawing is empty.", status=400, mimetype="text/plain")

    return Response(svg, mimetype="text/plain")


@app.errorhandler(413)
def too_large(_error):
    return Response("The payload exceeds the 1 MB limit.", status=413, mimetype="text/plain")
