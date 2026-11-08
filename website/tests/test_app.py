import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from app import app  # noqa: E402


def test_svg_is_returned_as_plain_source():
    svg = b'<svg xmlns="http://www.w3.org/2000/svg"><polyline points="0,0 10,20"/></svg>'
    response = app.test_client().post("/api/drawing", data=svg, content_type="image/svg+xml")
    assert response.status_code == 200
    assert response.mimetype == "text/plain"
    assert response.data == svg


def test_other_content_types_are_rejected():
    response = app.test_client().post("/api/drawing", json={"strokes": []})
    assert response.status_code == 415


def test_empty_svg_is_rejected():
    response = app.test_client().post("/api/drawing", data=b" ", content_type="image/svg+xml")
    assert response.status_code == 400
