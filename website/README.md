# Flask payload viewer

Draw on the canvas and click **Send drawing**. The browser converts each
stroke into an SVG polyline and each single tap into a circle. Flask returns
the SVG source, and the page displays it as plain text.

The browser posts the SVG source directly to `/api/drawing` with content type
`image/svg+xml`. Requests are limited to 1 MB.
