"""Server for the viz tool."""

import sys
from pathlib import Path

from flask import Flask, jsonify, send_from_directory

# Place the wrapper module on the python path:
REPO_ROOT = Path(__file__).parent.parent.absolute()
sys.path.insert(0, str(REPO_ROOT / "build" / "wrapper"))

from .plotting import compute_values

# Where static HTML files are found:
DIST_DIR = Path(__file__).parent.absolute() / "dist"

app = Flask(__name__)


@app.route("/plot")
def plot_api():
    return compute_values()


@app.route("/assets/<path:filename>")
def static_endpoint(filename):
    return send_from_directory(DIST_DIR, filename)


@app.route("/")
def index():
    return send_from_directory(DIST_DIR, "index.html")


if __name__ == "__main__":
    app.run(debug=True)
