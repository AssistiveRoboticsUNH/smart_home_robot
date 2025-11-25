from flask import Flask, render_template, jsonify, request
import json, os, threading
import pathlib

app = Flask(__name__)

from ament_index_python.packages import get_package_share_directory
pkg_path = pathlib.Path(get_package_share_directory('shr_display'))
DATA_PATH =  pkg_path / 'config' / 'protocol_routines.json'

# DATA_PATH = os.path.join(os.path.dirname(__file__), "protocol_routines.json")
_lock = threading.Lock()

def _load_data():
    with _lock:
        if not os.path.exists(DATA_PATH):
            return []
        with open(DATA_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        # ensure 'confirmed' exists for older files
        changed = False
        for item in data:
            if "confirmed" not in item:
                item["confirmed"] = False
                changed = True
        if changed:
            _save_data(data)
        return data

def _save_data(data):
    with _lock:
        with open(DATA_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

@app.route("/")
def index():
    return render_template("protocols.html")

@app.route("/api/protocols", methods=["GET"])
def get_protocols():
    return jsonify(_load_data())

@app.route("/api/confirm", methods=["POST"])
def set_confirmed():
    """
    Body:
    {
      "index": <int>,            # row index in the JSON list
      "confirmed": <bool>        # true/false
    }
    """
    body = request.get_json(force=True, silent=True) or {}
    idx = body.get("index")
    val = body.get("confirmed")

    data = _load_data()
    if not isinstance(idx, int) or idx < 0 or idx >= len(data):
        return jsonify({"error": "Invalid index"}), 400
    if not isinstance(val, bool):
        return jsonify({"error": "confirmed must be boolean"}), 400

    data[idx]["confirmed"] = val
    _save_data(data)

    # TODO (optional): notify robot here if you want a side-effect (e.g., ZMQ/ROS2/HTTP)
    # send_to_robot({"type": "protocol_confirmed", "index": idx, "confirmed": val})

    return jsonify({"ok": True, "item": data[idx]})

@app.route("/api/provided", methods=["POST"])
def set_provided():
    """
    Optional: lets your robot update 'provided' (Reminded) status.
    Body:
    {
      "index": <int>,
      "provided": <bool>
    }
    """
    body = request.get_json(force=True, silent=True) or {}
    idx = body.get("index")
    val = body.get("provided")

    data = _load_data()
    if not isinstance(idx, int) or idx < 0 or idx >= len(data):
        return jsonify({"error": "Invalid index"}), 400
    if not isinstance(val, bool):
        return jsonify({"error": "provided must be boolean"}), 400

    data[idx]["provided"] = val
    _save_data(data)
    return jsonify({"ok": True, "item": data[idx]})

if __name__ == "__main__":
    # Run as: python app.py  (serves on http://localhost:5000)
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
