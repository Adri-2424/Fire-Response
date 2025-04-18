from flask import Flask, render_template, request, jsonify
import pandas as pd
import numpy as np
from scipy.spatial import KDTree
import math
import os

app = Flask(__name__)
print("Templates directory:", os.path.join(app.root_path, "templates"))

try:
    DF = pd.read_csv(
        "cleaned_incident_data.csv",
        dtype={"Latitude": float, "Longitude": float}
    )
    DF.dropna(subset=["Latitude", "Longitude"], inplace=True)

    if DF.empty:
        raise ValueError("CSV file has no valid coordinates after cleaning!")

    # replace *all* NaN with None once, so every endpoint serialises clean JSON
    DF = DF.where(pd.notnull(DF), None)

    coords = list(zip(DF["Latitude"], DF["Longitude"]))
    tree = KDTree(coords)
    print(f"✅ KDTree built with {len(coords)} points.")

except Exception as e:
    print(f"❌ Initialization failed: {e}")
    DF = pd.DataFrame()
    tree = None

# ── routes ────────────────────────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/all_incidents")
def all_incidents():
    if DF.empty:
        return jsonify({"error": "Data not loaded"}), 500
    # random 1 000‑row sample keeps the first load light
    return jsonify(DF.sample(1000).to_dict(orient="records"))


@app.route("/query")
def query():
    try:
        lat = request.args.get("lat", type=float)
        lon = request.args.get("lon", type=float)
        radius_km = request.args.get("radius", type=float)

        if None in (lat, lon, radius_km):
            return jsonify({"error": "Missing lat/lon/radius"}), 400
        if tree is None or DF.empty:
            return jsonify({"error": "Data not initialised"}), 500

        radius_deg = radius_km / 111.0           # rough deg ↔ km
        dist, idx = tree.query((lat, lon))
        nearest = DF.iloc[idx].to_dict()
        nearest["distance_km"] = dist * 111.0 if math.isfinite(dist) else 0.0

        idxs = tree.query_ball_point((lat, lon), radius_deg)
        incidents = DF.iloc[idxs].to_dict(orient="records")

        return jsonify({"nearest": nearest, "incidents": incidents})

    except Exception as e:
        print(f"🔥 /query error: {e}")
        return jsonify({"error": str(e)}), 500


if __name__ == "__main__":
    app.run(debug=True)
