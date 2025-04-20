from flask import Flask, render_template, request, jsonify
import pandas as pd, math, os, json
from scipy.spatial import KDTree

#configuration 
CSV_FILE   = "cleaned_incident_data.csv"
SAMPLE_MAX = 1_000           # how many rows to return from /all_incidents
DEG_PER_KM = 1.0 / 111.0     # ≈ degrees of lat / lon per kilometre

#Same fire‑unit coordinates used in main.cpp
UNIT_COORDS = {
    "E7":  (29.700735771993592, -82.38651442738922),
    "SQ3": (29.665023513467546, -82.29958933643647),
    "E3":  (29.66395757157737,  -82.30181092608116),
    "E2":  (29.629974283595164, -82.35531600790573),
    "Q9":  (29.62361312432365,  -82.3706184157027),
    "Q8":  (29.69829338320167,  -82.37038952707218),
    "TW1": (29.6558004642621,   -82.32716385331416),
    "E5":  (29.68089304705855,  -82.3350131946103),
    "E1":  (29.6535648986405,   -82.32892497332307),
    "E4":  (29.65677821533309,  -82.38085479231843),
    "SQ1": (29.65274296122673,  -82.3274143683069),
    "SQ2": (29.634307529137704, -82.37794834140252),
    "HZ2": (29.667737620635176, -82.34663823759034),
    "Q2":  (29.634968679459902, -82.35666260626792),
    "TW2": (29.633897140286848, -82.35655893525745),
    "CR6-1": (29.68157895,      -82.276542475),
    "L9":  (29.628166453149003, -82.38419263932411),
    "Q1":  (29.65610122258656,  -82.32648061217381),
    "TR8": (29.69629839603961,  -82.36386959405941),
    "DC1": (29.65152004373069,  -82.33963253032567),
    "CRP1": (29.664385421875,   -82.33400179340278),
    "DC2": (29.690520165011428, -82.39215404813429),
    "TR9": (29.6273168984375,   -82.3834842890625),
    "CRP2": (29.66423636,       -82.33255536),
    "TR2": (29.62992382669149,  -82.3576894853617),
    "MRU4": (29.652504392,      -82.34277542),
    "CR6-3": (29.678154608695653, -82.28699813043478),
    "DC3":  (29.6548518,        -82.35892079999999),
    "SQ9":  (29.623905466154763, -82.37939707866005),
    "TR1":  (29.656132808510645, -82.32722745212766),
    "E9":   (29.654222857142862, -82.34809002380952),
    "SQ4":  (29.655778666666667, -82.32370466666667),
    "CR61": (29.679144866666668, -82.27718196666666),
    "HZ1":  (29.67977952801816,  -82.33657692804908),
}

#helpers yippeee
def haversine_km(lat1, lon1, lat2, lon2):
    """Great‑circle distance (km)."""
    R = 6371.0
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (
        math.sin(d_lat/2)**2
        + math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * math.sin(d_lon/2)**2
    )
    return 2 * R * math.asin(math.sqrt(a))


def nearest_unit_for(lat, lon):
    best = {"unit": None, "lat": None, "lon": None, "distance_km": float("inf")}
    for unit, (u_lat, u_lon) in UNIT_COORDS.items():
        d = haversine_km(lat, lon, u_lat, u_lon)
        if d < best["distance_km"]:
            best.update({"unit": unit, "lat": u_lat, "lon": u_lon, "distance_km": round(d, 3)})
    return best


# Flask app
app = Flask(__name__)
print("Templates dir:", os.path.join(app.root_path, "templates"))

try:
    DF = pd.read_csv(CSV_FILE, dtype={"Latitude": float, "Longitude": float})
    DF.dropna(subset=["Latitude", "Longitude"], inplace=True)
    DF = DF.where(pd.notnull(DF), None)          # NaN → None for clean JSON
    tree = KDTree(list(zip(DF["Latitude"], DF["Longitude"])))
    print(f"✅ KDTree built with {len(DF)} incidents.")

except Exception as e:
    print("❌ CSV load / KDTree build failed:", e)
    DF, tree = pd.DataFrame(), None


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/all_incidents")
def all_incidents():
    if DF.empty:
        return jsonify({"error": "incident data not loaded"}), 500
    sample = DF if len(DF) <= SAMPLE_MAX else DF.sample(SAMPLE_MAX)
    return jsonify(sample.to_dict(orient="records"))


@app.route("/query")
def query():
    if DF.empty or tree is None:
        return jsonify({"error": "data unavailable"}), 500

    lat = request.args.get("lat", type=float)
    lon = request.args.get("lon", type=float)
    radius_km = request.args.get("radius", type=float, default=5)

    if None in (lat, lon):
        return jsonify({"error": "missing lat / lon"}), 400

    # nearest single incident 
    dist_deg, idx = tree.query((lat, lon))
    nearest = DF.iloc[idx].to_dict()
    nearest_dist_km = dist_deg / DEG_PER_KM
    nearest["distance_km"] = round(nearest_dist_km, 3)

    # incidents within radius 
    idxs = tree.query_ball_point((lat, lon), radius_km * DEG_PER_KM)
    incidents = DF.iloc[idxs].to_dict(orient="records")

    # nearest unit 
    unit_info = nearest_unit_for(nearest["Latitude"], nearest["Longitude"])

    return jsonify({
        "nearest": nearest,
        "nearest_unit": unit_info,
        "incidents": incidents
    })


if __name__ == "__main__":
    app.run(debug=True)
