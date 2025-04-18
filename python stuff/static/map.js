
   L.Icon.Default.mergeOptions({
    iconRetinaUrl:
      "https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon-2x.png",
    iconUrl:
      "https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon.png",
    shadowUrl:
      "https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png"
  });
  
  
  const map = L.map("map").setView([29.7, -82.3], 10);
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png")
    .addTo(map);
  
  const allMarkers = L.markerClusterGroup().addTo(map);
  
 
  fetch("/all_incidents")
    .then(r => r.json())
    .then(data => {
      if (!Array.isArray(data)) throw new Error("Bad /all_incidents JSON");
  
      data.forEach(inc => {
        if (inc.Latitude && inc.Longitude) {
          L.marker([inc.Latitude, inc.Longitude]).addTo(allMarkers);
        }
      });
  
      if (allMarkers.getLayers().length) {
        map.fitBounds(allMarkers.getBounds(), { padding: [20, 20] });
      }
    })
    .catch(err => console.error("All‑incidents load failed:", err));
  
  
  function runQuery() {
    const lat    = +document.getElementById("lat").value;
    const lon    = +document.getElementById("lon").value;
    const radius = +document.getElementById("radius").value;
  
    fetch(`/query?lat=${lat}&lon=${lon}&radius=${radius}`)
      .then(r => r.json())
      .then(data => {
        if (data.error) return console.error("Query error:", data.error);
  
        // clear any previous query graphics
        if (window.radiusLayer) map.removeLayer(window.radiusLayer);
        window.radiusLayer = L.layerGroup().addTo(map);
  
        // draw query circle
        L.circle([lat, lon], {
          radius: radius * 1000,   // km → m
          color:  "red",
          fillOpacity: 0.1
        }).addTo(window.radiusLayer);
  
        // add result markers
        data.incidents.forEach(inc => {
          L.marker([inc.Latitude, inc.Longitude]).addTo(window.radiusLayer);
        });
      })
      .catch(err => console.error("Query failed:", err));
  }
  