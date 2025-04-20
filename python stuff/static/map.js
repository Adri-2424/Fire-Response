/* global L */
L.Icon.Default.mergeOptions({
  iconRetinaUrl: "https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon-2x.png",
  iconUrl:      "https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon.png",
  shadowUrl:    "https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png"
});

const map = L.map("map").setView([29.7, -82.3], 10);
L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png")
  .addTo(map);

// all‑incidents layer (clustered) 
const allMarkers = L.markerClusterGroup().addTo(map);

fetch("/all_incidents")
  .then(r => r.json())
  .then(list => {
    list.forEach(inc => {
      if (inc.Latitude && inc.Longitude) {
        L.marker([inc.Latitude, inc.Longitude]).addTo(allMarkers);
      }
    });
    if (allMarkers.getLayers().length)
      map.fitBounds(allMarkers.getBounds(), { padding: [20, 20] });
  })
  .catch(err => console.error("All‑incidents load failed:", err));


// Lat/Lon / Radius query stuff 
function runQuery() {
  const lat    = +document.getElementById("lat").value;
  const lon    = +document.getElementById("lon").value;
  const radius = +document.getElementById("radius").value;

  fetch(`/query?lat=${lat}&lon=${lon}&radius=${radius}`)
    .then(r => r.json())
    .then(data => {
      if (data.error) return console.error("Query error:", data.error);

      // purge previous query graphics
      if (window.queryLayer) map.removeLayer(window.queryLayer);
      window.queryLayer = L.layerGroup().addTo(map);

      // draw radius circle around user point
      L.circle([lat, lon], {
        radius: radius * 1000,     // km → m
        color: "red",
        fillOpacity: 0.06
      }).addTo(window.queryLayer);

      // add each incident inside radius (small grey dot)
      data.incidents.forEach(inc => {
        L.circleMarker([inc.Latitude, inc.Longitude], {
          radius: 4, weight: 0, color: "#444", fillOpacity: 0.6
        }).addTo(window.queryLayer)
          .bindPopup(`Incident ID: ${inc.IncidentNumber || "?"}`);
      });

      // nearest incident (blue)
      const ni = data.nearest;
      L.circleMarker([ni.Latitude, ni.Longitude], {
        radius: 8, color: "blue", fillColor: "blue", fillOpacity: 0.9
      }).addTo(window.queryLayer)
        .bindPopup(
          `<b>Nearest incident</b><br>ID: ${ni.IncidentNumber || "?"}<br>` +
          `Distance: ${ni.distance_km.toFixed(2)} km`
        ).openPopup();

      // nearest responding unit (green)
      const nu = data.nearest_unit;
      L.circleMarker([nu.lat, nu.lon], {
        radius: 8, color: "green", fillColor: "green", fillOpacity: 0.9
      }).addTo(window.queryLayer)
        .bindPopup(
          `<b>Likely responding unit: ${nu.unit}</b><br>` +
          `Distance to incident: ${nu.distance_km.toFixed(2)} km`
        );

      // update Info panel
      document.getElementById("info").textContent = JSON.stringify({
        nearest_incident: {
          id: ni.IncidentNumber || "?",
          distance_km: ni.distance_km
        },
        nearest_unit: nu
      }, null, 2);

      // adjust map view
      const group = L.featureGroup(window.queryLayer.getLayers());
      map.fitBounds(group.getBounds().pad(0.4));
    })
    .catch(err => console.error("Query failed:", err));
}
