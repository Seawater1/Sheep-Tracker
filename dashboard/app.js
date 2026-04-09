const map = L.map("map", {
  zoomControl: true
}).setView([51.8985, -8.4756], 7);

L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
  maxZoom: 19,
  attribution: "&copy; OpenStreetMap contributors"
}).addTo(map);

const trackLine = L.polyline([], {
  color: "#2d6a4f",
  weight: 4,
  opacity: 0.9
}).addTo(map);

let trackerMarker = null;
const history = [];
let lastRenderedKey = null;

function formatLatLon(value) {
  return (value / 1e7).toFixed(6);
}

function formatTimestamp(isoString) {
  const date = new Date(isoString);
  if (Number.isNaN(date.getTime())) {
    return "-";
  }
  return date.toLocaleString();
}

function readDecodedPayload(event) {
  return event?.uplink_message?.decoded_payload ?? null;
}

function pointFromEvent(event) {
  const payload = readDecodedPayload(event);
  if (!payload) {
    throw new Error("TTN event is missing uplink_message.decoded_payload");
  }

  const lat = payload.lat_e7;
  const lon = payload.lon_e7;
  if (!lat || !lon) {
    throw new Error("Decoded payload does not contain a valid GPS fix yet");
  }

  return {
    deviceId: event?.end_device_ids?.device_id ?? "unknown-device",
    devEui: event?.end_device_ids?.dev_eui ?? "-",
    animalName: event?.end_device_ids?.device_id ?? "unnamed tracker",
    battMv: payload.batt_mv ?? null,
    solarMv: payload.solar_mv ?? null,
    altitude: payload.alt_m ?? null,
    gpsUnix: payload.gps_unix ?? null,
    receivedAt: event?.received_at ?? new Date().toISOString(),
    latitude: lat / 1e7,
    longitude: lon / 1e7
  };
}

function updateSummary(point) {
  document.getElementById("animal-name").textContent = point.animalName;
  document.getElementById("device-id").textContent = `${point.deviceId} / ${point.devEui}`;
  document.getElementById("battery").textContent = point.battMv ? `${point.battMv} mV` : "-";
  document.getElementById("solar").textContent = point.solarMv ? `${point.solarMv} mV` : "-";
  document.getElementById("altitude").textContent = point.altitude !== null ? `${point.altitude} m` : "-";
  document.getElementById("updated").textContent = formatTimestamp(point.receivedAt);
}

function updateMap(point) {
  const key = `${point.deviceId}:${point.receivedAt}`;
  if (key !== lastRenderedKey) {
    history.push([point.latitude, point.longitude]);
    lastRenderedKey = key;
  }
  trackLine.setLatLngs(history);

  const popupHtml = `
    <div class="tracker-popup">
      <strong>${point.animalName}</strong><br>
      Battery: ${point.battMv ?? "-"} mV<br>
      Solar: ${point.solarMv ?? "-"} mV<br>
      Lat: ${formatLatLon(point.latitude * 1e7)}<br>
      Lon: ${formatLatLon(point.longitude * 1e7)}
    </div>
  `;

  if (!trackerMarker) {
    trackerMarker = L.marker([point.latitude, point.longitude]).addTo(map);
  } else {
    trackerMarker.setLatLng([point.latitude, point.longitude]);
  }

  trackerMarker.bindPopup(popupHtml);
  map.fitBounds(trackLine.getBounds(), {
    padding: [40, 40],
    maxZoom: 16
  });
}

function setMessage(text, isError = false) {
  const message = document.getElementById("input-message");
  message.textContent = text;
  message.style.color = isError ? "#a64040" : "";
}

function loadEvent(event) {
  const point = pointFromEvent(event);
  updateSummary(point);
  if (point.latitude && point.longitude) {
    updateMap(point);
    document.getElementById("tracker-status").textContent = "Tracking Live Test Data";
    setMessage(`Showing ${point.deviceId} at ${formatTimestamp(point.receivedAt)}.`);
  } else {
    document.getElementById("tracker-status").textContent = "Latest Uplink Has No GPS Fix";
    setMessage(`Latest uplink for ${point.deviceId} has no GPS fix yet.`);
  }
}

function loadPoint(point) {
  if (!point) {
    throw new Error("Latest tracker point is missing");
  }
  updateSummary(point);
  if (point.hasFix && point.latitude !== null && point.longitude !== null) {
    updateMap(point);
    document.getElementById("tracker-status").textContent = "Tracking Live Feed";
    setMessage(`Live update for ${point.deviceId} at ${formatTimestamp(point.receivedAt)}.`);
  } else {
    document.getElementById("tracker-status").textContent = "Live Feed Without GPS Fix";
    setMessage(`Latest uplink for ${point.deviceId} arrived at ${formatTimestamp(point.receivedAt)} but has no GPS fix yet.`);
  }
}

async function pollLatest() {
  try {
    const response = await fetch("/api/latest", {
      headers: {
        Accept: "application/json"
      }
    });
    if (!response.ok) {
      return;
    }
    const payload = await response.json();
    if (payload?.point) {
      loadPoint(payload.point);
    }
  } catch (_error) {
    // Local static testing without the backend should stay usable.
  }
}

document.getElementById("load-sample").addEventListener("click", () => {
  document.getElementById("ttn-input").value = JSON.stringify(window.sampleTtnEvent, null, 2);
  loadEvent(window.sampleTtnEvent);
});

document.getElementById("apply-json").addEventListener("click", () => {
  const text = document.getElementById("ttn-input").value.trim();
  if (!text) {
    setMessage("Paste a TTN webhook JSON event first.", true);
    return;
  }

  try {
    const parsed = JSON.parse(text);
    loadEvent(parsed);
  } catch (error) {
    setMessage(error.message || "Could not parse TTN JSON.", true);
  }
});

document.getElementById("ttn-input").value = JSON.stringify(window.sampleTtnEvent, null, 2);
loadEvent(window.sampleTtnEvent);
pollLatest();
window.setInterval(pollLatest, 10000);
