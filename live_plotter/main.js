// --- Cesium Viewer ---
const viewer = new Cesium.Viewer("cesiumContainer", {
    sceneMode: Cesium.SceneMode.COLUMBUS_VIEW,
    baseLayerPicker: false,
    timeline: false,
    animation: false,
    terrainProvider: new Cesium.EllipsoidTerrainProvider(),

    // imageryProvider: new Cesium.UrlTemplateImageryProvider({
    //     url: "./tiles/{z}/{x}/{y}.png",
    //     maximumLevel: 15,
    //     tilingScheme: new Cesium.WebMercatorTilingScheme()
    // })
});

// --- Default Camera View ---
viewer.camera.setView({
    destination: Cesium.Cartesian3.fromDegrees(-87.51416, 40.388527, 10000), // launch site
    orientation: {
        heading: Cesium.Math.toRadians(0),
        pitch: Cesium.Math.toRadians(-45),
        roll: 0
    }
});

// --- DOM  Elements ---
const headingSlider = document.getElementById("headingSlider");
const pitchSlider = document.getElementById("pitchSlider");
const controlsDiv = document.getElementById("controls");
let trackedStage = null;

// --- Sliders ---
headingSlider.addEventListener("input", () => {
    if (trackedStage) return;   // ignore sliders while tracking because Cesium overwrites
    const heading = Cesium.Math.toRadians(Number(headingSlider.value));
    const pitch = Cesium.Math.toRadians(Number(pitchSlider.value));
    viewer.camera.setView({
        destination: viewer.camera.positionWC,
        orientation: {
            heading: heading,
            pitch: pitch,
            roll: 0
        }
    });
    document.getElementById("headingValue").textContent = headingSlider.value;
    document.getElementById("pitchValue").textContent = pitchSlider.value;
});
pitchSlider.addEventListener("input", () => {
    if (trackedStage) return;
    const heading = Cesium.Math.toRadians(Number(headingSlider.value));
    const pitch = Cesium.Math.toRadians(Number(pitchSlider.value));
    viewer.camera.setView({
        destination: viewer.camera.positionWC,
        orientation: {
            heading: heading,
            pitch: pitch,
            roll: 0
        }
    });
    document.getElementById("headingValue").textContent = headingSlider.value;
    document.getElementById("pitchValue").textContent = pitchSlider.value;
});

// --- Tracking Buttons ---
document.querySelectorAll("#trackButtons button").forEach(btn => {
    btn.addEventListener("click", () => {
        trackedStage = stages[btn.dataset.stage] || null;
        viewer.trackedEntity = trackedStage ? trackedStage.entity : null;
        controlsDiv.style.display = "none";
    });
});

// --- Stop Tracking ---
document.getElementById("stopTracking").addEventListener("click", () => {
    trackedStage = null;
    viewer.trackedEntity = null;
    controlsDiv.style.display = "block";
});

// --- Stage Configurations ---
const STAGE_CONFIG = [
    { name: "Booster",      color: Cesium.Color.FIREBRICK },
    { name: "Sustainer",    color: Cesium.Color.BLUE },
    // Add more if needed
];

const stages = {};
const TRAIL_UPDATE_MS = 250; // throttle trail updates

STAGE_CONFIG.forEach((config, index) => {
    // Stage entity
    const entity = viewer.entities.add({
        position: Cesium.Cartesian3.fromDegrees(0, 0, 0),
        point: {
            pixelSize: 10,
            color: config.color
        },
        label: {
            text: config.name,
            font: "14px monospace",
            pixelOffset: new Cesium.Cartesian2(0, -20)
        }
    });
    // Trail polyline
    const trail = viewer.entities.add({
        polyline: {
            positions: [],
            width: 2,
            material: config.color.withAlpha(.8),
            clampToGround:false
        }
    });

    stages[index] = {
        webEl: document.getElementById(`stage${index + 1}_gps`),
        latest: null,
        lastUpdate: 0,
        normAlt: null,
        fired: false,
        entity: entity,
        positions: [],
        trail: trail
    };
});

// --- Telemetry Handler ---
function handleTelemetry(data) {
    const stage = stages[data.stage];
    if (!stage) return;

    const timestamp = Date.now();

    if (!data.lat || !data.lon) return;

    const lat = data.lat;
    const lon = data.lon;
    const alt = data.alt ?? stage.latest?.alt ?? 0;

    const point = { lat, lon, alt, fsm: data.fsm, timestamp };
    stage.latest = point;

    // Determine launch
    if (!stage.fired) {
        stage.normAlt = alt;
        if (point.fsm > 2) stage.fired = true;
    }

    const pos = Cesium.Cartesian3.fromDegrees(lon, lat, alt - (stage.normAlt || 0));
    stage.entity.position = pos;

    if (stage.webEl) stage.webEl.textContent = fmtGPS(point);

    if (stage.fired && (timestamp - stage.lastUpdate > TRAIL_UPDATE_MS || stage.positions.length === 0)) {
        stage.positions.push(pos);
        stage.trail.polyline.positions = stage.positions;
        stage.lastUpdate = timestamp;
    }
}

// --- GPS Formatter ---
const GPS_FORMAT = { lat: { decimals: 6, width: 10 }, lon: { decimals: 6, width: 10 }, alt: { decimals: 0, width: 7 } };
function fmt(num, decimals, width) {
    const s = num.toFixed(decimals);
    return width ? s.padStart(width) : s;
}
function fmtGPS(pt) {
    return `GPS: ${fmt(pt.lat, GPS_FORMAT.lat.decimals, GPS_FORMAT.lat.width)} ` +
           `${fmt(pt.lon, GPS_FORMAT.lon.decimals, GPS_FORMAT.lon.width)} ` + 
           `${fmt(pt.alt, GPS_FORMAT.alt.decimals, GPS_FORMAT.alt.width)}`;
}

// --- SSE Stream ---
window.onload = () => {
    const source = new EventSource("/stream");
    source.onmessage = (event) => handleTelemetry(JSON.parse(event.data));
    source.onerror = (err) => console.error("SSE error", err);
};