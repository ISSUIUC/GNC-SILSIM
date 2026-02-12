# Telemetry Live Plotter
A real-time telemetry visualization system for rocket launches.

Author: Maximilian Kulasik

## Overview
Telemetry Live Plotter is a visualization tool that supports:
- MQTT-based live telemetry
- Serial replay from recorded launches
- Offline map tile rendering

The system is designed to be modular, extensible, and stage-agnostic.

## Architecture
- Backend (Python)
    - Connects to MQTT or reads serial replay
    - Parses and filters telemetry
    - Broadcasts data via SSE

- Frontend (JavaScript)
    - Receives telemetry via EventSource
    - Updates plots and map overlays in real time

### Data Flow
MQTT Broker -> Backend -> SSE -> Frontend
Serial File -> Backend -> SSE -> Frontend

## Running the Project
### Starting the Backend
python `server.py`

The backend runs on a localhost port, which is printed to the console when starting.

### Configuration
All runtime behavior is controlled in `server.py`.

Variable  | Description 
USE_MQTT  | True for live MQTT mode, False for serial replay
MQTT_HOST | MQTT broker hostname
MQTT_PORT | MQTT broker port

Only one data source should be enabled at a time.

## Telemetry Data Format
Incoming telemetry must follow this JSON structure:

```json
{
    "data": {
        "type": "data",
        "value": {
            "barometer_altitude": float,
            "latitude": float,
            "longitude": float,
            "altitude": float,
            "FSM_State": integer,
            "is_sustainer": integer,
            ...
        }
    }
}
```

## MQTT Mode:
To use live telemetry via MQTT:
1. Set `USE_MQTT = True` in `server.py`
2. Configure:
    `MQTT_HOST`
    `MQTT_PORT`
3. Ensure the broker is running and accessible

The backend subscribes to:
- FlightData-Sustainer
- FlightData-Booster

## Serial Replay Mode:
To replay previous launch data:
1. Place your telemetry file at:
    `serial_replay/serial.txt`
2. Set `USE_MQTT = False` in `server.py`
3. Start the backend normally

The file must contain one telemetry packet per line in the expected JSON format.

## Offline Map Mode:
To use offline map tiles:
1. Generate XYZ tiles (EPSG:3857 projection) for your desired bounding box
2. Place tiles in:
    `tiles/{z}/{x}/{y}.png`
3. Uncomment `imageryProvider` in `main.js`
4. Adjust `maximumLevel` as needed

Note: Tile generation is not handled by this project. However, Cesium is installed locally for offline use.

## Extending the System
### Adding New Telemetry Fields
If new fields are added:
1. Update `filter_telemetry()` in backend
2. Ensure frontend expects the same key names
3. Maintain JSON structure:

```json
{
    "data": {
        "type": "data",
        "value": {
            "barometer_altitude": float,
            "latitude": float,
            "longitude": float,
            "altitude": float,
            "FSM_State": integer,
            "is_sustainer": integer,
            ...
        }
    }
}
```

### Switching Data Sources
All data sources must output telemetry in the same standardized format. This ensures the frontend does not require modification when changing input sources.

### Notes on FSM States
Currently the frontend assumes that FSM states 0-2 correspond to pre-launch phases. These states are used to reduce plotting of polylines and allow the rocket position to update during setup.

If FSM definitions change:
1. Update the frontend code where pre-launch logic depends on FSM states
2. Ensure the frontend visualization reflects the new FSM state mappings

This ensures that telemetry is interpreted correctly and that plots remain accurate.

### Additional Stages
The backend does not assume a fixed number of stages. Stage differentiation is determined dynamically from the is_sustainer field in incoming telemetry. 

To support additional stages:
1. Provide telemetry packets with a valid stage identifier
2. Add corresponding UI controls (button / map marker / overlay)
3. Ensuring the frontend recognizes the new stage index

No backend structural changes are required for additional stages.