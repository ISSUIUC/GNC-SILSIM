// Minimal EKF Interactive Plotter - from scratch
// - Fetches JSON from /api/results
// - Subscribes to /events (SSE) to live-reload on CSV changes
// - Renders with Plotly and wires existing controls (dataType, fsmFrom, fsmTo, rawData)

(function () {
	const fsmOrder = [
		'STATE_IDLE', 'STATE_FIRST_BOOST', 'STATE_BURNOUT', 'STATE_COAST',
		'STATE_APOGEE', 'STATE_DROGUE_DEPLOY', 'STATE_DROGUE', 'STATE_MAIN_DEPLOY',
		'STATE_MAIN', 'STATE_LANDED', 'STATE_SUSTAINER_IGNITION', 'STATE_SECOND_BOOST',
		'STATE_FIRST_SEPARATION'
	];

	const dataLabels = {
		pos_x: 'Position X', pos_y: 'Position Y', pos_z: 'Position Z',
		vel_x: 'Velocity X', vel_y: 'Velocity Y', vel_z: 'Velocity Z',
		acc_x: 'Acceleration X', acc_y: 'Acceleration Y', acc_z: 'Acceleration Z',
		raw_gps_latitude: 'Raw GPS Latitude', raw_gps_longitude: 'Raw GPS Longitude', raw_gps_altitude: 'Raw GPS Altitude',
		raw_gps_x: 'GPS X (from Raw GPS Altitude)', raw_gps_y: 'GPS Y (from Raw GPS Longitude)', raw_gps_z: 'GPS Z (from Raw GPS Latitude)',
		raw_baro_alt: 'Raw Barometer Altitude',
		raw_highg_ax: 'Raw HighG X', raw_highg_ay: 'Raw HighG Y', raw_highg_az: 'Raw HighG Z'
	};
	
	// GPS conversion constants - will be set from first data point
	let LAUNCH_LAT = null; // in microdegrees
	let LAUNCH_LON = null; // in microdegrees
	let LAUNCH_LAT_DEG = null; // in degrees
	let LAUNCH_LON_DEG = null; // in degrees
	let GROUND_ALT = null; // meters (from first GPS altitude)
	const METERS_PER_DEG_LAT = 111320; // meters per degree latitude (constant)
	
	// Initialize launch coordinates from first valid GPS data point
	function initializeLaunchCoordinates() {
		if (!data || data.length === 0) return;
		
		// Find first row with valid GPS data
		for (let i = 0; i < data.length; i++) {
			const row = data[i];
			const lat = row.raw_gps_latitude;
			const lon = row.raw_gps_longitude;
			const alt = row.raw_gps_altitude;
			
			if (lat !== undefined && lon !== undefined && alt !== undefined &&
			    !Number.isNaN(lat) && !Number.isNaN(lon) && !Number.isNaN(alt) &&
			    lat !== 'nan' && lon !== 'nan' && alt !== 'nan') {
				// Store launch coordinates in microdegrees
				LAUNCH_LAT = lat;
				LAUNCH_LON = lon;
				GROUND_ALT = alt;
				
				// Convert to degrees for calculations
				LAUNCH_LAT_DEG = lat / 1e7;
				LAUNCH_LON_DEG = lon / 1e7;
				
				console.log(`[GPS] Launch coordinates initialized: lat=${LAUNCH_LAT_DEG.toFixed(6)}, lon=${LAUNCH_LON_DEG.toFixed(6)}, alt=${GROUND_ALT.toFixed(2)}m`);
				return;
			}
		}
	}
	
	// Convert GPS coordinates to local x, y, z coordinates
	// Input lat/lon are in microdegrees (degrees * 10^7)
	function convertGPSToLocal(lat, lon, alt) {
		if (lat === undefined || lon === undefined || alt === undefined ||
		    Number.isNaN(lat) || Number.isNaN(lon) || Number.isNaN(alt) ||
		    LAUNCH_LAT === null || LAUNCH_LON === null || GROUND_ALT === null) {
			return null;
		}
		
		// Convert from microdegrees to degrees
		const latDeg = lat / 1e7;
		const lonDeg = lon / 1e7;
		
		// Calculate meters per degree longitude at launch latitude
		const metersPerDegLon = METERS_PER_DEG_LAT * Math.cos(LAUNCH_LAT_DEG * Math.PI / 180);
		
		// Convert to local coordinates
		// X = east-west (positive east, from longitude)
		const x = (lonDeg - LAUNCH_LON_DEG) * metersPerDegLon;
		// Y = north-south (positive north, from latitude)
		const y = (latDeg - LAUNCH_LAT_DEG) * METERS_PER_DEG_LAT;
		// Z = up-down (positive up, relative to ground level)
		const z = alt - GROUND_ALT;
		
		return { x, y, z };
	}

	let data = [];
	let filteredData = [];
	let hasRendered = false;
	let lastMtime = null;
	let pollTimer = null;
	let showFSMLabels = false;
	let viewMode = '2D'; // '2D' or '3D'
	let gpsViewActive = false;
	let map = null;
	let gpsPolyline = null;
	let launchMarker = null;
	let cesiumViewer = null;
	let cesiumEntityCollection = null;
	
	// Performance optimization: cache FSM order indices
	const fsmIndexMap = {};
	fsmOrder.forEach((state, idx) => {
		fsmIndexMap[state] = idx;
	});
	
	// Debounce timer for filter changes
	let renderTimer = null;

	async function fetchResults() {
		console.log('[API] GET /api/results');
		const resp = await fetch('/api/results', { cache: 'no-store' });
		if (!resp.ok) throw new Error('Failed to fetch results');
		const payload = await resp.json();
		data = payload.rows || [];
		if (payload && typeof payload.mtime !== 'undefined') {
			lastMtime = payload.mtime;
		}
		console.log(`[API] rows=${data.length}`);
		if (!Array.isArray(data) || data.length === 0) throw new Error('No data found');
		
		// Initialize launch coordinates from first GPS data point
		initializeLaunchCoordinates();
	}

	function getSel(id) {
		// For multi-select, return array of selected values
		if (id === 'dataType' || id === 'rawData') {
			const wrapper = document.getElementById(id + 'Wrapper');
			if (!wrapper) return id === 'rawData' ? ['none'] : [];
			const checkboxes = wrapper.querySelectorAll('input[type="checkbox"]:checked');
			const values = Array.from(checkboxes).map(cb => cb.value);
			return id === 'rawData' ? (values.length > 0 ? values : ['none']) : (values.length > 0 ? values : []);
		}
		// For regular selects
		return document.getElementById(id).value;
	}

	function filterBySelections() {
		const dataTypes = getSel('dataType'); // Array of selected types
		const from = getSel('fsmFrom');
		const to = getSel('fsmTo');
		const fromIdx = fsmIndexMap[from] ?? -1;
		const toIdx = fsmIndexMap[to] ?? -1;

		// Optimized filtering: use cached index map and early returns
		filteredData = [];
		for (let i = 0; i < data.length; i++) {
			const row = data[i];
			
			// Check FSM range first (cheaper check)
			if (row.fsm && row.fsm !== 'nan') {
				const idx = fsmIndexMap[row.fsm];
				if (idx === undefined || idx < fromIdx || idx > toIdx) continue;
			}
			
			// Check if at least one selected data type has a valid value
			if (dataTypes.length > 0) {
				let hasValidValue = false;
				for (let j = 0; j < dataTypes.length; j++) {
					const v = row[dataTypes[j]];
					if (v !== undefined && !Number.isNaN(v) && v !== 'nan') {
						hasValidValue = true;
						break;
					}
				}
				if (hasValidValue) {
					filteredData.push(row);
				}
			} else {
				// No data types selected, include all rows that pass FSM filter
				filteredData.push(row);
			}
		}
		console.log(`[FILTER] kept=${filteredData.length}`);
	}

	function getYAxisLabel(dataType) {
		if (dataType.includes('pos') || dataType.includes('altitude')) return 'Position (m)';
		if (dataType.includes('vel')) return 'Velocity (m/s)';
		if (dataType.includes('acc') || dataType.includes('highg')) return 'Acceleration (m/s²)';
		return 'Value';
	}

	// Helper function to format FSM state names for display
	function formatFSMLabel(fsmState) {
		const labels = {
			'STATE_IDLE': 'IDLE',
			'STATE_FIRST_BOOST': 'FIRST BOOST',
			'STATE_BURNOUT': 'BURNOUT',
			'STATE_COAST': 'COAST',
			'STATE_APOGEE': 'APOGEE',
			'STATE_DROGUE_DEPLOY': 'DROGUE DEPLOY',
			'STATE_DROGUE': 'DROGUE',
			'STATE_MAIN_DEPLOY': 'MAIN DEPLOY',
			'STATE_MAIN': 'MAIN',
			'STATE_LANDED': 'LANDED',
			'STATE_SUSTAINER_IGNITION': 'SUSTAINER IGNITION',
			'STATE_SECOND_BOOST': 'SECOND BOOST',
			'STATE_FIRST_SEPARATION': 'FIRST SEPARATION'
		};
		return labels[fsmState] || fsmState.replace('STATE_', '').replace(/_/g, ' ');
	}

	function computeFSMMarkers() {
		const dataTypes = getSel('dataType'); // Array
		const from = getSel('fsmFrom');
		const to = getSel('fsmTo');
		const fromIdx = fsmIndexMap[from] ?? -1;
		const toIdx = fsmIndexMap[to] ?? -1;

		// Use already filtered data for value calculation (much faster)
		// Collect values from all selected data types
		const values = [];
		if (dataTypes.length > 0) {
			for (let i = 0; i < filteredData.length; i++) {
				dataTypes.forEach(dataType => {
					const v = filteredData[i][dataType];
					if (v !== undefined && !Number.isNaN(v)) {
						values.push(v);
					}
				});
			}
		}

		// Use reduce instead of spread operator to avoid stack overflow with large arrays
		const maxVal = values.length ? values.reduce((a, b) => Math.max(a, b), -Infinity) : 1000;
		const minVal = values.length ? values.reduce((a, b) => Math.min(a, b), Infinity) : 0;

		// Compute markers - only iterate once through data
		const markers = [];
		let last = null;
		for (let i = 0; i < data.length; i++) {
			const r = data[i];
			if (r.fsm && r.fsm !== 'nan' && r.fsm !== last) {
				const idx = fsmIndexMap[r.fsm];
				if (idx !== undefined && idx >= fromIdx && idx <= toIdx) {
					markers.push({ 
						x: r.timestamp, 
						label: formatFSMLabel(r.fsm),
						rawLabel: r.fsm
					});
				}
				last = r.fsm;
			}
		}
		return { markers, yMin: minVal, yMax: maxVal };
	}

	// Color palette for multiple traces
	const traceColors = [
		'#667eea', '#e74c3c', '#2ecc71', '#f39c12', '#9b59b6', 
		'#1abc9c', '#e67e22', '#3498db', '#95a5a6', '#34495e'
	];
	
	function render3D() {
		const showKalman = document.getElementById('showKalman3D').checked;
		const showGPS = document.getElementById('showGPS3D').checked;
		const showBaro = document.getElementById('showBaro3D').checked;
		const from = getSel('fsmFrom');
		const to = getSel('fsmTo');
		const fromIdx = fsmIndexMap[from] ?? -1;
		const toIdx = fsmIndexMap[to] ?? -1;

		const traces = [];
		
		// Render Kalman position data (always uses pos_x, pos_y, pos_z)
		if (showKalman) {
			const xVals = [];
			const yVals = [];
			const zVals = [];
			const timeVals = [];
			
			for (let i = 0; i < filteredData.length; i++) {
				const r = filteredData[i];
				// Map: X position → Z axis (vertical), Y position → X axis, Z position → Y axis
				xVals.push(r.pos_y);  // Y position on X axis (horizontal)
				yVals.push(r.pos_z);  // Z position on Y axis (horizontal)
				zVals.push(r.pos_x);  // X position on Z axis (vertical)
				timeVals.push(r.timestamp);
			}
			
			if (xVals.length > 0) {
				traces.push({
					x: xVals,
					y: yVals,
					z: zVals,
					type: 'scatter3d',
					mode: 'lines+markers',
					name: 'Kalman Position',
					line: { color: traceColors[0], width: 4 },
					marker: { size: 3, color: timeVals, colorscale: 'Viridis', showscale: true, colorbar: { title: 'Time (s)' } }
				});
			}
		}
		
		// Render GPS position data (always uses raw_gps_x, raw_gps_y, raw_gps_z)
		if (showGPS) {
			const xVals = [];
			const yVals = [];
			const zVals = [];
			const timeVals = [];
			
			for (let i = 0; i < data.length; i++) {
				const r = data[i];
				
				// Check FSM range
				if (r.fsm && r.fsm !== 'nan') {
					const idx = fsmIndexMap[r.fsm];
					if (idx === undefined || idx < fromIdx || idx > toIdx) continue;
				}
				
				const local = convertGPSToLocal(
					r.raw_gps_latitude,
					r.raw_gps_longitude,
					r.raw_gps_altitude
				);
				
				if (local) {
					// Map GPS coordinates: GPS X (altitude) → Z axis (vertical), GPS Y (longitude) → X axis, GPS Z (latitude) → Y axis
					xVals.push(local.x); // GPS Y (longitude) on X axis (horizontal)
					yVals.push(local.y); // GPS Z (latitude) on Y axis (horizontal)
					zVals.push(local.z); // GPS X (altitude) on Z axis (vertical)
					timeVals.push(r.timestamp);
				}
			}
			
			if (xVals.length > 0) {
				traces.push({
					x: xVals,
					y: yVals,
					z: zVals,
					type: 'scatter3d',
					mode: 'lines+markers',
					name: 'GPS Position',
					line: { color: traceColors[1], width: 2, dash: 'dash' },
					marker: { size: 3, color: traceColors[1], opacity: 0.7 }
				});
			}
		}
		
		// Render barometer altitude as vertical axis (if selected)
		// Note: This displays altitude along Z-axis (vertical) with X=Y=0, showing vertical trajectory
		if (showBaro) {
			const xVals = [];
			const yVals = [];
			const zVals = [];
			const timeVals = [];
			
			for (let i = 0; i < data.length; i++) {
				const r = data[i];
				
				// Check FSM range
				if (r.fsm && r.fsm !== 'nan') {
					const idx = fsmIndexMap[r.fsm];
					if (idx === undefined || idx < fromIdx || idx > toIdx) continue;
				}
				
				const alt = r.raw_baro_alt;
				if (alt !== undefined && !Number.isNaN(alt) && alt !== 'nan') {
					xVals.push(0); // Place at origin for X and Y (horizontal)
					yVals.push(0);
					zVals.push(alt); // Barometer altitude on Z axis (vertical)
					timeVals.push(r.timestamp);
				}
			}
			
			if (xVals.length > 0) {
				traces.push({
					x: xVals,
					y: yVals,
					z: zVals,
					type: 'scatter3d',
					mode: 'lines+markers',
					name: 'Barometer Altitude (Vertical)',
					line: { color: traceColors[2], width: 2, dash: 'dot' },
					marker: { size: 3, color: traceColors[2], opacity: 0.7 }
				});
			}
		}
		
		// If no traces, show message
		if (traces.length === 0) {
			const layout = {
				title: { text: '3D Position Trajectory', font: { size: 20, color: '#2c3e50' } },
				scene: {
					xaxis: { title: 'Y Position (m)', gridcolor: '#e9ecef', gridwidth: 1 },
					yaxis: { title: 'Z Position (m)', gridcolor: '#e9ecef', gridwidth: 1 },
					zaxis: { title: 'X Position (m) - Vertical', gridcolor: '#e9ecef', gridwidth: 1 },
					bgcolor: 'white'
				},
				plot_bgcolor: 'white',
				paper_bgcolor: 'white',
				font: { family: 'Segoe UI, Tahoma, Geneva, Verdana, sans-serif', size: 12, color: '#2c3e50' },
				margin: { t: 60, r: 50, b: 60, l: 80 }
			};
			
			const config = { responsive: true, displayModeBar: true, displaylogo: false };
			
			if (hasRendered) {
				Plotly.react('plot', [], layout, config);
			} else {
				Plotly.newPlot('plot', [], layout, config);
				hasRendered = true;
			}
			return;
		}
		
		const layout = {
			title: { text: '3D Position Trajectory', font: { size: 20, color: '#2c3e50' } },
			scene: {
				xaxis: { title: 'Y Position (m)', gridcolor: '#e9ecef', gridwidth: 1 },
				yaxis: { title: 'Z Position (m)', gridcolor: '#e9ecef', gridwidth: 1 },
				zaxis: { title: 'X Position (m) - Vertical', gridcolor: '#e9ecef', gridwidth: 1 },
				bgcolor: 'white',
				camera: {
					eye: { x: 1.5, y: 1.5, z: 1.5 }
				}
			},
			plot_bgcolor: 'white',
			paper_bgcolor: 'white',
			font: { family: 'Segoe UI, Tahoma, Geneva, Verdana, sans-serif', size: 12, color: '#2c3e50' },
			legend: { x: 0.02, y: 0.98, bgcolor: 'rgba(255,255,255,0.8)', bordercolor: '#e9ecef', borderwidth: 1 },
			margin: { t: 60, r: 50, b: 60, l: 80 }
		};
		
		const config = { responsive: true, displayModeBar: true, displaylogo: false };
		
		if (hasRendered) {
			Plotly.react('plot', traces, layout, config);
		} else {
			Plotly.newPlot('plot', traces, layout, config);
			hasRendered = true;
		}
		
		updateInfoPanel();
	}

	function render() {
		// If GPS view is active, render map instead of plot
		if (gpsViewActive) {
			renderGPSView();
			return;
		}
		
		// Route to appropriate render function based on view mode
		if (viewMode === '3D') {
			render3D();
			return;
		}
		
		const dataTypes = getSel('dataType'); // Array of selected data types
		const rawTypes = getSel('rawData'); // Array of selected raw data types
		const from = getSel('fsmFrom');
		const to = getSel('fsmTo');
		const fromIdx = fsmIndexMap[from] ?? -1;
		const toIdx = fsmIndexMap[to] ?? -1;

		const traces = [];
		
		// Render multiple Kalman data types
		if (dataTypes.length > 0) {
			dataTypes.forEach((dataType, idx) => {
				// Pre-extract x and y arrays in a single pass
				const xVals = [];
				const yVals = [];
				for (let i = 0; i < filteredData.length; i++) {
					xVals.push(filteredData[i].timestamp);
					yVals.push(filteredData[i][dataType]);
				}
				
		traces.push({
					x: xVals,
					y: yVals,
			type: 'scatter',
			mode: 'lines+markers',
			name: dataLabels[dataType] || dataType,
					line: { color: traceColors[idx % traceColors.length], width: 2 },
					marker: { size: 3, color: traceColors[idx % traceColors.length] }
				});
			});
		}

		// Render multiple raw data types
		if (rawTypes.length > 0 && !rawTypes.includes('none') && data[0]) {
			rawTypes.forEach((rawType, rawIdx) => {
				// Filter raw data in a single pass
				const rawX = [];
				const rawY = [];
				
				// Check if this is a GPS conversion type
				const isGPSConversion = rawType === 'raw_gps_x' || rawType === 'raw_gps_y' || rawType === 'raw_gps_z';
				const multiplier = rawType.startsWith('raw_highg') ? 9.81 : 1;
				
				for (let i = 0; i < data.length; i++) {
					const r = data[i];
					
					// Check FSM range
				if (r.fsm && r.fsm !== 'nan') {
						const idx = fsmIndexMap[r.fsm];
						if (idx === undefined || idx < fromIdx || idx > toIdx) continue;
					}
					
					let value = null;
					
					if (isGPSConversion) {
						// Convert GPS coordinates to local x, y, z
						const local = convertGPSToLocal(
							r.raw_gps_latitude,
							r.raw_gps_longitude,
							r.raw_gps_altitude
						);
						
						if (local) {
							// Map: X = altitude, Y = longitude, Z = latitude
							if (rawType === 'raw_gps_x') {
								value = local.z; // X from altitude
							} else if (rawType === 'raw_gps_y') {
								value = local.x; // Y from longitude
							} else if (rawType === 'raw_gps_z') {
								value = local.y; // Z from latitude
							}
						}
					} else {
						// Regular raw data
				const v = r[rawType];
						if (v !== undefined && !Number.isNaN(v) && v !== 'nan') {
							value = v * multiplier;
						}
					}
					
					if (value !== null && !Number.isNaN(value)) {
						rawX.push(r.timestamp);
						rawY.push(value);
					}
				}
				
				if (rawX.length > 0) {
					// Use different colors for raw data, offset from Kalman colors
					const colorIdx = (dataTypes.length + rawIdx) % traceColors.length;
			traces.push({
						x: rawX,
						y: rawY,
				type: 'scatter',
				mode: 'lines',
				name: dataLabels[rawType] || rawType,
						line: { color: traceColors[colorIdx], width: 1, dash: 'dash' },
				opacity: 0.7
					});
				}
			});
		}

		const fsmData = computeFSMMarkers();
		const markers = fsmData.markers;
		
		// Add vertical lines for FSM state changes (toggled via showFSMLabels)
		const shapes = [];
		const annotations = [];
		
		if (showFSMLabels && markers.length > 0) {
			// Use pre-computed yMin/yMax from computeFSMMarkers (already calculated from filteredData)
			const yMin = fsmData.yMin;
			const yMax = fsmData.yMax;
			
			markers.forEach((marker, idx) => {
				// Add vertical line (shape) - use paper coordinates to span full plot height
				shapes.push({
					type: 'line',
					xref: 'x',
					yref: 'paper',  // Use paper coordinates (0-1) to span full plot height
					x0: marker.x,
					x1: marker.x,
					y0: 0,
					y1: 1,
					line: {
						color: '#f39c12',
						width: 2,
						dash: 'dash'
					}
				});
				
				annotations.push({
					x: marker.x,
					y: yMax + (yMax - yMin) * 0.05,
					text: marker.label,
					showarrow: false,
					font: {
						size: 11,
						color: '#2c3e50',
						family: 'Arial, sans-serif'
					},
					bgcolor: 'rgba(255, 255, 255, 0.8)',
					bordercolor: '#f39c12',
					borderwidth: 1,
					borderpad: 4,
					textangle: -45,
					xanchor: 'left',
					yanchor: 'bottom'
				});
			});
			
			// Add a trace for hover tooltips on the vertical lines
			traces.push({
				x: markers.map((m) => m.x),
				y: markers.map(() => yMax),
				type: 'scatter',
				mode: 'markers',
				name: 'FSM Changes',
				marker: { size: 8, color: '#f39c12', symbol: 'diamond', line: { color: '#e67e22', width: 2 }, opacity: 0.7 },
				text: markers.map((m) => m.label),
				hovertemplate: '<b>%{text}</b><br>Time: %{x:.2f}s<br><extra></extra>',
				showlegend: true
			});
		}

		// Determine title and y-axis label from first selected data type
		const firstDataType = dataTypes.length > 0 ? dataTypes[0] : 'data';
		const title = dataTypes.length > 1 
			? `Multiple Data Types vs Time`
			: `${dataLabels[firstDataType] || firstDataType} vs Time`;

		const layout = {
			title: { text: title, font: { size: 20, color: '#2c3e50' } },
			xaxis: { title: 'Time (seconds)', gridcolor: '#e9ecef', gridwidth: 1 },
			yaxis: { title: getYAxisLabel(firstDataType), gridcolor: '#e9ecef', gridwidth: 1 },
			plot_bgcolor: 'white', paper_bgcolor: 'white',
			font: { family: 'Segoe UI, Tahoma, Geneva, Verdana, sans-serif', size: 12, color: '#2c3e50' },
			legend: { x: 0.02, y: 0.98, bgcolor: 'rgba(255,255,255,0.8)', bordercolor: '#e9ecef', borderwidth: 1 },
			margin: { t: 60, r: 50, b: 60, l: 80 },
			shapes: shapes,
			annotations: annotations
		};
		const config = { responsive: true, displayModeBar: true, modeBarButtonsToRemove: ['pan2d', 'lasso2d', 'select2d'], displaylogo: false };

		if (hasRendered) {
			Plotly.react('plot', traces, layout, config);
		} else {
			Plotly.newPlot('plot', traces, layout, config);
			hasRendered = true;
		}

		updateInfoPanel();
	}

	function updateInfoPanel() {
		const dataType = getSel('dataType');
		// Optimized: calculate min/max in a single pass
		let maxTime = -Infinity;
		let minTime = Infinity;
		for (let i = 0; i < data.length; i++) {
			const t = data[i].timestamp;
			if (t > maxTime) maxTime = t;
			if (t < minTime) minTime = t;
		}
		
		document.getElementById('totalPoints').textContent = data.length.toLocaleString();
		document.getElementById('filteredPoints').textContent = filteredData.length.toLocaleString();
		document.getElementById('flightDuration').textContent = `${(maxTime - minTime).toFixed(2)}s`;
		
		// Calculate min/max values in a single pass
		let maxVal = -Infinity;
		let minVal = Infinity;
		for (let i = 0; i < filteredData.length; i++) {
			const v = filteredData[i][dataType];
			if (v !== undefined && !Number.isNaN(v)) {
				if (v > maxVal) maxVal = v;
				if (v < minVal) minVal = v;
			}
		}
		
		if (maxVal !== -Infinity && minVal !== Infinity) {
			document.getElementById('maxValue').textContent = maxVal.toFixed(2);
			document.getElementById('minValue').textContent = minVal.toFixed(2);
		} else {
			document.getElementById('maxValue').textContent = 'N/A';
			document.getElementById('minValue').textContent = 'N/A';
		}
		
		const from = getSel('fsmFrom');
		const to = getSel('fsmTo');
		document.getElementById('currentFSM').textContent = from === to ? from : `${from} to ${to}`;
	}

	function onSelectionsChange() {
		// Debounce rapid filter changes for better performance
		if (renderTimer) {
			clearTimeout(renderTimer);
		}
		
		renderTimer = setTimeout(() => {
		try {
			filterBySelections();
				// Use requestAnimationFrame for smoother rendering
				requestAnimationFrame(() => {
			render();
				});
		} catch (e) {
			console.error(e);
			}
		}, 50); // 50ms debounce - fast enough to feel instant, but prevents excessive renders
	}

	function setupMultiSelect(wrapperId, buttonId, dropdownId, countId) {
		const wrapper = document.getElementById(wrapperId);
		const button = document.getElementById(buttonId);
		const dropdown = document.getElementById(dropdownId);
		const countSpan = document.getElementById(countId);
		
		if (!wrapper || !button || !dropdown) return;
		
		// Toggle dropdown
		button.addEventListener('click', (e) => {
			e.stopPropagation();
			const isOpen = dropdown.classList.contains('open');
			closeAllDropdowns();
			if (!isOpen) {
				dropdown.classList.add('open');
				button.classList.add('open');
			}
		});
		
		// Handle checkbox clicks with shift-click support
		const checkboxes = dropdown.querySelectorAll('input[type="checkbox"]');
		checkboxes.forEach(checkbox => {
			checkbox.addEventListener('click', (e) => {
				e.stopPropagation();
				// Shift-click toggles without closing dropdown
				if (!e.shiftKey) {
					// Regular click - allow default behavior
				}
				updateMultiSelectDisplay(wrapperId, buttonId, countId);
				onSelectionsChange();
			});
		});
		
		// Close dropdown when clicking outside
		document.addEventListener('click', (e) => {
			if (!wrapper.contains(e.target)) {
				dropdown.classList.remove('open');
				button.classList.remove('open');
			}
		});
		
		// Initialize display
		updateMultiSelectDisplay(wrapperId, buttonId, countId);
	}
	
	function closeAllDropdowns() {
		document.querySelectorAll('.multiselect-dropdown').forEach(dd => dd.classList.remove('open'));
		document.querySelectorAll('.multiselect-button').forEach(btn => btn.classList.remove('open'));
	}
	
	function updateMultiSelectDisplay(wrapperId, buttonId, countId) {
		const wrapper = document.getElementById(wrapperId);
		const button = document.getElementById(buttonId);
		const countSpan = document.getElementById(countId);
		if (!wrapper || !button) return;
		
		const checkboxes = wrapper.querySelectorAll('input[type="checkbox"]:checked');
		const count = checkboxes.length;
		
		if (count === 0) {
			button.querySelector('span:first-child').textContent = 
				wrapperId === 'dataTypeWrapper' ? 'Select data types...' : 'Select raw data...';
			if (countSpan) countSpan.textContent = '';
		} else if (count === 1) {
			// Show the single selected option name
			const label = checkboxes[0].closest('.multiselect-option').querySelector('label');
			button.querySelector('span:first-child').textContent = label ? label.textContent : checkboxes[0].value;
			if (countSpan) countSpan.textContent = '';
		} else {
			// Show "Multiple options Selected" for multiple selections
			button.querySelector('span:first-child').textContent = 
				wrapperId === 'dataTypeWrapper' ? 'Multiple Options Selected' : 'Multiple Options Selected';
			if (countSpan) countSpan.textContent = `(${count})`;
		}
	}

	function setupControls() {
		// Setup multi-select dropdowns
		setupMultiSelect('dataTypeWrapper', 'dataTypeButton', 'dataTypeDropdown', 'dataTypeCount');
		setupMultiSelect('rawDataWrapper', 'rawDataButton', 'rawDataDropdown', 'rawDataCount');
		
		// Setup regular selects
		['fsmFrom', 'fsmTo'].forEach((id) => {
			document.getElementById(id).addEventListener('change', onSelectionsChange);
		});
		
		// Set default selection for dataType (first option)
		const firstDataType = document.querySelector('#dataTypeWrapper input[type="checkbox"]');
		if (firstDataType) {
			firstDataType.checked = true;
			updateMultiSelectDisplay('dataTypeWrapper', 'dataTypeButton', 'dataTypeCount');
		}
	}

	function setupSSE() {
		try {
			const es = new EventSource('/events');
			es.onopen = () => console.log('[SSE] connected');
			es.onmessage = async () => {
				console.log('[SSE] update');
				await fetchResults();
				filterBySelections();
				render();
				// Update GPS view if active
				if (gpsViewActive && map) {
					renderGPSView();
				}
			};
			es.onerror = (e) => {
				console.warn('[SSE] error', e);
				// SSE may be blocked; fallback to polling ensures updates
				startPollingFallback();
			};
		} catch {}
	}

	async function pollOnce() {
		try {
			const resp = await fetch('/api/results', { cache: 'no-store' });
			if (!resp.ok) return;
			const payload = await resp.json();
			if (typeof payload.mtime !== 'undefined' && payload.mtime !== lastMtime) {
				console.log('[POLL] change detected');
				lastMtime = payload.mtime;
				data = payload.rows || [];
				filterBySelections();
				render();
				// Update GPS view if active
				if (gpsViewActive && map) {
					renderGPSView();
				}
			}
		} catch {}
	}

	function startPollingFallback() {
		if (pollTimer) return;
		console.log('[POLL] starting fallback polling');
		pollTimer = setInterval(pollOnce, 2000);
	}

	async function boot() {
		try {
			await fetchResults();
			setupControls();
			filterBySelections();
			render();
			setupSSE();
			// Also enable polling as a safety net in case SSE never connects
			setTimeout(() => {
				if (!lastMtime) {
					startPollingFallback();
				}
			}, 3000);
		} catch (e) {
			console.error(e);
			const plot = document.getElementById('plot');
			plot.innerHTML = '';
			const div = document.createElement('div');
			div.className = 'error';
			div.textContent = (e && e.message) ? e.message : 'Failed to load data';
			plot.appendChild(div);
		}
	}

	document.addEventListener('DOMContentLoaded', boot);

	// Expose reset for the button
	window.resetView = function resetView() {
		if (gpsViewActive && cesiumViewer) {
			// Reset Cesium camera to view all GPS points
			if (cesiumEntityCollection && cesiumEntityCollection.values.length > 0) {
				const positions = [];
				cesiumEntityCollection.values.forEach(entity => {
					if (entity.polyline && entity.polyline.positions) {
						const polyPositions = entity.polyline.positions.getValue();
						if (Array.isArray(polyPositions)) {
							positions.push(...polyPositions);
						}
					}
				});
				if (positions.length > 0) {
					const boundingSphere = Cesium.BoundingSphere.fromPoints(positions);
					cesiumViewer.camera.flyToBoundingSphere(boundingSphere, {
						duration: 1.0,
						offset: new Cesium.HeadingPitchRange(0, -0.5, boundingSphere.radius * 1.5)
					});
				}
			}
			return;
		}
		
		if (viewMode === '3D') {
			// Reset 3D camera view
			Plotly.relayout('plot', {
				'scene.camera': {
					eye: { x: 1.5, y: 1.5, z: 1.5 }
				}
			});
		} else {
			// Reset 2D view
			Plotly.relayout('plot', { 'xaxis.autorange': true, 'yaxis.autorange': true });
		}
	};
	
	// Toggle GPS View
	window.toggleGPSView = function toggleGPSView() {
		gpsViewActive = !gpsViewActive;
		const btn = document.getElementById('gpsViewBtn');
		const plotDiv = document.getElementById('plot');
		const mapDiv = document.getElementById('map');
		
		if (gpsViewActive) {
			btn.textContent = 'Plot View';
			// Show Cesium, hide plot and map
			const cesiumDiv = document.getElementById('cesiumContainer');
			if (plotDiv) plotDiv.style.display = 'none';
			if (mapDiv) mapDiv.style.display = 'none';
			if (cesiumDiv) {
				cesiumDiv.style.display = 'block';
				cesiumDiv.classList.add('gps-view-active');
			}
			renderGPSView();
		} else {
			btn.textContent = 'GPS View';
			// Show plot normally, hide Cesium
			const cesiumDiv = document.getElementById('cesiumContainer');
			if (plotDiv) plotDiv.style.display = 'block';
			if (mapDiv) mapDiv.style.display = 'none';
			if (cesiumDiv) {
				cesiumDiv.style.display = 'none';
				cesiumDiv.classList.remove('gps-view-active');
			}
			// Re-render the regular 3D view
			hasRendered = false;
			onSelectionsChange();
		}
	};
	
	function initMap() {
		// Initialize map centered on launch coordinates
		if (LAUNCH_LAT_DEG === null || LAUNCH_LON_DEG === null) {
			console.error('[GPS View] Launch coordinates not initialized');
			return;
		}
		
		map = L.map('map').setView([LAUNCH_LAT_DEG, LAUNCH_LON_DEG], 15);
		
		// Add OpenStreetMap tiles
		L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
			attribution: '© OpenStreetMap contributors',
			maxZoom: 19
		}).addTo(map);
		
		// Add launch site marker
		const launchIcon = L.divIcon({
			className: 'launch-marker',
			html: '<div style="background-color: #e74c3c; width: 20px; height: 20px; border-radius: 50%; border: 3px solid white; box-shadow: 0 2px 4px rgba(0,0,0,0.3);"></div>',
			iconSize: [20, 20],
			iconAnchor: [10, 10]
		});
		
		launchMarker = L.marker([LAUNCH_LAT_DEG, LAUNCH_LON_DEG], { icon: launchIcon })
			.addTo(map)
			.bindPopup('<b>Launch Site</b><br>Lat: ' + LAUNCH_LAT_DEG.toFixed(6) + '<br>Lon: ' + LAUNCH_LON_DEG.toFixed(6));
	}
	
	function initCesium() {
		if (cesiumViewer) return; // Already initialized
		
		if (LAUNCH_LAT_DEG === null || LAUNCH_LON_DEG === null) {
			console.error('[GPS View] Launch coordinates not initialized');
			return;
		}
		
		const cesiumDiv = document.getElementById('cesiumContainer');
		if (!cesiumDiv) {
			console.error('[GPS View] Cesium container not found');
			return;
		}
		
		try {
			// Initialize Cesium viewer with simpler configuration
			// Use EllipsoidTerrainProvider for basic terrain (no external API needed)
			cesiumViewer = new Cesium.Viewer('cesiumContainer', {
				terrainProvider: new Cesium.EllipsoidTerrainProvider(),
				baseLayerPicker: false,
				geocoder: false,
				homeButton: false,
				infoBox: false,
				navigationHelpButton: false,
				navigationInstructionsInitiallyVisible: false,
				sceneModePicker: false,
				selectionIndicator: false,
				timeline: false,
				animation: false,
				fullscreenButton: false,
				vrButton: false
			});
			
			// Remove default imagery layers and add OpenStreetMap
			cesiumViewer.imageryLayers.removeAll();
			const osmLayer = cesiumViewer.imageryLayers.addImageryProvider(
				new Cesium.OpenStreetMapImageryProvider({
					url: 'https://a.tile.openstreetmap.org/'
				})
			);
			
			// Set initial camera position looking down at the launch site
			cesiumViewer.camera.setView({
				destination: Cesium.Cartesian3.fromDegrees(
					LAUNCH_LON_DEG,
					LAUNCH_LAT_DEG,
					1000 // Height in meters
				),
				orientation: {
					heading: 0.0,
					pitch: Cesium.Math.toRadians(-90), // Look straight down
					roll: 0.0
				}
			});
			
			// Create entity collection for GPS trajectory
			cesiumEntityCollection = cesiumViewer.entities;
			
			console.log('[GPS View] Cesium initialized successfully');
		} catch (error) {
			console.error('[GPS View] Error initializing Cesium:', error);
			cesiumViewer = null;
		}
	}
	
	function renderGPSView() {
		if (!data || data.length === 0) return;
		
		// Show Cesium container first, then initialize
		const cesiumDiv = document.getElementById('cesiumContainer');
		if (cesiumDiv) {
			cesiumDiv.style.display = 'block';
			cesiumDiv.classList.add('gps-view-active');
		}
		
		// Initialize Cesium if not already done
		if (!cesiumViewer) {
			// Wait a bit for container to be visible
			setTimeout(() => {
				initCesium();
				if (cesiumViewer) {
					renderGPSData();
				}
			}, 100);
            return;
        }
		
		renderGPSData();
	}
	
	function renderGPSData() {
		if (!cesiumViewer || !data || data.length === 0) return;
		
		const from = getSel('fsmFrom');
		const to = getSel('fsmTo');
		const fromIdx = fsmIndexMap[from] ?? -1;
		const toIdx = fsmIndexMap[to] ?? -1;
		
		// Collect GPS points with altitude
		const gpsPoints = [];
		let lastValidPoint = null;
		let minAlt = Infinity;
		let maxAlt = -Infinity;
		let minLat = Infinity;
		let maxLat = -Infinity;
		let minLon = Infinity;
		let maxLon = -Infinity;
		
		for (let i = 0; i < data.length; i++) {
			const r = data[i];
			
			// Check FSM range
			if (r.fsm && r.fsm !== 'nan') {
				const idx = fsmIndexMap[r.fsm];
				if (idx === undefined || idx < fromIdx || idx > toIdx) continue;
			}
			
			const lat = r.raw_gps_latitude;
			const lon = r.raw_gps_longitude;
			const alt = r.raw_gps_altitude;
			
			if (lat !== undefined && lon !== undefined && alt !== undefined &&
			    !Number.isNaN(lat) && !Number.isNaN(lon) && !Number.isNaN(alt) &&
			    lat !== 'nan' && lon !== 'nan' && alt !== 'nan') {
				const latDeg = lat / 1e7;
				const lonDeg = lon / 1e7;
				gpsPoints.push([latDeg, lonDeg, alt, r.timestamp]);
				lastValidPoint = [latDeg, lonDeg, alt, r.timestamp];
				
				if (alt < minAlt) minAlt = alt;
				if (alt > maxAlt) maxAlt = alt;
				if (latDeg < minLat) minLat = latDeg;
				if (latDeg > maxLat) maxLat = latDeg;
				if (lonDeg < minLon) minLon = lonDeg;
				if (lonDeg > maxLon) maxLon = lonDeg;
			}
		}
		
		if (gpsPoints.length === 0) return;
		
		// Clear existing entities
		cesiumEntityCollection.removeAll();
		
		// Create positions array for the polyline
		const positions = [];
		const colors = [];
		
		for (let i = 0; i < gpsPoints.length; i++) {
			const [latDeg, lonDeg, alt, timestamp] = gpsPoints[i];
			
			// Convert to Cesium Cartesian3 (longitude, latitude, height)
			positions.push(
				Cesium.Cartesian3.fromDegrees(lonDeg, latDeg, alt)
			);
			
			// Color based on altitude
			const altRange = maxAlt - minAlt;
			const normAlt = altRange > 0 ? (alt - minAlt) / altRange : 0.5;
			const color = getAltitudeColorRGB(normAlt);
			colors.push(
				Cesium.Color.fromBytes(color.r, color.g, color.b, 255)
			);
		}
		
		// Create polyline segments with color-coded altitude
		if (positions.length > 1) {
			// Create segments with different colors based on altitude
			for (let i = 0; i < positions.length - 1; i++) {
				const color = colors[i];
				cesiumEntityCollection.add({
					polyline: {
						positions: [positions[i], positions[i + 1]],
						width: 5,
						material: color.withAlpha(0.9),
						clampToGround: false,
						arcType: Cesium.ArcType.GEODESIC
					}
				});
			}
		}
		
		// Add launch site marker
		cesiumEntityCollection.add({
			position: Cesium.Cartesian3.fromDegrees(LAUNCH_LON_DEG, LAUNCH_LAT_DEG, GROUND_ALT),
			point: {
				pixelSize: 12,
				color: Cesium.Color.RED,
				outlineColor: Cesium.Color.WHITE,
				outlineWidth: 2,
				heightReference: Cesium.HeightReference.CLAMP_TO_GROUND
			},
			label: {
				text: 'Launch Site',
				font: '14pt sans-serif',
				fillColor: Cesium.Color.WHITE,
				outlineColor: Cesium.Color.BLACK,
				outlineWidth: 2,
				style: Cesium.LabelStyle.FILL_AND_OUTLINE,
				verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
				pixelOffset: new Cesium.Cartesian2(0, -32)
			}
		});
		
		// Add end position marker
		if (lastValidPoint) {
			const [endLat, endLon, endAlt] = lastValidPoint;
			cesiumEntityCollection.add({
				position: Cesium.Cartesian3.fromDegrees(endLon, endLat, endAlt),
				point: {
					pixelSize: 10,
					color: Cesium.Color.GREEN,
					outlineColor: Cesium.Color.WHITE,
					outlineWidth: 2,
					heightReference: Cesium.HeightReference.NONE
				},
				label: {
					text: 'End Position',
					font: '14pt sans-serif',
					fillColor: Cesium.Color.WHITE,
					outlineColor: Cesium.Color.BLACK,
					outlineWidth: 2,
					style: Cesium.LabelStyle.FILL_AND_OUTLINE,
					verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
					pixelOffset: new Cesium.Cartesian2(0, -32)
				}
			});
		}
		
		// Fly to the trajectory
		if (positions.length > 0) {
			const boundingSphere = Cesium.BoundingSphere.fromPoints(positions);
			// Calculate a good viewing angle - look at the trajectory from an angle
			const height = Math.max(boundingSphere.radius * 2, 500); // Minimum 500m height
			cesiumViewer.camera.flyToBoundingSphere(boundingSphere, {
				duration: 2.0,
				offset: new Cesium.HeadingPitchRange(
					0, // Heading (north)
					Cesium.Math.toRadians(-45), // Pitch (45 degrees down from horizontal)
					height // Distance from center
				)
			});
		}
		
		// Ensure Cesium container is visible and resized
		const cesiumDiv = document.getElementById('cesiumContainer');
		if (cesiumDiv && cesiumViewer) {
			cesiumDiv.style.display = 'block';
			// Resize Cesium viewer to match container
			setTimeout(() => {
				if (cesiumViewer) {
					cesiumViewer.resize();
				}
			}, 100);
		}
	}
	
	// Helper function to get RGB color from normalized altitude
	function getAltitudeColorRGB(normalizedAlt) {
		const t = Math.max(0, Math.min(1, normalizedAlt));
		
		if (t < 0.33) {
			const localT = t / 0.33;
			return {
				r: Math.round(52 + (46 - 52) * localT),
				g: Math.round(152 + (204 - 152) * localT),
				b: Math.round(219 + (113 - 219) * localT)
			};
		} else if (t < 0.67) {
			const localT = (t - 0.33) / 0.34;
			return {
				r: Math.round(46 + (243 - 46) * localT),
				g: Math.round(204 + (156 - 204) * localT),
				b: Math.round(113 + (18 - 113) * localT)
			};
		} else {
			const localT = (t - 0.67) / 0.33;
			return {
				r: Math.round(243 + (231 - 243) * localT),
				g: Math.round(156 + (76 - 156) * localT),
				b: Math.round(18 + (60 - 18) * localT)
			};
		}
	}
	
	// Helper function to get color based on normalized altitude (0-1)
	function getAltitudeColor(normalizedAlt) {
		// Clamp to 0-1 range
		const t = Math.max(0, Math.min(1, normalizedAlt));
		
		// Color gradient: blue (low) -> green (medium-low) -> yellow (medium-high) -> red (high)
		if (t < 0.33) {
			// Blue to green
			const localT = t / 0.33;
			const r = Math.round(52 + (46 - 52) * localT);  // 52 -> 46
			const g = Math.round(152 + (204 - 152) * localT);  // 152 -> 204
			const b = Math.round(219 + (113 - 219) * localT);  // 219 -> 113
			return `rgb(${r}, ${g}, ${b})`;
		} else if (t < 0.67) {
			// Green to yellow
			const localT = (t - 0.33) / 0.34;
			const r = Math.round(46 + (243 - 46) * localT);  // 46 -> 243
			const g = Math.round(204 + (156 - 204) * localT);  // 204 -> 156
			const b = Math.round(113 + (18 - 113) * localT);  // 113 -> 18
			return `rgb(${r}, ${g}, ${b})`;
		} else {
			// Yellow to red
			const localT = (t - 0.67) / 0.33;
			const r = Math.round(243 + (231 - 243) * localT);  // 243 -> 231
			const g = Math.round(156 + (76 - 156) * localT);  // 156 -> 76
			const b = Math.round(18 + (60 - 18) * localT);  // 18 -> 60
			return `rgb(${r}, ${g}, ${b})`;
		}
	}

	// Toggle FSM labels visibility
	window.toggleFSMLabels = function toggleFSMLabels() {
		showFSMLabels = !showFSMLabels;
		const btn = document.getElementById('toggleFSMLabels');
		btn.textContent = showFSMLabels ? 'Hide FSM Labels' : 'Show FSM Labels';
		// Re-render to update labels
		onSelectionsChange();
	};
	
	// Switch between 2D and 3D view
	window.switchView = function switchView(mode) {
		viewMode = mode;
		
		// Update tab appearance
		document.getElementById('view2D').classList.toggle('active', mode === '2D');
		document.getElementById('view3D').classList.toggle('active', mode === '3D');
		
		// Show/hide GPS View button based on mode
		const gpsViewBtn = document.getElementById('gpsViewBtn');
		if (gpsViewBtn) {
			if (mode === '3D') {
				gpsViewBtn.style.display = '';
			} else {
				gpsViewBtn.style.display = 'none';
				// If GPS view is active, switch back to plot view
				if (gpsViewActive) {
					toggleGPSView();
				}
			}
		}
		
		// Show/hide controls based on mode
		if (mode === '3D') {
			// In 3D mode, hide some controls and update available options
			update3DControls();
		} else {
			// In 2D mode, show all controls
			update2DControls();
		}
		
		// Reset render state and re-render
		hasRendered = false;
		onSelectionsChange();
	};
	
	function update3DControls() {
		// Hide FSM labels button in 3D mode (not applicable)
		const toggleBtn = document.getElementById('toggleFSMLabels');
		if (toggleBtn) toggleBtn.style.display = 'none';
		
		// Show GPS View button in 3D mode
		const gpsViewBtn = document.getElementById('gpsViewBtn');
		if (gpsViewBtn) gpsViewBtn.style.display = '';
		
		// Show 3D controls, hide 2D controls
		const controls3D = document.getElementById('controls3D');
		const controls2D = document.getElementById('controls2D');
		if (controls3D) controls3D.style.display = 'flex';
		if (controls2D) controls2D.style.display = 'none';
		
		// Set up event listeners for 3D checkboxes
		const showKalman3D = document.getElementById('showKalman3D');
		const showGPS3D = document.getElementById('showGPS3D');
		const showBaro3D = document.getElementById('showBaro3D');
		
		if (showKalman3D) {
			showKalman3D.onchange = onSelectionsChange;
		}
		if (showGPS3D) {
			showGPS3D.onchange = onSelectionsChange;
		}
		if (showBaro3D) {
			showBaro3D.onchange = onSelectionsChange;
		}
	}
	
	function update2DControls() {
		// Show FSM labels button in 2D mode
		const toggleBtn = document.getElementById('toggleFSMLabels');
		if (toggleBtn) toggleBtn.style.display = '';
		
		// Show 2D controls, hide 3D controls
		const controls3D = document.getElementById('controls3D');
		const controls2D = document.getElementById('controls2D');
		if (controls3D) controls3D.style.display = 'none';
		if (controls2D) controls2D.style.display = 'flex';
		
		// Show all options in 2D mode
		const dataTypeDropdown = document.getElementById('dataTypeDropdown');
		if (dataTypeDropdown) {
			const options = dataTypeDropdown.querySelectorAll('.multiselect-option');
			options.forEach(opt => {
				opt.style.display = 'flex';
			});
		}
		
		const rawDataDropdown = document.getElementById('rawDataDropdown');
		if (rawDataDropdown) {
			const options = rawDataDropdown.querySelectorAll('.multiselect-option');
			options.forEach(opt => {
				opt.style.display = 'flex';
			});
		}
	}
})();
