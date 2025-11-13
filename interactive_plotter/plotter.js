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
		return document.getElementById(id).value;
	}

	function filterBySelections() {
		const dataType = getSel('dataType');
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
			
			// Check data value
			const v = row[dataType];
			if (v !== undefined && !Number.isNaN(v) && v !== 'nan') {
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
		const dataType = getSel('dataType');
		const from = getSel('fsmFrom');
		const to = getSel('fsmTo');
		const fromIdx = fsmIndexMap[from] ?? -1;
		const toIdx = fsmIndexMap[to] ?? -1;

		// Use already filtered data for value calculation (much faster)
		const values = [];
		for (let i = 0; i < filteredData.length; i++) {
			const v = filteredData[i][dataType];
			if (v !== undefined && !Number.isNaN(v)) {
				values.push(v);
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

	function render() {
		const dataType = getSel('dataType');
		const rawType = getSel('rawData');
		const from = getSel('fsmFrom');
		const to = getSel('fsmTo');
		const fromIdx = fsmIndexMap[from] ?? -1;
		const toIdx = fsmIndexMap[to] ?? -1;

		const traces = [];
		
		// Pre-extract x and y arrays in a single pass (much faster than multiple maps)
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
			line: { color: '#667eea', width: 2 },
			marker: { size: 3, color: '#667eea' }
		});

		if (rawType !== 'none' && data[0]) {
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
				traces.push({
					x: rawX,
					y: rawY,
					type: 'scatter',
					mode: 'lines',
					name: dataLabels[rawType] || rawType,
					line: { color: '#e74c3c', width: 1, dash: 'dash' },
					opacity: 0.7
				});
			}
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

		const layout = {
			title: { text: `${dataLabels[dataType] || dataType} vs Time`, font: { size: 20, color: '#2c3e50' } },
			xaxis: { title: 'Time (seconds)', gridcolor: '#e9ecef', gridwidth: 1 },
			yaxis: { title: getYAxisLabel(dataType), gridcolor: '#e9ecef', gridwidth: 1 },
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

	function setupControls() {
		['dataType', 'fsmFrom', 'fsmTo', 'rawData'].forEach((id) => {
			document.getElementById(id).addEventListener('change', onSelectionsChange);
		});
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
		Plotly.relayout('plot', { 'xaxis.autorange': true, 'yaxis.autorange': true });
	};

	// Toggle FSM labels visibility
	window.toggleFSMLabels = function toggleFSMLabels() {
		showFSMLabels = !showFSMLabels;
		const btn = document.getElementById('toggleFSMLabels');
		btn.textContent = showFSMLabels ? 'Hide FSM Labels' : 'Show FSM Labels';
		// Re-render to update labels
		onSelectionsChange();
	};
})();
