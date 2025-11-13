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
		altitude: 'Altitude',
		raw_baro_alt: 'Raw Barometer Altitude',
		raw_highg_ax: 'Raw HighG X', raw_highg_ay: 'Raw HighG Y', raw_highg_az: 'Raw HighG Z'
	};

	let data = [];
	let filteredData = [];
	let hasRendered = false;
	let lastMtime = null;
	let pollTimer = null;

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
	}

	function getSel(id) {
		return document.getElementById(id).value;
	}

	function filterBySelections() {
		const dataType = getSel('dataType');
		const from = getSel('fsmFrom');
		const to = getSel('fsmTo');
		const fromIdx = fsmOrder.indexOf(from);
		const toIdx = fsmOrder.indexOf(to);

		filteredData = data.filter((row) => {
			if (row.fsm && row.fsm !== 'nan') {
				const idx = fsmOrder.indexOf(row.fsm);
				if (idx < fromIdx || idx > toIdx) return false;
			}
			const v = row[dataType];
			return !(v === undefined || Number.isNaN(v) || v === 'nan');
		});
		console.log(`[FILTER] kept=${filteredData.length}`);
	}

	function getYAxisLabel(dataType) {
		if (dataType.includes('pos') || dataType.includes('altitude')) return 'Position (m)';
		if (dataType.includes('vel')) return 'Velocity (m/s)';
		if (dataType.includes('acc') || dataType.includes('highg')) return 'Acceleration (m/s²)';
		return 'Value';
	}

	function computeFSMMarkers() {
		const dataType = getSel('dataType');
		const from = getSel('fsmFrom');
		const to = getSel('fsmTo');
		const fromIdx = fsmOrder.indexOf(from);
		const toIdx = fsmOrder.indexOf(to);

		const values = data
			.filter((r) => {
				if (r.fsm && r.fsm !== 'nan') {
					const idx = fsmOrder.indexOf(r.fsm);
					if (idx < fromIdx || idx > toIdx) return false;
				}
				const v = r[dataType];
				return !(v === undefined || Number.isNaN(v) || v === 'nan');
			})
			.map((r) => r[dataType])
			.filter((v) => !Number.isNaN(v));

		// Use reduce instead of spread operator to avoid stack overflow with large arrays
		const maxVal = values.length ? values.reduce((a, b) => Math.max(a, b), -Infinity) : 1000;
		const minVal = values.length ? values.reduce((a, b) => Math.min(a, b), Infinity) : 0;
		const y = maxVal + (maxVal - minVal) * 0.1;

		const markers = [];
		let last = null;
		for (const r of data) {
			if (r.fsm && r.fsm !== 'nan' && r.fsm !== last) {
				const idx = fsmOrder.indexOf(r.fsm);
				if (idx >= fromIdx && idx <= toIdx) {
					markers.push({ x: r.timestamp, y, label: r.fsm });
				}
				last = r.fsm;
			}
		}
		return markers;
	}

	function render() {
		const dataType = getSel('dataType');
		const rawType = getSel('rawData');
		const from = getSel('fsmFrom');
		const to = getSel('fsmTo');
		const fromIdx = fsmOrder.indexOf(from);
		const toIdx = fsmOrder.indexOf(to);

		const traces = [];
		traces.push({
			x: filteredData.map((r) => r.timestamp),
			y: filteredData.map((r) => r[dataType]),
			type: 'scatter',
			mode: 'lines+markers',
			name: dataLabels[dataType] || dataType,
			line: { color: '#667eea', width: 2 },
			marker: { size: 3, color: '#667eea' }
		});

		if (rawType !== 'none' && data[0] && data[0][rawType] !== undefined) {
			const raw = data.filter((r) => {
				if (r.fsm && r.fsm !== 'nan') {
					const idx = fsmOrder.indexOf(r.fsm);
					if (idx < fromIdx || idx > toIdx) return false;
				}
				const v = r[rawType];
				return !(v === undefined || Number.isNaN(v) || v === 'nan');
			});
			traces.push({
				x: raw.map((r) => r.timestamp),
				y: raw.map((r) => rawType.startsWith('raw_highg') ? r[rawType] * 9.81 : r[rawType]),
				type: 'scatter',
				mode: 'lines',
				name: dataLabels[rawType] || rawType,
				line: { color: '#e74c3c', width: 1, dash: 'dash' },
				opacity: 0.7
			});
		}

		const markers = computeFSMMarkers();
		if (markers.length) {
			traces.push({
				x: markers.map((m) => m.x),
				y: markers.map((m) => m.y),
				type: 'scatter',
				mode: 'markers',
				name: 'FSM Changes',
				marker: { size: 10, color: '#f39c12', symbol: 'diamond', line: { color: '#e67e22', width: 2 } },
				text: markers.map((m) => m.label),
				textposition: 'top center',
				textfont: { size: 12, color: '#2c3e50', family: 'Arial, sans-serif' },
				hovertemplate: '<b>%{text}</b><br>Time: %{x:.2f}s<br><extra></extra>'
			});
		}

		const layout = {
			title: { text: `${dataLabels[dataType] || dataType} vs Time`, font: { size: 20, color: '#2c3e50' } },
			xaxis: { title: 'Time (seconds)', gridcolor: '#e9ecef', gridwidth: 1 },
			yaxis: { title: getYAxisLabel(dataType), gridcolor: '#e9ecef', gridwidth: 1 },
			plot_bgcolor: 'white', paper_bgcolor: 'white',
			font: { family: 'Segoe UI, Tahoma, Geneva, Verdana, sans-serif', size: 12, color: '#2c3e50' },
			legend: { x: 0.02, y: 0.98, bgcolor: 'rgba(255,255,255,0.8)', bordercolor: '#e9ecef', borderwidth: 1 },
			margin: { t: 60, r: 50, b: 60, l: 80 }
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
		// Use reduce instead of spread operator to avoid stack overflow with large arrays
		const timestamps = data.map((r) => r.timestamp);
		const maxTime = timestamps.reduce((a, b) => Math.max(a, b), -Infinity);
		const minTime = timestamps.reduce((a, b) => Math.min(a, b), Infinity);
		document.getElementById('totalPoints').textContent = data.length.toLocaleString();
		document.getElementById('filteredPoints').textContent = filteredData.length.toLocaleString();
		document.getElementById('flightDuration').textContent = `${(maxTime - minTime).toFixed(2)}s`;
		const vals = filteredData.map((r) => r[dataType]).filter((v) => !Number.isNaN(v));
		if (vals.length) {
			const maxVal = vals.reduce((a, b) => Math.max(a, b), -Infinity);
			const minVal = vals.reduce((a, b) => Math.min(a, b), Infinity);
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
		try {
			filterBySelections();
			render();
		} catch (e) {
			console.error(e);
		}
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
})();
