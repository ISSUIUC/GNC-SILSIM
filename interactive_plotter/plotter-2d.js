// 2D Plotting functionality for the interactive plotter

(function() {
	'use strict';
	
	if (!window.PlotterCommon) {
		console.error('[2D Plotter] PlotterCommon not loaded');
		return;
	}
	
	const Common = window.PlotterCommon;
	
	// Expose 2D plotter functions
	window.Plotter2D = {
		// Render 2D plot
		render: function(filteredData, showFSMLabels, hasRendered) {
			const dataTypes = this.getSel('dataType'); // Array of selected data types
			const rawTypes = this.getSel('rawData'); // Array of selected raw data types
			const from = this.getSel('fsmFrom');
			const to = this.getSel('fsmTo');
			const fromIdx = Common.fsmIndexMap[from] ?? -1;
			const toIdx = Common.fsmIndexMap[to] ?? -1;

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
						name: Common.dataLabels[dataType] || dataType,
						line: { color: Common.traceColors[idx % Common.traceColors.length], width: 2 },
						marker: { size: 3, color: Common.traceColors[idx % Common.traceColors.length] }
					});
				});
			}

			// Render multiple raw data types
			if (rawTypes.length > 0 && !rawTypes.includes('none') && Common.data[0]) {
				rawTypes.forEach((rawType, rawIdx) => {
					// Filter raw data in a single pass
					const rawX = [];
					const rawY = [];
					
					// Check if this is a GPS conversion type
					const isGPSConversion = rawType === 'raw_gps_x' || rawType === 'raw_gps_y' || rawType === 'raw_gps_z';
					const multiplier = rawType.startsWith('raw_highg') ? 9.81 : 1;
					
					for (let i = 0; i < Common.data.length; i++) {
						const r = Common.data[i];
						
						// Check FSM range
						if (r.fsm && r.fsm !== 'nan') {
							const idx = Common.fsmIndexMap[r.fsm];
							if (idx === undefined || idx < fromIdx || idx > toIdx) continue;
						}
						
						let value = null;
						
						if (isGPSConversion) {
							// Convert GPS coordinates to local x, y, z
							const local = Common.convertGPSToLocal(
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
						const colorIdx = (dataTypes.length + rawIdx) % Common.traceColors.length;
						traces.push({
							x: rawX,
							y: rawY,
							type: 'scatter',
							mode: 'lines',
							name: Common.dataLabels[rawType] || rawType,
							line: { color: Common.traceColors[colorIdx], width: 1, dash: 'dash' },
							opacity: 0.7
						});
					}
				});
			}

			const fsmData = this.computeFSMMarkers(filteredData, dataTypes, from, to);
			const markers = fsmData.markers;
			
			// Add vertical lines for FSM state changes (toggled via showFSMLabels)
			const shapes = [];
			const annotations = [];
			
			if (showFSMLabels && markers.length > 0) {
				// Use pre-computed yMin/yMax from computeFSMMarkers
				const yMin = fsmData.yMin;
				const yMax = fsmData.yMax;
				
				markers.forEach((marker) => {
					// Add vertical line (shape) - use paper coordinates to span full plot height
					shapes.push({
						type: 'line',
						xref: 'x',
						yref: 'paper',
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
				: `${Common.dataLabels[firstDataType] || firstDataType} vs Time`;

			const layout = {
				title: { text: title, font: { size: 20, color: '#2c3e50' } },
				xaxis: { title: 'Time (seconds)', gridcolor: '#e9ecef', gridwidth: 1 },
				yaxis: { title: Common.getYAxisLabel(firstDataType), gridcolor: '#e9ecef', gridwidth: 1 },
				plot_bgcolor: 'white', paper_bgcolor: 'white',
				font: { family: 'Segoe UI, Tahoma, Geneva, Verdana, sans-serif', size: 12, color: '#2c3e50' },
				legend: { x: 0.02, y: 0.98, bgcolor: 'rgba(255,255,255,0.8)', bordercolor: '#e9ecef', borderwidth: 1 },
				margin: { t: 60, r: 50, b: 60, l: 80 },
				shapes: shapes,
				annotations: annotations,
				hovermode: 'x unified' // Show all traces at the same x (timestamp) value
			};
			const config = { responsive: true, displayModeBar: true, modeBarButtonsToRemove: ['pan2d', 'lasso2d', 'select2d'], displaylogo: false };

			if (hasRendered) {
				Plotly.react('plot', traces, layout, config);
			} else {
				Plotly.newPlot('plot', traces, layout, config);
			}
		},
		
		// Compute FSM markers for 2D view
		computeFSMMarkers: function(filteredData, dataTypes, from, to) {
			const fromIdx = Common.fsmIndexMap[from] ?? -1;
			const toIdx = Common.fsmIndexMap[to] ?? -1;

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
			for (let i = 0; i < Common.data.length; i++) {
				const r = Common.data[i];
				if (r.fsm && r.fsm !== 'nan' && r.fsm !== last) {
					const idx = Common.fsmIndexMap[r.fsm];
					if (idx !== undefined && idx >= fromIdx && idx <= toIdx) {
						markers.push({ 
							x: r.timestamp, 
							label: Common.formatFSMLabel(r.fsm),
							rawLabel: r.fsm
						});
					}
					last = r.fsm;
				}
			}
			return { markers, yMin: minVal, yMax: maxVal };
		},
		
		// Get selection value (helper)
		getSel: function(id) {
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
		},
		
		// Update info panel for 2D view
		updateInfoPanel: function(filteredData) {
			// Optimized: calculate min/max in a single pass
			let maxTime = -Infinity;
			let minTime = Infinity;
			for (let i = 0; i < Common.data.length; i++) {
				const t = Common.data[i].timestamp;
				if (t > maxTime) maxTime = t;
				if (t < minTime) minTime = t;
			}
			
			document.getElementById('totalPoints').textContent = Common.data.length.toLocaleString();
			document.getElementById('filteredPoints').textContent = filteredData.length.toLocaleString();
			document.getElementById('flightDuration').textContent = `${(maxTime - minTime).toFixed(2)}s`;
			
			const from = this.getSel('fsmFrom');
			const to = this.getSel('fsmTo');
			document.getElementById('currentFSM').textContent = from === to ? from : `${from} to ${to}`;
			
			// Update data statistics for 2D view
			const dataTypes = this.getSel('dataType');
			const rawTypes = this.getSel('rawData');
			const statsContainer = document.getElementById('dataStatsContainer');
			const statsList = document.getElementById('dataStatsList');
			
			// Collect all data types to show stats for
			const allDataTypes = [];
			
			// Add Kalman data types
			if (dataTypes.length > 0) {
				dataTypes.forEach(dt => {
					allDataTypes.push({
						type: dt,
						label: Common.dataLabels[dt] || dt,
						isRaw: false
					});
				});
			}
			
			// Add raw data types (excluding 'none')
			if (rawTypes.length > 0 && !rawTypes.includes('none')) {
				rawTypes.forEach(rt => {
					allDataTypes.push({
						type: rt,
						label: Common.dataLabels[rt] || rt,
						isRaw: true
					});
				});
			}
			
			// Calculate and display stats for each data type
			if (allDataTypes.length > 0) {
				statsContainer.style.display = 'block';
				statsList.innerHTML = '';
				
				const fromIdx = Common.fsmIndexMap[from] ?? -1;
				const toIdx = Common.fsmIndexMap[to] ?? -1;
				
				allDataTypes.forEach(({ type, label, isRaw }) => {
					let minVal = Infinity;
					let maxVal = -Infinity;
					let hasData = false;
					
					// Calculate min/max for this data type
					const dataSource = isRaw ? Common.data : filteredData;
					
					for (let i = 0; i < dataSource.length; i++) {
						const r = dataSource[i];
						
						// For raw data, check FSM range
						if (isRaw && r.fsm && r.fsm !== 'nan') {
							const idx = Common.fsmIndexMap[r.fsm];
							if (idx === undefined || idx < fromIdx || idx > toIdx) continue;
						}
						
						let value = null;
						
						// Handle GPS conversion types
						if (type === 'raw_gps_x' || type === 'raw_gps_y' || type === 'raw_gps_z') {
							const local = Common.convertGPSToLocal(
								r.raw_gps_latitude,
								r.raw_gps_longitude,
								r.raw_gps_altitude
							);
							
							if (local) {
								if (type === 'raw_gps_x') {
									value = local.z;
								} else if (type === 'raw_gps_y') {
									value = local.x;
								} else if (type === 'raw_gps_z') {
									value = local.y;
								}
							}
						} else {
							// Regular data type
							const multiplier = type.startsWith('raw_highg') ? 9.81 : 1;
							const v = r[type];
							if (v !== undefined && !Number.isNaN(v) && v !== 'nan') {
								value = v * multiplier;
							}
						}
						
						if (value !== null && !Number.isNaN(value)) {
							hasData = true;
							if (value > maxVal) maxVal = value;
							if (value < minVal) minVal = value;
						}
					}
					
					// Create stat item
					const statItem = document.createElement('div');
					statItem.className = 'data-stat-item';
					
					if (hasData && minVal !== Infinity && maxVal !== -Infinity) {
						statItem.innerHTML = `
							<span class="data-stat-label">${label}</span>
							<div class="data-stat-values">
								<span class="data-stat-min">Min: <span>${minVal.toFixed(2)}</span></span>
								<span class="data-stat-max">Max: <span>${maxVal.toFixed(2)}</span></span>
							</div>
						`;
					} else {
						statItem.innerHTML = `
							<span class="data-stat-label">${label}</span>
							<div class="data-stat-values">
								<span style="color: #6c757d;">No data</span>
							</div>
						`;
					}
					
					statsList.appendChild(statItem);
				});
			} else {
				statsContainer.style.display = 'none';
			}
		}
	};
})();

