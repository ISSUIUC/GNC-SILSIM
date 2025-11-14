// Main plotter coordination - handles state, controls, and routing between 2D/3D views

(function() {
	'use strict';
	
	if (!window.PlotterCommon) {
		console.error('[Main Plotter] PlotterCommon not loaded');
		return;
	}
	
	if (!window.Plotter2D) {
		console.error('[Main Plotter] Plotter2D not loaded');
		return;
	}
	
	if (!window.Plotter3D) {
		console.error('[Main Plotter] Plotter3D not loaded');
		return;
	}
	
	const Common = window.PlotterCommon;
	const Plotter2D = window.Plotter2D;
	const Plotter3D = window.Plotter3D;
	
	// State variables
	let filteredData = [];
	let hasRendered = false;
	let pollTimer = null;
	let showFSMLabels = false;
	let viewMode = '2D'; // '2D' or '3D'
	let gpsViewActive = false;
	
	// Debounce timer for filter changes
	let renderTimer = null;
	
	// Optimized filter data by selections (for 2D view) - processes in chunks for large datasets
	// Returns a Promise that resolves when filtering is complete
	function filterBySelections() {
		const dataTypes = Plotter2D.getSel('dataType'); // Array of selected types
		const from = Plotter2D.getSel('fsmFrom');
		const to = Plotter2D.getSel('fsmTo');
		const fromIdx = Common.fsmIndexMap[from] ?? -1;
		const toIdx = Common.fsmIndexMap[to] ?? -1;

		// For large datasets, process in chunks to avoid blocking the UI
		const CHUNK_SIZE = 50000; // Process 50k rows at a time
		const totalRows = Common.data.length;
		
		if (totalRows > CHUNK_SIZE) {
			// Show loading for large datasets
			Common.showLoading('Filtering data...', `Processing ${totalRows.toLocaleString()} rows...`);
			
			// Process in chunks asynchronously - return a Promise
			return new Promise((resolve) => {
				filteredData = [];
				let processed = 0;
				
				const processChunk = (startIdx) => {
					const endIdx = Math.min(startIdx + CHUNK_SIZE, totalRows);
					
					for (let i = startIdx; i < endIdx; i++) {
						const row = Common.data[i];
						
						// Check FSM range first (cheaper check)
						if (row.fsm && row.fsm !== 'nan') {
							const idx = Common.fsmIndexMap[row.fsm];
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
					
					processed = endIdx;
					
					// Update progress
					if (endIdx < totalRows) {
						const progress = Math.round((processed / totalRows) * 100);
						Common.showLoading('Filtering data...', `Processing ${processed.toLocaleString()} / ${totalRows.toLocaleString()} rows (${progress}%)`);
						
						// Process next chunk asynchronously
						setTimeout(() => processChunk(endIdx), 0);
					} else {
						// Done processing
						console.log(`[FILTER] kept=${filteredData.length}`);
						Common.hideLoading();
						resolve();
					}
				};
				
				// Start processing
				processChunk(0);
			});
		} else {
			// Small dataset - process synchronously (fast)
			filteredData = [];
			for (let i = 0; i < Common.data.length; i++) {
				const row = Common.data[i];
				
				// Check FSM range first (cheaper check)
				if (row.fsm && row.fsm !== 'nan') {
					const idx = Common.fsmIndexMap[row.fsm];
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
			return Promise.resolve();
		}
	}
	
	// Main render function - routes to appropriate view
	function render() {
		// If GPS view is active, render map instead of plot
		if (gpsViewActive) {
			Plotter3D.renderGPSView();
			return;
		}
		
		// Route to appropriate render function based on view mode
		if (viewMode === '3D') {
			Plotter3D.render(hasRendered);
			hasRendered = true;
		} else {
			Plotter2D.render(filteredData, showFSMLabels, hasRendered);
			hasRendered = true;
			Plotter2D.updateInfoPanel(filteredData);
		}
	}
	
	// Update info panel (for 2D view)
	function updateInfoPanel() {
		if (viewMode === '2D') {
			Plotter2D.updateInfoPanel(filteredData);
		} else {
			// Hide stats container in 3D view
			const statsContainer = document.getElementById('dataStatsContainer');
			if (statsContainer) {
				statsContainer.style.display = 'none';
			}
		}
	}
	
	// Handle selection changes with debouncing
	function onSelectionsChange() {
		// Debounce rapid filter changes for better performance
		if (renderTimer) {
			clearTimeout(renderTimer);
		}
		
		renderTimer = setTimeout(async () => {
			try {
				if (viewMode === '2D') {
					// For large datasets, show loading during filtering
					if (Common.data.length > 50000) {
						Common.showLoading('Updating view...', 'Filtering data...');
					}
					await filterBySelections();
				}
				// Use requestAnimationFrame for smoother rendering
				requestAnimationFrame(() => {
					render();
				});
			} catch (e) {
				console.error(e);
				Common.hideLoading();
			}
		}, 50); // 50ms debounce
	}
	
	// Multi-select dropdown setup
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
			const label = checkboxes[0].closest('.multiselect-option').querySelector('label');
			button.querySelector('span:first-child').textContent = label ? label.textContent : checkboxes[0].value;
			if (countSpan) countSpan.textContent = '';
		} else {
			button.querySelector('span:first-child').textContent = 'Multiple Options Selected';
			if (countSpan) countSpan.textContent = `(${count})`;
		}
	}
	
	// Setup controls
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
	
	// SSE setup
	function setupSSE() {
		try {
			const es = new EventSource('/events');
			es.onopen = () => console.log('[SSE] connected');
			es.onmessage = async () => {
				console.log('[SSE] update');
				await Common.fetchResults(true, true); // Force refresh on SSE update, show loading
				if (viewMode === '2D') {
					await filterBySelections();
				}
				render();
				Common.hideLoading();
				// Update GPS view if active
				if (gpsViewActive && Plotter3D.cesiumViewer) {
					Plotter3D.renderGPSData();
				}
			};
			es.onerror = (e) => {
				console.warn('[SSE] error', e);
				startPollingFallback();
			};
		} catch {}
	}
	
	// Optimized polling - only check mtime, not full data
	async function pollOnce() {
		try {
			const resp = await fetch('/api/results?mtime_only=true', { cache: 'no-store' });
			if (!resp.ok) return;
			const payload = await resp.json();
			if (typeof payload.mtime !== 'undefined' && payload.mtime !== Common.lastMtime) {
				console.log('[POLL] change detected');
				// File changed, fetch full data
				await Common.fetchResults(true, true); // Force refresh, show loading
				if (viewMode === '2D') {
					await filterBySelections();
				}
				render();
				Common.hideLoading();
				// Update GPS view if active
				if (gpsViewActive && Plotter3D.cesiumViewer) {
					Plotter3D.renderGPSData();
				}
			}
		} catch {}
	}
	
	function startPollingFallback() {
		if (pollTimer) return;
		console.log('[POLL] starting fallback polling');
		pollTimer = setInterval(pollOnce, 2000);
	}
	
	// Switch between 2D and 3D view - batch DOM updates to avoid visual glitches
	window.switchView = function switchView(mode) {
		viewMode = mode;
		
		// Batch all DOM updates together using requestAnimationFrame
		requestAnimationFrame(() => {
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
				update3DControls();
			} else {
				update2DControls();
			}
			
			// Reset render state and re-render after controls are updated
			hasRendered = false;
			// Use another requestAnimationFrame to ensure DOM is fully updated before rendering
			requestAnimationFrame(() => {
				onSelectionsChange();
			});
		});
	};
	
	function update3DControls() {
		// Hide FSM labels button in 3D mode
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
			showKalman3D.onchange = () => {
				onSelectionsChange();
				if (gpsViewActive && Plotter3D.cesiumViewer) {
					Plotter3D.renderGPSData();
				}
			};
		}
		if (showGPS3D) {
			showGPS3D.onchange = () => {
				onSelectionsChange();
				if (gpsViewActive && Plotter3D.cesiumViewer) {
					Plotter3D.renderGPSData();
				}
			};
		}
		if (showBaro3D) {
			showBaro3D.onchange = onSelectionsChange;
		}
		
		// Set up event listeners for 3D FSM dropdowns
		const fsmFrom3D = document.getElementById('fsmFrom3D');
		const fsmTo3D = document.getElementById('fsmTo3D');
		if (fsmFrom3D) {
			fsmFrom3D.addEventListener('change', () => {
				onSelectionsChange();
				if (gpsViewActive && Plotter3D.cesiumViewer) {
					Plotter3D.renderGPSData();
				}
			});
		}
		if (fsmTo3D) {
			fsmTo3D.addEventListener('change', () => {
				onSelectionsChange();
				if (gpsViewActive && Plotter3D.cesiumViewer) {
					Plotter3D.renderGPSData();
				}
			});
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
	
	// Toggle FSM labels visibility
	window.toggleFSMLabels = function toggleFSMLabels() {
		showFSMLabels = !showFSMLabels;
		const btn = document.getElementById('toggleFSMLabels');
		btn.textContent = showFSMLabels ? 'Hide FSM Labels' : 'Show FSM Labels';
		onSelectionsChange();
	};
	
	// Toggle GPS View
	window.toggleGPSView = function toggleGPSView() {
		gpsViewActive = !gpsViewActive;
		const btn = document.getElementById('gpsViewBtn');
		const plotDiv = document.getElementById('plot');
		const mapDiv = document.getElementById('map');
		
		if (gpsViewActive) {
			btn.textContent = 'Plot View';
			const cesiumDiv = document.getElementById('cesiumContainer');
			if (plotDiv) plotDiv.style.display = 'none';
			if (mapDiv) mapDiv.style.display = 'none';
			if (cesiumDiv) {
				cesiumDiv.style.display = 'block';
				cesiumDiv.classList.add('gps-view-active');
			}
			Plotter3D.renderGPSView();
		} else {
			btn.textContent = 'Map view (work in progress)';
			const cesiumDiv = document.getElementById('cesiumContainer');
			if (plotDiv) plotDiv.style.display = 'block';
			if (mapDiv) mapDiv.style.display = 'none';
			if (cesiumDiv) {
				cesiumDiv.style.display = 'none';
				cesiumDiv.classList.remove('gps-view-active');
			}
			hasRendered = false;
			onSelectionsChange();
		}
	};
	
	// Reset view
	window.resetView = function resetView() {
		if (gpsViewActive && Plotter3D.cesiumViewer) {
			Plotter3D.resetCesiumView();
			return;
		}
		
		if (viewMode === '3D') {
			Plotly.relayout('plot', {
				'scene.camera': {
					eye: { x: 1.5, y: 1.5, z: 1.5 }
				}
			});
		} else {
			Plotly.relayout('plot', { 'xaxis.autorange': true, 'yaxis.autorange': true });
		}
	};
	
	// Boot function
	async function boot() {
		try {
			await Common.fetchResults(false, true); // Show loading indicator
			setupControls();
			update2DControls(); // Initialize controls for default 2D view
			await filterBySelections();
			render();
			Common.hideLoading();
			setupSSE();
			// Enable polling as a safety net
			setTimeout(() => {
				if (!Common.lastMtime) {
					startPollingFallback();
				}
			}, 3000);
		} catch (e) {
			console.error(e);
			Common.hideLoading();
			const plot = document.getElementById('plot');
			plot.innerHTML = '';
			const div = document.createElement('div');
			div.className = 'error';
			div.textContent = (e && e.message) ? e.message : 'Failed to load data';
			plot.appendChild(div);
		}
	}
	
	document.addEventListener('DOMContentLoaded', boot);
})();

