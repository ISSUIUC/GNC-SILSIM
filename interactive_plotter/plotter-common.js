// Common utilities, constants, and data management for the interactive plotter
// Shared between 2D and 3D views

(function() {
	'use strict';
	
	// Expose shared state and functions to window for other modules
	window.PlotterCommon = {
		// Constants
		fsmOrder: [
			'STATE_IDLE', 'STATE_FIRST_BOOST', 'STATE_BURNOUT', 'STATE_COAST',
			'STATE_APOGEE', 'STATE_DROGUE_DEPLOY', 'STATE_DROGUE', 'STATE_MAIN_DEPLOY',
			'STATE_MAIN', 'STATE_LANDED', 'STATE_SUSTAINER_IGNITION', 'STATE_SECOND_BOOST',
			'STATE_FIRST_SEPARATION'
		],
		
		dataLabels: {
			pos_x: 'Position X', pos_y: 'Position Y', pos_z: 'Position Z',
			vel_x: 'Velocity X', vel_y: 'Velocity Y', vel_z: 'Velocity Z',
			acc_x: 'Acceleration X', acc_y: 'Acceleration Y', acc_z: 'Acceleration Z',
			raw_gps_latitude: 'Raw GPS Latitude', raw_gps_longitude: 'Raw GPS Longitude', raw_gps_altitude: 'Raw GPS Altitude',
			raw_gps_x: 'GPS X (from Raw GPS Altitude)', raw_gps_y: 'GPS Y (from Raw GPS Longitude)', raw_gps_z: 'GPS Z (from Raw GPS Latitude)',
			raw_baro_alt: 'Raw Barometer Altitude',
			raw_highg_ax: 'Raw HighG X', raw_highg_ay: 'Raw HighG Y', raw_highg_az: 'Raw HighG Z'
		},
		
		traceColors: [
			'#667eea', '#e74c3c', '#2ecc71', '#f39c12', '#9b59b6', 
			'#1abc9c', '#e67e22', '#3498db', '#95a5a6', '#34495e'
		],
		
		METERS_PER_DEG_LAT: 111320, // meters per degree latitude (constant)
		
		// State variables
		data: [],
		filteredData: [],
		lastMtime: null,
		cachedPayload: null, // Cache the full API response
		fsmIndexMap: {},
		
		// GPS conversion constants
		LAUNCH_LAT: null,
		LAUNCH_LON: null,
		LAUNCH_LAT_DEG: null,
		LAUNCH_LON_DEG: null,
		GROUND_ALT: null,
		
		// Initialize FSM index map
		initFSMIndexMap: function() {
			this.fsmIndexMap = {};
			this.fsmOrder.forEach((state, idx) => {
				this.fsmIndexMap[state] = idx;
			});
		},
		
		// Initialize launch coordinates from first valid GPS data point
		initializeLaunchCoordinates: function() {
			if (!this.data || this.data.length === 0) return;
			
			// Find first row with valid GPS data
			for (let i = 0; i < this.data.length; i++) {
				const row = this.data[i];
				const lat = row.raw_gps_latitude;
				const lon = row.raw_gps_longitude;
				const alt = row.raw_gps_altitude;
				
				if (lat !== undefined && lon !== undefined && alt !== undefined &&
				    !Number.isNaN(lat) && !Number.isNaN(lon) && !Number.isNaN(alt) &&
				    lat !== 'nan' && lon !== 'nan' && alt !== 'nan') {
					// Store launch coordinates in microdegrees
					this.LAUNCH_LAT = lat;
					this.LAUNCH_LON = lon;
					this.GROUND_ALT = alt;
					
					// Convert to degrees for calculations
					this.LAUNCH_LAT_DEG = lat / 1e7;
					this.LAUNCH_LON_DEG = lon / 1e7;
					
					console.log(`[GPS] Launch coordinates initialized: lat=${this.LAUNCH_LAT_DEG.toFixed(6)}, lon=${this.LAUNCH_LON_DEG.toFixed(6)}, alt=${this.GROUND_ALT.toFixed(2)}m`);
					return;
				}
			}
		},
		
		// Convert GPS coordinates to local x, y, z coordinates
		convertGPSToLocal: function(lat, lon, alt) {
			if (lat === undefined || lon === undefined || alt === undefined ||
			    Number.isNaN(lat) || Number.isNaN(lon) || Number.isNaN(alt) ||
			    this.LAUNCH_LAT === null || this.LAUNCH_LON === null || this.GROUND_ALT === null) {
				return null;
			}
			
			// Convert from microdegrees to degrees
			const latDeg = lat / 1e7;
			const lonDeg = lon / 1e7;
			
			// Calculate meters per degree longitude at launch latitude
			const metersPerDegLon = this.METERS_PER_DEG_LAT * Math.cos(this.LAUNCH_LAT_DEG * Math.PI / 180);
			
			// Convert to local coordinates
			const x = (lonDeg - this.LAUNCH_LON_DEG) * metersPerDegLon;
			const y = (latDeg - this.LAUNCH_LAT_DEG) * this.METERS_PER_DEG_LAT;
			const z = alt - this.GROUND_ALT;
			
			return { x, y, z };
		},
		
		// Convert local coordinates to GPS coordinates
		convertLocalToGPS: function(pos_x, pos_y, pos_z) {
			if (pos_x === undefined || pos_y === undefined || pos_z === undefined ||
			    Number.isNaN(pos_x) || Number.isNaN(pos_y) || Number.isNaN(pos_z) ||
			    this.LAUNCH_LAT === null || this.LAUNCH_LON === null || this.GROUND_ALT === null) {
				return null;
			}
			
			const metersPerDegLon = this.METERS_PER_DEG_LAT * Math.cos(this.LAUNCH_LAT_DEG * Math.PI / 180);
			const lonDeg = this.LAUNCH_LON_DEG + pos_y / metersPerDegLon;
			const latDeg = this.LAUNCH_LAT_DEG + pos_z / this.METERS_PER_DEG_LAT;
			const alt = pos_x + this.GROUND_ALT;
			
			return { lat: latDeg, lon: lonDeg, alt: alt };
		},
		
		// Show loading indicator
		showLoading: function(message = 'Loading data...', progress = '') {
			const overlay = document.getElementById('loadingOverlay');
			const text = overlay?.querySelector('.loading-text');
			const progressEl = document.getElementById('loadingProgress');
			if (overlay) overlay.style.display = 'flex';
			if (text) text.textContent = message;
			if (progressEl) progressEl.textContent = progress;
		},
		
		// Hide loading indicator
		hideLoading: function() {
			const overlay = document.getElementById('loadingOverlay');
			if (overlay) overlay.style.display = 'none';
		},
		
		// Optimized fetch with caching - only fetch if mtime changed
		fetchResults: async function(forceRefresh = false, showLoadingIndicator = true) {
			// If we have cached data and mtime hasn't changed, return cached data
			if (!forceRefresh && this.cachedPayload && this.lastMtime !== null) {
				// Check if file has changed by fetching just the mtime
				try {
					const resp = await fetch('/api/results?mtime_only=true', { cache: 'no-store' });
					if (resp.ok) {
						const payload = await resp.json();
						if (payload.mtime === this.lastMtime) {
							// No change, use cached data
							this.data = this.cachedPayload.rows || [];
							return;
						}
					}
				} catch (e) {
					// If mtime check fails, fall back to full fetch
					console.warn('[API] mtime check failed, fetching full data');
				}
			}
			
			if (showLoadingIndicator) {
				this.showLoading('Fetching data from server...', '');
			}
			
			console.log('[API] GET /api/results');
			const resp = await fetch('/api/results', { cache: 'no-store' });
			if (!resp.ok) {
				if (showLoadingIndicator) this.hideLoading();
				throw new Error('Failed to fetch results');
			}
			
			if (showLoadingIndicator) {
				this.showLoading('Processing data...', 'Parsing CSV data...');
			}
			
			const payload = await resp.json();
			
			// Cache the payload
			this.cachedPayload = payload;
			this.data = payload.rows || [];
			
			if (payload && typeof payload.mtime !== 'undefined') {
				this.lastMtime = payload.mtime;
			}
			
			console.log(`[API] rows=${this.data.length}`);
			if (!Array.isArray(this.data) || this.data.length === 0) {
				if (showLoadingIndicator) this.hideLoading();
				throw new Error('No data found');
			}
			
			if (showLoadingIndicator && this.data.length > 50000) {
				this.showLoading('Processing large dataset...', `Loaded ${this.data.length.toLocaleString()} data points`);
			}
			
			// Initialize launch coordinates from first GPS data point
			this.initializeLaunchCoordinates();
		},
		
		// Helper function to format FSM state names for display
		formatFSMLabel: function(fsmState) {
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
		},
		
		// Get Y-axis label based on data type
		getYAxisLabel: function(dataType) {
			if (dataType.includes('pos') || dataType.includes('altitude')) return 'Position (m)';
			if (dataType.includes('vel')) return 'Velocity (m/s)';
			if (dataType.includes('acc') || dataType.includes('highg')) return 'Acceleration (m/s²)';
			return 'Value';
		}
	};
	
	// Initialize FSM index map
	window.PlotterCommon.initFSMIndexMap();
})();

