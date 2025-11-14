// 3D Plotting and GPS View functionality for the interactive plotter

(function() {
	'use strict';
	
	if (!window.PlotterCommon) {
		console.error('[3D Plotter] PlotterCommon not loaded');
		return;
	}
	
	const Common = window.PlotterCommon;
	
	// State variables for 3D/GPS view
	let cesiumViewer = null;
	let cesiumEntityCollection = null;
	
	// Expose 3D plotter functions
	window.Plotter3D = {
		cesiumViewer: null, // Will be set when initialized
		cesiumEntityCollection: null, // Will be set when initialized
		
		// Render 3D plot
		render: function(hasRendered) {
			const showKalman = document.getElementById('showKalman3D').checked;
			const showGPS = document.getElementById('showGPS3D').checked;
			const showBaro = document.getElementById('showBaro3D').checked;
			// Use 3D-specific FSM dropdowns if they exist, otherwise fall back to 2D dropdowns
			const from = document.getElementById('fsmFrom3D') ? document.getElementById('fsmFrom3D').value : document.getElementById('fsmFrom').value;
			const to = document.getElementById('fsmTo3D') ? document.getElementById('fsmTo3D').value : document.getElementById('fsmTo').value;
			const fromIdx = Common.fsmIndexMap[from] ?? -1;
			const toIdx = Common.fsmIndexMap[to] ?? -1;

			const traces = [];
			
			// Render Kalman position data (always uses pos_x, pos_y, pos_z)
			if (showKalman) {
				const xVals = [];
				const yVals = [];
				const zVals = [];
				const timeVals = [];
				
				// Filter Kalman data using 3D FSM dropdowns
				for (let i = 0; i < Common.data.length; i++) {
					const r = Common.data[i];
					
					// Check FSM range
					if (r.fsm && r.fsm !== 'nan') {
						const idx = Common.fsmIndexMap[r.fsm];
						if (idx === undefined || idx < fromIdx || idx > toIdx) continue;
					}
					
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
						line: { color: Common.traceColors[0], width: 4 },
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
				
				for (let i = 0; i < Common.data.length; i++) {
					const r = Common.data[i];
					
					// Check FSM range
					if (r.fsm && r.fsm !== 'nan') {
						const idx = Common.fsmIndexMap[r.fsm];
						if (idx === undefined || idx < fromIdx || idx > toIdx) continue;
					}
					
					const local = Common.convertGPSToLocal(
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
						line: { color: Common.traceColors[1], width: 2, dash: 'dash' },
						marker: { size: 3, color: Common.traceColors[1], opacity: 0.7 }
					});
				}
			}
			
			// Render barometer altitude as vertical axis (if selected)
			if (showBaro) {
				const xVals = [];
				const yVals = [];
				const zVals = [];
				const timeVals = [];
				
				// Filter barometer data using 3D FSM dropdowns
				for (let i = 0; i < Common.data.length; i++) {
					const r = Common.data[i];
					
					// Check FSM range
					if (r.fsm && r.fsm !== 'nan') {
						const idx = Common.fsmIndexMap[r.fsm];
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
						line: { color: Common.traceColors[2], width: 2, dash: 'dot' },
						marker: { size: 3, color: Common.traceColors[2], opacity: 0.7 }
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
			}
		},
		
		// Initialize Cesium viewer
		initCesium: function() {
			if (cesiumViewer) return; // Already initialized
			
			if (Common.LAUNCH_LAT_DEG === null || Common.LAUNCH_LON_DEG === null) {
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
				cesiumViewer.imageryLayers.addImageryProvider(
					new Cesium.OpenStreetMapImageryProvider({
						url: 'https://a.tile.openstreetmap.org/'
					})
				);
				
				// Set initial camera position looking down at the launch site
				cesiumViewer.camera.setView({
					destination: Cesium.Cartesian3.fromDegrees(
						Common.LAUNCH_LON_DEG,
						Common.LAUNCH_LAT_DEG,
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
				this.cesiumViewer = cesiumViewer;
				this.cesiumEntityCollection = cesiumEntityCollection;
				
				console.log('[GPS View] Cesium initialized successfully');
			} catch (error) {
				console.error('[GPS View] Error initializing Cesium:', error);
				cesiumViewer = null;
			}
		},
		
		// Render GPS view (Cesium globe)
		renderGPSView: function() {
			if (!Common.data || Common.data.length === 0) return;
			
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
					this.initCesium();
					if (cesiumViewer) {
						this.renderGPSData();
					}
				}, 100);
				return;
			}
			
			this.renderGPSData();
		},
		
		// Render GPS data on Cesium globe
		renderGPSData: function() {
			if (!cesiumViewer || !Common.data || Common.data.length === 0) return;
			
			// Temporarily disabled Kalman data on map view due to crashes - only show GPS
			const showKalman = false; // Disabled for now
			const showGPS = document.getElementById('showGPS3D').checked;
			// Use 3D-specific FSM dropdowns if they exist, otherwise fall back to 2D dropdowns
			const from = document.getElementById('fsmFrom3D') ? document.getElementById('fsmFrom3D').value : document.getElementById('fsmFrom').value;
			const to = document.getElementById('fsmTo3D') ? document.getElementById('fsmTo3D').value : document.getElementById('fsmTo').value;
			const fromIdx = Common.fsmIndexMap[from] ?? -1;
			const toIdx = Common.fsmIndexMap[to] ?? -1;
			
			// Clear existing entities
			cesiumEntityCollection.removeAll();
			
			let allPositions = []; // For bounding sphere calculation
			let lastValidGPSPoint = null;
			
			// Collect GPS points with altitude
			if (showGPS) {
				const gpsPoints = [];
				let minAlt = Infinity;
				let maxAlt = -Infinity;
				
				for (let i = 0; i < Common.data.length; i++) {
					const r = Common.data[i];
					
					// Check FSM range
					if (r.fsm && r.fsm !== 'nan') {
						const idx = Common.fsmIndexMap[r.fsm];
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
						lastValidGPSPoint = [latDeg, lonDeg, alt, r.timestamp];
						
						if (alt < minAlt) minAlt = alt;
						if (alt > maxAlt) maxAlt = alt;
					}
				}
				
				// Render GPS trajectory
				if (gpsPoints.length > 1) {
					const positions = [];
					const colors = [];
					
					for (let i = 0; i < gpsPoints.length; i++) {
						const [latDeg, lonDeg, alt, timestamp] = gpsPoints[i];
						
						// Convert to Cesium Cartesian3 (longitude, latitude, height)
						const pos = Cesium.Cartesian3.fromDegrees(lonDeg, latDeg, alt);
						positions.push(pos);
						allPositions.push(pos);
						
						// Color based on altitude
						const altRange = maxAlt - minAlt;
						const normAlt = altRange > 0 ? (alt - minAlt) / altRange : 0.5;
						const color = this.getAltitudeColorRGB(normAlt);
						colors.push(
							Cesium.Color.fromBytes(color.r, color.g, color.b, 255)
						);
					}
					
					// Create polyline segments with color-coded altitude
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
			}
			
			// Add launch site marker
			cesiumEntityCollection.add({
				position: Cesium.Cartesian3.fromDegrees(Common.LAUNCH_LON_DEG, Common.LAUNCH_LAT_DEG, Common.GROUND_ALT),
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
			if (lastValidGPSPoint) {
				const [endLat, endLon, endAlt] = lastValidGPSPoint;
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
						text: 'GPS End',
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
			if (allPositions.length > 0) {
				const boundingSphere = Cesium.BoundingSphere.fromPoints(allPositions);
				const height = Math.max(boundingSphere.radius * 2, 500);
				cesiumViewer.camera.flyToBoundingSphere(boundingSphere, {
					duration: 2.0,
					offset: new Cesium.HeadingPitchRange(
						0,
						Cesium.Math.toRadians(-45),
						height
					)
				});
			}
			
			// Ensure Cesium container is visible and resized
			const cesiumDiv = document.getElementById('cesiumContainer');
			if (cesiumDiv && cesiumViewer) {
				cesiumDiv.style.display = 'block';
				setTimeout(() => {
					if (cesiumViewer) {
						cesiumViewer.resize();
					}
				}, 100);
			}
		},
		
		// Helper function to get RGB color from normalized altitude
		getAltitudeColorRGB: function(normalizedAlt) {
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
		},
		
		// Reset Cesium camera view
		resetCesiumView: function() {
			if (!cesiumViewer || !cesiumEntityCollection) return;
			
			if (cesiumEntityCollection.values.length > 0) {
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
		}
	};
})();

