//frontend/assets/websocket_handler.js


document.addEventListener('DOMContentLoaded', function() {
    const local_ip = window.location.hostname; 

    // Load pako for decompression
    const pakoScript = document.createElement('script');
    pakoScript.src = "https://cdnjs.cloudflare.com/ajax/libs/pako/2.1.0/pako.min.js";
    pakoScript.onload = function() {
        console.log("pako library loaded for decompression.");
        initializeWebSockets();
    };
    pakoScript.onerror = function() {
        console.error("Failed to load pako library.");
    };
    document.head.appendChild(pakoScript);

    function initializeWebSockets() {
        // WebSocket for SLAM Map
        const mapSocket = new WebSocket(`ws://${local_ip}:8000/ws/map_data`);

        // Initialize the full grid (assuming max size known)
        let fullGrid = [];
        let layoutInitialized = false;

        mapSocket.onmessage = function(event) {
            const mapData = JSON.parse(event.data);

            if (mapData && mapData.updateRegions && mapData.width && mapData.height) {
                const width = mapData.width;
                const height = mapData.height;

                // Initialize the grid if it's empty
                if (fullGrid.length === 0) {
                    for (let y = 0; y < height; y++) {
                        fullGrid.push(new Array(width).fill(-1));  // Default value
                    }
                }

                // Update only changed regions
                const updateRegions = mapData.updateRegions;  // Array of objects: {row, data: [updated row data]}
                updateRegions.forEach(region => {
                    const rowIndex = region.row;
                    const rowData = region.data;

                    // Update only the modified rows in the full grid
                    fullGrid[rowIndex] = rowData;
                });

                // Initialize layout and plot only once
                if (!layoutInitialized) {
                    Plotly.newPlot('map-graph', [{
                        z: fullGrid,
                        type: 'heatmapgl',
                        colorscale: 'Viridis',
                        zmin: -1,
                        zmax: 100
                    }], {
                        title: 'SLAM Map',
                        margin: { r: 0, t: 0, l: 0, b: 0 },
                        xaxis: { scaleanchor: 'y', scaleratio: 1 },
                        yaxis: { scaleanchor: 'x', scaleratio: 1 }
                    });
                    layoutInitialized = true;
                } else {
                    // Use `Plotly.update` instead of `Plotly.react`
                    Plotly.update('map-graph', {
                        z: [fullGrid]
                    });
                }
            }
        };

        mapSocket.onclose = function(event) {
            console.log('SLAM Map WebSocket closed:', event);
        };

        mapSocket.onerror = function(error) {
            console.error('SLAM Map WebSocket error:', error);
        };
       // WebSocket for Topics
       const topicsSocket = new WebSocket(`ws://${local_ip}:8000/ws/topics`);
       topicsSocket.onmessage = function(event) {
           const data = JSON.parse(event.data);
           const topics = data.topics || [];

           const dropdownMenu = document.getElementById('topics-dropdown-menu');

           // Ensure we're targeting the dropdown-menu div inside DropdownMenu
           const dropdownMenuDiv = dropdownMenu.querySelector('.dropdown-menu');
           if (dropdownMenuDiv) {
               // Clear existing items
               dropdownMenuDiv.innerHTML = '';

               if (topics.length === 0) {
                   const item = document.createElement('a');
                   item.className = 'dropdown-item';
                   item.textContent = 'No available topics';
                   item.href = '#';
                   dropdownMenuDiv.appendChild(item);
               } else {
                   topics.forEach(topic => {
                       const item = document.createElement('a');
                       item.className = 'dropdown-item';
                       item.textContent = `${topic.name} (${topic.type})`;
                       item.href = '#';
                       dropdownMenuDiv.appendChild(item);
                   });
               }
           } else {
               console.error('Dropdown menu container not found.');
           }
       };

       topicsSocket.onclose = function(event) {
           console.log('Topics WebSocket closed:', event);
           setTimeout(initializeWebSockets, 3000);  // Attempt to reconnect after 3 seconds
       };

       topicsSocket.onerror = function(error) {
           console.error('Topics WebSocket error:', error);
       };
        // WebSocket for Camera Feed
        const cameraSocket = new WebSocket(`ws://${local_ip}:8000/ws/camera_feed`);

        cameraSocket.onmessage = function(event) {
            const cameraFeed = document.getElementById("camera-feed");  
            if (cameraFeed) {
                cameraFeed.src = 'data:image/jpeg;base64,' + event.data;  // Update the image source with the frame
            } else {
                console.error('Element with id "camera-feed" not found.');
            }
        };

        cameraSocket.onclose = function(event) {
            console.log('Camera WebSocket closed:', event);
        };

        cameraSocket.onerror = function(error) {
            console.error('Camera WebSocket error:', error);
        };
    }});
