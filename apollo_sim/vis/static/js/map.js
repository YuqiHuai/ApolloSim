// Canvas setup
const mapCanvas = document.getElementById("mapCanvas");
const mapCtx = mapCanvas.getContext("2d");

// Get the container's width and height
const container = document.querySelector('.simulator-container');
const containerWidth = container.clientWidth;
const containerHeight = container.clientHeight;

// Set the canvas dimensions to match the container
mapCanvas.width = containerWidth;
mapCanvas.height = containerHeight;

// Boundary types with colors and styles
const BOUNDARY_TYPE = {
    "UNKNOWN": { color: "rgb(208,208,208)", style: "solid" },
    "DOTTED_YELLOW": { color: "rgb(238,191,0)", style: "dotted" },
    "DOTTED_WHITE": { color: "rgb(255, 255, 255)", style: "dotted" },
    "SOLID_YELLOW": { color: "rgb(238,191,0)", style: "solid" },
    "SOLID_WHITE": { color: "rgb(255, 255, 255)", style: "solid" },
    "DOUBLE_YELLOW": { color: "rgb(238,191,0)", style: "double" },
    "CURB": { color: "rgb(119,119,119)", style: "solid" }
};

// Function to draw the map
export function drawMap() {
    // console.log("drawMap: min_x, min_y, max_x, max_y", min_x, min_y, max_x, max_y);
    mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    mapCtx.save();

    mapCtx.translate(offsetX, offsetY);
    mapCtx.scale(scale, scale);
    mapCtx.rotate(rotationAngle);

    mapData.lanes.forEach(lane => {
        drawLane(lane);
        drawBoundary(lane, "left_boundary", lane.left_boundary_type);
        drawBoundary(lane, "right_boundary", lane.right_boundary_type);
    });

    mapCtx.restore();
}

function drawLane(lane) {
    const polygon = lane.polygon;
    mapCtx.beginPath();
    polygon.forEach(([x, y], i) => {
        const normalizedX = x - min_x;
        const normalizedY = y - min_y;
        if (i === 0) mapCtx.moveTo(normalizedX, normalizedY);
        else mapCtx.lineTo(normalizedX, normalizedY);
    });
    mapCtx.closePath();
    mapCtx.fillStyle = "rgba(180,180,180,0.8)";
    mapCtx.fill();
}

function drawBoundary(lane, boundaryKey, boundaryTypeKey) {
    const boundary = lane[boundaryKey];
    const boundaryStyle = BOUNDARY_TYPE[boundaryTypeKey];

    if (!boundary || boundary.length === 0) return;

    mapCtx.strokeStyle = boundaryStyle.color;
    mapCtx.lineWidth = boundaryStyle.style === "double" ? 0.2 : 0.2;

    if (boundaryStyle.style === "dotted") {
        mapCtx.setLineDash([5, 5]);
    } else {
        mapCtx.setLineDash([]);
    }

    mapCtx.beginPath();
    boundary.forEach(([x, y], index) => {
        const normalizedX = x - min_x;
        const normalizedY = y - min_y;
        if (index === 0) mapCtx.moveTo(normalizedX, normalizedY);
        else mapCtx.lineTo(normalizedX, normalizedY);
    });
    mapCtx.stroke();
}
