const minScale = 1;
const maxScale = 50;
const rotationStep = 0.002; // Adjust rotation sensitivity

export function addCanvasListeners(canvas, drawCallback) {

    const initialScale = scale;
    const initialOffsetX = offsetX;
    const initialOffsetY = offsetY;
    const initialRotationAngle = rotationAngle;

    let isDragging = false;
    let isRotating = false;
    let startX, startY;

    // Zoom controls with non-passive wheel event listener
    canvas.addEventListener("wheel", function(e) {
        e.preventDefault();

        // Get the position of the mouse relative to the canvas
        const mouseX = e.clientX;
        const mouseY = e.clientY;

        // Adjust scaleStep dynamically based on current scale
        const dynamicScaleStep = scale * 0.05;  // Adjust the multiplier (e.g., 0.05) for preferred speed

        // Determine the scale change based on scroll direction
        const newScale = e.deltaY < 0
            ? Math.min(scale + dynamicScaleStep, maxScale)
            : Math.max(scale - dynamicScaleStep, minScale);

        // Calculate the scale ratio
        const scaleRatio = newScale / scale;

        // Adjust the offset so the zoom centers on the mouse position
        offsetX = mouseX - scaleRatio * (mouseX - offsetX);
        offsetY = mouseY - scaleRatio * (mouseY - offsetY);

        // Update the scale
        scale = newScale;

        drawCallback();
    }, { passive: false });

    canvas.addEventListener("contextmenu", e => e.preventDefault());

    canvas.addEventListener("mousedown", function (e) {
        e.preventDefault();

        if (e.button === 0) { // Left-click for panning
            isDragging = true;
            startX = e.clientX - offsetX;
            startY = e.clientY - offsetY;
        } else if (e.button === 2) { // Right-click for rotation
            isRotating = true;
            startX = e.clientX;
            startY = e.clientY;
        }
    });

    canvas.addEventListener("mousemove", function (e) {
        if (isDragging) {
            offsetX = e.clientX - startX;
            offsetY = e.clientY - startY;

            drawCallback();
        } else if (isRotating) {
            const dx = e.clientX - startX;
            const dy = e.clientY - startY;
            const angleDelta = -dx * rotationStep;

            rotationAngle += angleDelta;
            startX = e.clientX;
            startY = e.clientY;

            drawCallback();
        }
    });

    canvas.addEventListener("mouseup", () => {
        isDragging = false;
        isRotating = false;
    });

    canvas.addEventListener("mouseleave", () => {
        isDragging = false;
        isRotating = false;
    });

    // Reset function to restore the initial state
    function resetView() {
        scale = initialScale;
        offsetX = initialOffsetX;
        offsetY = initialOffsetY;
        rotationAngle = initialRotationAngle;
        isDragging = false;
        isRotating = false;

        drawCallback(); // Redraw with initial state
    }

    // Attach resetView function to the reset button
    document.getElementById('resetViewButton').addEventListener('click', resetView);
}
