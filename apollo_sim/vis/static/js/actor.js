const actorCanvas = document.getElementById("actorCanvas");
const actorCtx = actorCanvas.getContext("2d");

// Get the container's width and height
const container = document.querySelector('.simulator-container');
const containerWidth = container.clientWidth;
const containerHeight = container.clientHeight;

// Set the canvas dimensions to match the container
actorCanvas.width = containerWidth;
actorCanvas.height = containerHeight;

let actors = null;

const ACTOR_TYPE = {
    "vehicle": { color: "rgb(79,108,243)" },
    "walker": { color: "rgb(245,177,8)" },
    "static": { color: "rgb(184,29,227)" },
    "bicycle": { color: "rgb(0,175,149)" },
};

const ROLE_TYPE = {
    "ads": { color: "rgb(245,82,82)" }
};


export function updateActors(newActors) {
    actors = newActors;
    drawActor();
}

// Function to draw actors
export function drawActor() {
    if (!actors) return;

    actorCtx.clearRect(0, 0, actorCanvas.width, actorCanvas.height);
    actorCtx.save();

    actorCtx.translate(offsetX, offsetY);
    actorCtx.scale(scale, scale);
    actorCtx.rotate(rotationAngle);

    actors.forEach(actor => {
        const polygon = actor.polygon;
        actorCtx.beginPath();

        // Calculate the bounding box of the polygon for centering the text
        let minX = Infinity;
        let minY = Infinity;
        let maxX = -Infinity;
        let maxY = -Infinity;

        polygon.forEach(([x, y], i) => {
            const normalizedX = x - min_x;
            const normalizedY = y - min_y;
            if (i === 0) actorCtx.moveTo(normalizedX, normalizedY);
            else actorCtx.lineTo(normalizedX, normalizedY);

            // Update bounding box
            minX = Math.min(minX, normalizedX);
            minY = Math.min(minY, normalizedY);
            maxX = Math.max(maxX, normalizedX);
            maxY = Math.max(maxY, normalizedY);
        });
        actorCtx.closePath();

        const actorStyle = actor.role === "ads"
            ? ROLE_TYPE[actor.role]
            : ACTOR_TYPE[actor.category.split(".")[0]];

        // console.log(actorStyle)
        actorCtx.fillStyle = actorStyle.color;
        actorCtx.fill();
        actorCtx.strokeStyle = "black";
        actorCtx.lineWidth = 0.03;
        actorCtx.stroke();

        // Calculate the center of the bounding box
        const centerX = (minX + maxX) / 2;
        const centerY = (minY + maxY) / 2;

        // Set font size based on bounding box height
        const fontSize = 0.6; // Adjust 4 as needed
        actorCtx.font = `${fontSize}px Arial`;
        actorCtx.textAlign = "center";
        actorCtx.textBaseline = "middle";

        // Draw the actor ID text at the center of the polygon
        actorCtx.fillStyle = "black";
        // Draw the actor ID on the first line
        actorCtx.fillText(`${actor.id}`, centerX, centerY - fontSize / 2 - 0.1);

        // Draw the speed on the second line, offsetting by the font size
        actorCtx.fillText(`${parseFloat(actor.speed).toFixed(2)} m/s`, centerX, centerY + fontSize / 2 + 0.1);
    });

    actorCtx.restore();
}
