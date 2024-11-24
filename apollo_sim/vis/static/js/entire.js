import { addCanvasListeners } from './listener.js';
import { drawMap } from './map.js';
import { drawTrafficLight } from './traffic_light.js';
import { drawActor } from './actor.js';
import { drawInfo } from './info.js';

export function drawAll() {
    drawMap();
    drawTrafficLight();
    drawActor();
    drawInfo();
}

// Initialize canvases' sizes on load and resize
export function resizeCanvas() {
    const width = window.innerWidth;
    const height = window.innerHeight;
    document.getElementById("mapCanvas").width = width;
    document.getElementById("mapCanvas").height = height;
    document.getElementById("trafficLightCanvas").width = width;
    document.getElementById("trafficLightCanvas").height = height;
    document.getElementById("actorCanvas").width = width;
    document.getElementById("actorCanvas").height = height;
    drawAll();
}

addCanvasListeners(mapCanvas, drawAll);
addCanvasListeners(actorCanvas, drawAll);
addCanvasListeners(trafficLightCanvas, drawAll);

window.onload = resizeCanvas;
window.onresize = resizeCanvas;