import { updateActors } from './actor.js';
import { updateTrafficLights } from './traffic_light.js';
import { drawAll } from './entire.js';
import { updateInfo } from "./info.js";

const socket = io();

socket.on('connect', () => {
    console.log("Connected to the server via SocketIO");
});

socket.on('update', function(data) {
    // Dispatch data to each module
    updateInfo(data.map_name, data.frame, data.game_time, data.real_time);
    updateActors(data.actors);
    updateTrafficLights(data.traffic_lights);
    drawAll();
});
