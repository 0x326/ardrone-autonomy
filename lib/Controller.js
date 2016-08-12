// Controller.js
'use strict';
// Should this file output debugging information to `console`?
var shouldSendDebugInfo = true;

var EventEmitter = require('events').EventEmitter;
var Timers  = require('timers');
var util    = require('util');
var PID     = require('./PID');
var EKF     = require('./EKF');
var Camera  = require('./Camera');

var EPS_LINEAR      = 0.05; // We are ok with 5 cm horizontal precision
var EPS_ALTITUDE      = 0.1; // We are ok with 10 cm altitude precision
var EPS_ANGULAR      = 0.1; // We are ok with 0.1 rad precision (5 deg)
var STABLE_DELAY = 200; // Time in ms to wait before declaring the drone on target

module.exports = Controller;
util.inherits(Controller, EventEmitter);
function Controller(client, options) {
    EventEmitter.call(this);

    options = options || {};

    // A ardrone client to pilot the drone
    this._client  = client;

    // The position of a roundel tag to detect
    this._tag     = options.tag || {x: 0, y: 0, yaw: 0};

    // Configure the four PID required to control the drone
    // Use user-defined PID values. If they are not defined, use default values
    options.pid = options.pid || {x_axis:   {p_constant: 0.5, i_constant: 0, d_constant: 0.35}, 
                                  y_axis:   {p_constant: 0.5, i_constant: 0, d_constant: 0.35}, 
                                  z_axis:   {p_constant: 0.8, i_constant: 0, d_constant: 0.35}, 
                                  yaw_axis: {p_constant: 1.0, i_constant: 0, d_constant: 0.30}}
    // PID controller for each axis: new PID( P constant, I constant, D constant )
    this._pid_x   = new PID(options.pid.x_axis.p_constant,
                            options.pid.x_axis.i_constant,
                            options.pid.x_axis.d_constant);
    this._pid_y   = new PID(options.pid.y_axis.p_constant, 
                            options.pid.y_axis.i_constant,
                            options.pid.y_axis.d_constant);
    this._pid_z   = new PID(options.pid.z_axis.p_constant,
                            options.pid.z_axis.i_constant,
                            options.pid.z_axis.d_constant);
    this._pid_yaw = new PID(options.pid.yaw_axis.p_constant, 
                            options.pid.yaw_axis.i_constant,
                            options.pid.yaw_axis.d_constant);

    // kalman filter is used for the drone state estimation
    this._ekf     = new EKF(options);

    // Used to process images and backproject them
    this._camera  = new Camera();

    // Control will only work if enabled
    this._enabled = false;

    // Ensure that we don't enter the processing loop twice
    this._busy = false;

    // The current target goal and an optional callback to trigger
    // when goal is reached
    this._goal     = null;
    this._callback = null;

    // The last known state
    this._state   = null,
    
    // The last known non-flight related drone info
    this._drone_state = null,

    // The last time we have reached the goal (all control commands = 0)
    this._last_ok = 0;

    // Register the listener on navdata for our control loop
    var self = this;
    client.on('navdata', function(navdata) {
        if (!this._busy && self._hasNecessaryData(navdata)) {
            this._busy = true;
            self._processNavdata(navdata);
            self._control(navdata);
            this._busy = false;
        }
    });
}

Controller.prototype._hasNecessaryData = function (navdata)
{
    if (typeof navdata.demo == "object" && typeof navdata.magneto == "object" && typeof navdata.magneto.heading.fusionUnwrapped != "undefined") {
        // navdata is good
        return true;
    }
    else {
        // navdata is not enough
        // Configure drone to send over all navdata
        this._client.config("general:navdata_demo", false);
        return false;
    }
};

/*
 * Enable auto-pilot. The controller will attempt to bring
 * the drone (and maintain it) to the goal.
 */
Controller.prototype.enable = function() {
    this._pid_x.reset();
    this._pid_y.reset();
    this._pid_z.reset();
    this._pid_yaw.reset();
    this._enabled = true;
};

/*
 * Disable auto-pilot. The controller will stop all actions
 * and send a stop command to the drone.
 */
Controller.prototype.disable = function() {
    this._enabled = false;
    this._client.stop();
}

/*
 * Return the drone state (x,y,z,yaw) as estimated
 * by the Kalman Filter.
 */
Controller.prototype.state = function() {
    return this._state;
}

/*
 * Sets the goal to the current state and attempt to hover on top.
 */
Controller.prototype.hover = function() {
    this.disable();
}

/*
 * Reset the kalman filter to its base state (default is x:0, y:0, yaw:0).
 *
 * This is especially usefull to set mark the drone position as the starting position
 * after takeoff. We must disable, to ensure that the zeroing does not trigger a sudden move
 * of the drone.
 */
Controller.prototype.zero = function() {
    this.disable();
    this._ekf.reset();
}

/*
 * Move forward (direction faced by the front camera) by the given
 * distance (in meters).
 */
Controller.prototype.forward = function(distance, callback) {
       // Our starting position
    var state = this.state();

    if (state == null)
        if (shouldSendDebugInfo)
            console.warn("Unable to go forward since current location is unknown");
    else {
        // Remap our target position in the world coordinates
        var gx = state.x + Math.cos(state.yaw) * distance;
        var gy = state.y + Math.sin(state.yaw) * distance;

        // Assign the new goal
        this._go({x: gx, y: gy, z: state.z, yaw: state.yaw}, callback);
    }
}

/*
 * Move backward by the given distance (in meters).
 */
Controller.prototype.backward = function(distance, callback) {
    return this.forward(-distance, callback);
}

/*
 * Move right (front being the direction faced by the front camera) by the given
 * distance (in meters).
 */
Controller.prototype.right = function(distance, callback) {
    // Our starting position
    var state = this.state();

    if (state == null)
        if (shouldSendDebugInfo)
            console.warn("Unable to go right since current location is unknown");
    else {
        // Remap our target position in the world coordinates
        var gx = state.x - Math.sin(state.yaw) * distance;
        var gy = state.y + Math.cos(state.yaw) * distance;

        // Assign the new goal
        this._go({x: gx, y: gy, z: state.z, yaw: state.yaw}, callback);
    }
}

/*
 * Move left by the given distance (in meters).
 */
Controller.prototype.left = function(distance, callback) {
    return this.right(-distance, callback);
}

/*
 * Turn clockwise of the given angle. Note that this does not
 * force a clockwise motion, if the angle is > 180 then the drone
 * will turn in the other direction, taking the shortest path.
 */
Controller.prototype.cw = function(angle, callback) {
    var state = this.state();
    var yaw   = state.yaw.toDeg() + angle;

    return this._go({x: state.x, y: state.y, z: state.z, yaw: yaw.toRad()}, callback);
}

/*
 * Turn counter clockwise of the given angle (in degrees)
 */
Controller.prototype.ccw = function(angle, callback) {
    return this.cw(-angle, callback);
}

/*
 * Climb ups by the given distance (in meters).
 */
Controller.prototype.up = function(distance, callback) {
    var state = this.state();
    return this._go({x: state.x, y: state.y, z: state.z + distance, yaw: state.yaw}, callback);
}

/*
 * Lower itself by the given distance (in meters).
 */
Controller.prototype.down = function(distance, callback) {
    return this.up(-distance, callback);
}

/*
 * Go to the target altitude
 */
Controller.prototype.altitude = function(altitude, callback) {
    var state = this.state();
    return this._go({x: state.x, y: state.y, z: altitude, yaw: state.yaw}, callback);
}

/*
 * Go to the target yaw (argument in degree)
 */
Controller.prototype.yaw = function(yaw, callback) {
    var state = this.state();
    return this._go({x: state.x, y: state.y, z: state.z, yaw: yaw.toRad()}, callback);
}

/*
 * Sets a new goal and enable the controller. When the goal
 * is reached, the callback is called with the current state.
 *
 * x,y,z in meters
 * yaw in degrees
 */
Controller.prototype.go = function(goal, callback) {
    if (goal.yaw != undefined) {
        goal.yaw = goal.yaw.toRad();
    }

    return this._go(goal, callback);
}

Controller.prototype._go = function(goal, callback) {
    // Since we are going to modify goal settings, we
    // disable the controller, just in case.
    this.disable();

    // If no goal given, assume an empty goal
    goal = goal || {};

    // Normalize the yaw, to make sure we don't spin 360deg for
    // nothing :-)
    if (goal.yaw != undefined) {
        var yaw = goal.yaw;
        while (yaw >  Math.PI) { yaw -= 2 * Math.PI;}
        while (yaw < -Math.PI) { yaw += 2 * Math.PI;}
        goal.yaw = yaw;
    }

    // Make sure we don't attempt to go too low
    if (goal.z != undefined) {
        goal.z = Math.max(goal.z, 0.5);
    }

    // Update our goal
    this._goal = goal;
    this._goal.reached = false;

    // Keep track of the callback to trigger when we reach the goal
    this._callback = callback;

    // Reenable the controller
    this.enable();
}

Controller.prototype._processNavdata = function(navdata) {
    // Save non-flight related drone info
    this._drone_state = navdata.droneState;
    
    // EKF prediction step
    this._ekf.predict(navdata);
    
    // If a tag is detected by the bottom camera, we attempt a correction step
    // This require prior configuration of the client to detect the oriented
    // roundel and to enable the vision detect in navdata.
    // TODO: Add documentation about this
    if (navdata.visionDetect && navdata.visionDetect.nbDetected > 0) {
        // Fetch detected tag position, size and orientation
        var xc = navdata.visionDetect.xc[0]
          , yc = navdata.visionDetect.yc[0]
          , wc = navdata.visionDetect.width[0]
          , hc = navdata.visionDetect.height[0]
          , yaw = navdata.visionDetect.orientationAngle[0]
          , dist = navdata.visionDetect.dist[0] / 100 // Need meters
          ;

        // Compute measure tag position (relative to drone) by
        // back-projecting the pixel position p(x,y) to the drone
        // coordinate system P(X,Y).
        // TODO: Should we use dist or the measure altitude ?
        var camera = this._camera.p2m(xc + wc/2, yc + hc/2, dist);

        // We convert this to the controller coordinate system
        var measured = {x: -1 * camera.y, y: camera.x};

        // Rotation is provided by the drone, we convert to radians
        measured.yaw = yaw.toRad();

        // Execute the EKS correction step
        this._ekf.correct(measured, this._tag);
    }

    // Keep a local copy of the state
    this._state = this._ekf.state();
    this._state.z = navdata.demo.altitude;
    this._state.vx = navdata.demo.velocity.x / 1000; //We want m/s instead of mm/s
    this._state.vy = navdata.demo.velocity.y / 1000;
}

Controller.prototype._control = function(navdata) {
    // Do not control if not enabled
    if (!this._enabled) return;

    // Do not control if no known state or no goal defines
    if (this._goal == null || this._state ==  null) return;

    // Compute error between current state and goal
    var xError   = (this._goal.x != undefined)   ? this._goal.x   - this._state.x   : 0
      , yError   = (this._goal.y != undefined)   ? this._goal.y   - this._state.y   : 0
      , zError   = (this._goal.z != undefined)   ? this._goal.z   - this._state.z   : 0
      , yawError = (this._goal.yaw != undefined) ? this._goal.yaw - this._state.yaw : 0
      ;

    // Normalize eyaw within [-180, 180]
    while(yawError < -Math.PI) yawError += (2 * Math.PI);
    while(yawError >  Math.PI) yawError -= (2 * Math.PI);

    // Check if we are within the target area
    if ((Math.abs(xError) < EPS_LINEAR) && (Math.abs(yError) < EPS_LINEAR) && (Math.abs(zError) < EPS_ALTITUDE) && (Math.abs(yawError) < EPS_ANGULAR)) {
        // Have we been here before ?
        if (!this._goal.reached && this._last_ok != 0) {
            // And for long enough ?
            if ((Date.now() - this._last_ok) > STABLE_DELAY) {
                // Mark the goal has reached
                this._goal.reached = true;

                // We schedule the callback in the near future. This is to make
                // sure we finish all our work before the callback is called.
                if (this._callback != null) {
                    setTimeout(this._callback, 10);
                    this._callback = null;
                }

                // Emit a state reached
                this.emit('goalReached', this._state);
            }
        } else {
            this._last_ok = Date.now();
        }
    }
    // If we just left the goal, we notify
    else if (this._last_ok != 0) {
        // Reset last ok since we are in motion
        this._last_ok = 0;
        this._goal.reached = false;
        this.emit('goalLeft', this._state);
    }

    // Get Raw command from PID
    var xCommand = this._pid_x.getCommand(xError);
    var yCommand = this._pid_y.getCommand(yError);
    var zCommand = this._pid_z.getCommand(zError);
    var yawCommand = this._pid_yaw.getCommand(yawError);

    // Map the commands to drone orientation and normalize them
    // The drone only accepts directional speeds within [-1 units, 1 units]
    var yaw  = this._state.yaw;
    var normalizedXCommand   = within(Math.cos(yaw) * xCommand + Math.sin(yaw) * yCommand, -1, 1);
    var normalizedYCommand   = within(-Math.sin(yaw) * xCommand + Math.cos(yaw) * yCommand, -1, 1);
    var normalizedZCommand   = within(zCommand, -1, 1);
    var normalizedYawCommand = within(yawCommand, -1, 1);

    // Emit the control data for auditing
    this.emit('controlData', {
        state:   this._state,
        goal:    this._goal,
        error:   {ex: xError, ey: yError, ez: zError, eyaw: yawError},
        control: {ux: xCommand, uy: yCommand, uz: zCommand, uyaw: yawCommand},
        last_ok: this._last_ok,
        tag:     (navdata.visionDetect && navdata.visionDetect.nbDetected > 0) ? 1 : 0
    });

    // Check to see if commands are ok. For example, the drone will not go up if it is commanded soley to go up
    // the drone must be in motion
    /*if (Math.abs(normalizedZCommand) > 0.01 && !(Math.abs(normalizedXCommand) > 0.01 || Math.abs(normalizedYCommand) > 0.01 || Math.abs(normalizedYawCommand) > 0.01)) {
        // The drone needs to go up but there is no other motion
        // Give the drone a rotational command
        normalizedYawCommand = 0.1;
    }*/
    
    // Send commands to drone if the are significant
    if (Math.abs(normalizedXCommand) > 0.01) this._client.front(normalizedXCommand);
    if (Math.abs(normalizedYCommand) > 0.01) this._client.right(normalizedYCommand);
    if (Math.abs(normalizedZCommand) > 0.01) this._client.up(normalizedZCommand);
    if (Math.abs(normalizedYawCommand) > 0.01) this._client.clockwise(normalizedYawCommand);

}

function within(x, min, max) {
    if (x < min) {
        return min;
    } else if (x > max) {
        return max;
    } else {
        return x;
    }
}
