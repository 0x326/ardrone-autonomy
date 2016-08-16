// Extended Kalman Filter
// Learn more at https://en.wikipedia.org/wiki/Extended_Kalman_filter
// This file replaces StateEstimator.js, which is now not used

// Should this file output debugging information to `console`?
var shouldSendDebugInfo = true;

var sylvester    = require('sylvester');
var util         = require('util');

var Matrix       = sylvester.Matrix;
var Vector       = sylvester.Vector;

var geolib       = require('geolib');

module.exports = EKF;
function EKF(options) {
    'use strict';

    options = options || {};

    this._options     = options;

    this.reset();
}

EKF.prototype.state = function() {
    'use strict';
    return this._state;
};

EKF.prototype.confidence = function() {
    'use strict';
    return this._sigma;
};

EKF.prototype.reset = function() {
    'use strict';
    this._state       = this._options.state   || {x: 0, y: 0, yaw: 0, absoluteYaw: null};
    this._sigma       = Matrix.I(3);
    this._q           = Matrix.Diagonal([0.0003, 0.0003, 0.0001]);
    this._r           = Matrix.Diagonal([0.3, 0.3, 0.3]);
    this._last_yaw    = null;
    this._last_time   = null;
    this._last_gps_lat = null;
    this._last_gps_lon = null;
    this._last_gps_time = null;
};

EKF.prototype.predict = function(navdata) {
    'use strict';
    var doNotUpdateState = false;
    // LaTeX Math: \int (vx)dt = dx
    try {
        var pitch = navdata.demo.rotation.pitch.toRad(),
            roll  = navdata.demo.rotation.roll.toRad(),
            yaw   = normAngle((navdata.magneto.heading.fusionUnwrapped).toRad()),
            velocityX    = navdata.demo.velocity.x / 1000, // Convert milimeters/second to meters/second
            velocityY    = navdata.demo.velocity.y / 1000,
            currentTime     = navdata.time / 1000 // Convert miliseconds to seconds
        ;
    }
    catch (error) {
        if (error instanceof TypeError)
        {
            if(shouldSendDebugInfo) {
                console.warn("Error reading navdata");
            }
            return;
        }
        else {
            throw error;
        }
    }
    
    // Get GPS data if it is available
    if (typeof(navdata.gps) == "object" && navdata.gps !== null) {
        if (typeof(navdata.gps.latitude) == "number" && typeof(navdata.gps.longitude) == "number") {
            if (navdata.gps.latitude !== 0 || navdata.gps.longitude !== 0) {
                var gpsLat = navdata.gps.latitude,
                    gpsLon = navdata.gps.longitude,
                    hasGpsData = true
                ;
            }
        }
    }
    else {
        // GPS data unavailable
        var hasGpsData = false;
    }
       
    // We are not interested by the absolute yaw, but the yaw motion,
    // so we need at least a prior value to get started.
    if (this._last_yaw === null) {
        this._last_yaw = yaw;
        doNotUpdateState = true;
    }
    if (this._last_time === null) {
        this._last_time = currentTime;
        doNotUpdateState = true;
    }
    if (doNotUpdateState) {
        return;
    }
    
    var deltaT = currentTime - this._last_time;
    // We are not interested in the absolute yaw, but someone else might.
    // Save the absolute yaw in case
    this._state.absoluteYaw = yaw;
    
    // Compute the odometry by integrating the motion over delta_t
    var odometry = {deltaX: velocityX * deltaT, deltaY: velocityY * deltaT, deltaYaw: yaw - this._last_yaw};
    this._last_yaw  = yaw;

    // Update the state estimate
    var state = this._state;
    state.x   += Math.cos(state.yaw) * odometry.deltaX - Math.sin(state.yaw) * odometry.deltaY;
    state.y   += Math.sin(state.yaw) * odometry.deltaX + Math.cos(state.yaw) * odometry.deltaY;
    state.yaw += odometry.deltaYaw;
    
    // Normalize the yaw value
    state.yaw = normAngle(state.yaw);
    
    // Overwrite odometry estimate when there is GPS data (which yields more accurate data)
    if (hasGpsData) {
        if(shouldSendDebugInfo) {
            console.log("Analizing GPS data");
        }
        // We need a prior GPS value to calculate distance
        // If we don't have this, record the current GPS location for next time
        if (this._last_gps_lat === null || this._last_gps_lon === null || (new Date().valueOf() - this._last_gps_time.valueOf()) > 4000) {
            this._last_gps_lat = gpsLat;
            this._last_gps_lon = gpsLon;
            this._last_gps_time = new Date();
            return;
        }
        // Find displacement
        var displacement = geolib.getDistance({latitude: gpsLat,                longitude: gpsLon},
                                              {latitude: this._last_gps_lat,    longitude: this._last_gps_lon});
        var bearing = geolib.getCompassDirection({latitude: gpsLat,                longitude: gpsLon},
                                                 {latitude: this._last_gps_lat,    longitude: this._last_gps_lon});
        // Find X component of displacement
        var displacementXComponent = displacement * Math.cos(bearing);
        // Find Y component of displacement
        var displacmenetYComponent = displacement * Math.sin(bearing);
        
        // In order to detect whether the drone's yaw is correct.
        // Sometimes the drone rotates little by little and rotates its coordinate grid
        
        // Compare the GPS-based displacement to the estimate based displacement
        var estimatedDisplacement = $V([state.x, state.y]);
        var estimatedDisplacementAngle = estimatedDisplacement.angleFrom($V([1,0]));
        var angleError = estimatedDisplacementAngle - bearing;
        state.yaw -= angleError;
        
        // The former segment of code was just for angle calculation
        // Now we will overwrite the state calculation with the more accuate GPS data
        state.x = displacementXComponent;
        state.y = displacmenetYComponent;
        // Store GPS coordinates for the next calculation
        this._last_gps_lat = gpsLat;
        this._last_gps_lon = gpsLon;
        // Store current time so that the GPS point is not used after
        // it is not updated for a certain period of time, in the case of GPS signal loss
        this._last_gps_time = new Date();
    }

    // Compute the G term (due to the Taylor approximation to linearize the function),
    // where G is dimensionless magnetic moment (g-factor)
    var gFactor = $M(
           [[1, 0, -1 * Math.sin(state.yaw) * odometry.deltaX - Math.cos(state.yaw) * odometry.deltaY],
            [0, 1,  Math.cos(state.yaw) * odometry.deltaX - Math.sin(state.yaw) * odometry.deltaY],
            [0, 0, 1]]
            );

    // Compute the new sigma
    this._sigma = gFactor.multiply(this._sigma).multiply(gFactor.transpose()).add(this._q);
    
    // Store the current time for the next time
    this._last_time = currentTime;
};

// Only used when a tag is detected
 /*
  * measure.x:   x-position of marker in drone's xy-coordinate system (independant of roll, pitch)
  * measure.y:   y-position of marker in drone's xy-coordinate system (independant of roll, pitch)
  * measure.yaw: yaw rotation of marker, in drone's xy-coordinate system (independant of roll, pitch)
  *
  * pose.x:   x-position of marker in world-coordinate system
  * pose.y:   y-position of marker in world-coordinate system
  * pose.yaw: yaw-rotation of marker in world-coordinate system
  */
EKF.prototype.correct = function(measure, pose) {
    'use strict';
    // Compute expected measurement given our current state and the marker pose
    var state  = this._state;
    var psi    = state.yaw;
    this._s    = {x: state.x, y: state.y, yaw: state.yaw};

    // Normalized the measure yaw
    measure.yaw = normAngle(measure.yaw);
    this._m = {x: measure.x, y: measure.y, yaw: measure.yaw};

    var z1 = Math.cos(psi) * (pose.x - state.x) + Math.sin(psi) * (pose.y - state.y);
    var z2 = -1 * Math.sin(psi) * (pose.x - state.x) + Math.cos(psi) * (pose.y - state.y);
    var z3 = pose.yaw - psi;
    this._z = {x: z1, y: z2, yaw: z3};

    // Compute the error
    var e1 = measure.x - z1;
    var e2 = measure.y - z2;
    var e3 = measure.yaw - z3;
    this._e = {x: e1, y: e2, yaw: e3};

    // Compute the H term
    var H = $M([[ -Math.cos(psi), -Math.sin(psi), Math.sin(psi) * (state.x - pose.x) - Math.cos(psi) * (state.y - pose.y)],
                [  Math.sin(psi), -Math.cos(psi), Math.cos(psi) * (state.x - pose.x) + Math.sin(psi) * (state.y - pose.y)],
                [  0, 0, -1]]);

    // Compute the Kalman Gain
    var Ht = H.transpose();
    var K = this._sigma.multiply(Ht).multiply(H.multiply(this._sigma).multiply(Ht).add(this._r).inverse())

    // Correct the pose estimate
    var err = $V([e1, e2, e3]);
    var c = K . multiply(err);
    state.x = state.x + c.e(1);
    state.y = state.y + c.e(2);

//  TODO - This does not work, need more investigation.
//  In the meanwhile, we don't correct yaw based on observation.
//  state.yaw = state.yaw + c.e(3);

    this._sigma = Matrix.I(3).subtract(K.multiply(H)).multiply(this._sigma);
};

function normAngle(rad) {
    'use strict';
    while (rad >  Math.PI) { rad -= 2 * Math.PI;}
    while (rad < -Math.PI) { rad += 2 * Math.PI;}
    return rad;
}

/** Converts numeric degrees to radians */
if (typeof(Number.prototype.toRad) === "undefined") {
    Number.prototype.toRad = function() {
        'use strict';
        return this * Math.PI / 180;
    };
}

/** Converts radians to numeric dregrees */
if (typeof(Number.prototype.toDeg) === "undefined") {
    Number.prototype.toDeg = function() {
        'use strict';
        return this * 180 / Math.PI;
    };
}
