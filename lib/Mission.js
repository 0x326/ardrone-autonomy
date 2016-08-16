// Mission.js

// Should this file output debugging information to `console`?
var shouldSendDebugInfo = true;

var async = require('async'),
    fs    = require('fs')
  ;

module.exports = Mission;
function Mission(client, controller, options) {
    'use strict';

    options = options || {};

    this._options   = options;
    this._client    = client;
    this._control   = controller;

    this._steps     = [];
}

Mission.prototype.client = function() {
    'use strict';
    return this._client;
};

Mission.prototype.control = function() {
    'use strict';
    return this._control;
};

Mission.prototype.run = function(callback) {
    'use strict';
    this._abortMission = false;
    var self = this;
    async.waterfall(this._steps, function (error, results) {
        // Clean the buffer of commands (this._steps)
        self._steps = [];
        callback(error, results);
    });
};

Mission.prototype.abort = function () {
    'use strict';
    this._abortMission = true;
    if (this._control._goal !== null) {
        this._control._goal.reached = true;
    }
    this._client.stop();
};

Mission.prototype.log = function(path) {
    'use strict';
    var dataStream = fs.createWriteStream(path);
    var ekf = this._control._ekf;
    
    // For a proper CSV file, list the headers of the data once
    dataStream.write("controlData.state.x,controlData.state.y,controlData.state.z,controlData.state.yaw," + 
                    + "controlData.state.vx,controlData.state.vy," 
                    + "controlData.goal.x,controlData.goal.y,controlData.goal.z,controlData.goal.yaw,"
                    + "controlData.error.ex,controlData.error.ey,controlData.error.ez,controlData.error.eyaw,"
                    + "controlData.control.ux,controlData.control.uy,controlData.control.uz,controlData.control.uyaw,"
                    + "controlData.last_ok,controlData.tag,"
                    + "ekf._s.x,ekf._s.y,ekf._s.ekf._yaw.toDeg(),"
                    + "ekf._m.x,ekf._m.y,ekf._m.yaw.toDeg(),"
                    + "ekf._z.x,ekf._z.y,ekf._z.yaw.toDeg(),"
                    + "ekf._e.x,ekf._e.y,ekf._e.yaw.toDeg()"
                    + "\n");
    
    this._control.on('controlData', function(d) {
        var log = (d.state.x + "," +
                   d.state.y + "," +
                   d.state.z + "," +
                   d.state.yaw + "," +
                   d.state.vx + "," +
                   d.state.vy + "," +
                   d.goal.x + "," +
                   d.goal.y + "," +
                   d.goal.z + "," +
                   d.goal.yaw + "," +
                   d.error.ex + "," +
                   d.error.ey + "," +
                   d.error.ez + "," +
                   d.error.eyaw + "," +
                   d.control.ux + "," +
                   d.control.uy + "," +
                   d.control.uz + "," +
                   d.control.uyaw + "," +
                   d.last_ok + "," +
                   d.tag);

        if (d.tag > 0) {
            log = log + "," +
                  ekf._s.x + "," +
                  ekf._s.y + "," +
                  ekf._s.yaw.toDeg() + "," +
                  ekf._m.x + "," +
                  ekf._m.y + "," +
                  ekf._m.yaw.toDeg() + "," +
                  ekf._z.x + "," +
                  ekf._z.y + "," +
                  ekf._z.yaw.toDeg() + "," +
                  ekf._e.x + "," +
                  ekf._e.y + "," +
                  ekf._e.yaw.toDeg();
        } else {
            log = log + ",0,0,0,0,0,0";
        }

        log = log + "\n";

        dataStream.write(log);
    });
};


Mission.prototype.ftrim = function () {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.ftrim");
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting ftrim");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._client.ftrim();
            // We need to call the callback ourselves since .ftrim() does not accept a callback argument
            cb();
        }
    });
    return this;
};

Mission.prototype.calibrate = function () {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.calibrate");
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting calibrate");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._client.calibrate(0);
            // We need to call the callback ourselves since .calibrate() does not accept a callback argument
            // Calibration is usually finished in 1.5 seconds
            setTimeout(cb, 4000);
        }
    });
    return this;
};

Mission.prototype.takeoff = function() {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.takeoff");
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting takeoff");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._client.takeoff(cb);
        }
    });
    return this;
};

Mission.prototype.land = function() {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.land");
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting landing");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._client.land(cb);
        }
    });

    return this;
};

Mission.prototype.hover = function(delay) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.hover(%ds)", delay);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting hover");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.hover();
            setTimeout(function() {
                self._control.enable();
                cb();
            }, delay);
        }
    });

    return this;
};

Mission.prototype.wait = function(delay) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.wait(%ds)", delay);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting wait");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            setTimeout(cb, delay);
        }
    });

    return this;
};

Mission.prototype.task = function(task) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.task");
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting task");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            task(cb);
        }
    });

    return this;
};

Mission.prototype.taskSync = function(task) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.taskSync");
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting taskSync");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            task();
            cb();
        }
    });

    return this;
};

Mission.prototype.zero = function() {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.zero");
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting zero");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.zero();
            cb();
        }
    });

    return this;
};

Mission.prototype.go = function(goal) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.go: (%d,%d,%d)", goal.x, goal.y, goal.z);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting go");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.go(goal, cb);
        }
    });

    return this;
};

Mission.prototype.forward = function(distance) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.forward(%dm)", distance);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting forward");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.forward(distance, cb);
        }
    });

    return this;
};

Mission.prototype.backward = function(distance) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.backward(%dm)", distance);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting backward");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.backward(distance, cb);
        }
    });

    return this;
};

Mission.prototype.left = function(distance) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.left(%dm)", distance);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting left");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.left(distance, cb);
        }
    });

    return this;
};

Mission.prototype.right = function(distance) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.right(%dm)", distance);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting right");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.right(distance, cb);
        }
    });

    return this;
};

Mission.prototype.up = function(distance) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.up(%dm)", distance);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting up");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.up(distance, cb);
        }
    });

    return this;
};

Mission.prototype.down = function(distance) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.down(%dm)", distance);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting down");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.down(distance, cb);
        }
    });

    return this;
};

Mission.prototype.cw = function(angle) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.cw(%d˚)", angle);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting cw");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.cw(angle, cb);
        }
    });

    return this;
};

Mission.prototype.ccw = function(angle) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.ccw(%d˚)", angle);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting ccw");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.ccw(angle, cb);
        }
    });

    return this;
};

Mission.prototype.altitude = function(altitude) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.altitude(%d)", altitude);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting altitude");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.altitude(altitude, cb);
        }
    });

    return this;
};

Mission.prototype.yaw = function(angle) {
    'use strict';
    var self = this;
    this._steps.push(function(cb) {
        if (shouldSendDebugInfo) {
            console.log("async executing Mission.yaw(%d˚)", angle);
        }
        if (self._abortMission) {
            if (shouldSendDebugInfo) {
                console.log("Aborting yaw");
            }
            cb(new Error("Mission Aborted"));
        }
        else {
            self._control.yaw(angle,cb);
        }
    });

    return this;
};
