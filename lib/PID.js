// This is a PID controller
// See the following for more info
// https://oscarliang.com/understanding-pid-for-quadcopter-rc-flight/

module.exports = PID;
function PID(kp, ki, kd) {
    'use strict';
    this.configure(kp, ki, kd);
    this.reset();
}

PID.prototype.configure = function(kp,ki,kd) {
    'use strict';
    this._kp = kp;
    this._ki = ki;
    this._kd = kd;
};

PID.prototype.reset = function() {
    'use strict';
    this._last_time = 0;
    this._last_error = Infinity;
    this._error_sum = 0;
};

PID.prototype.getCommand = function(e) {
    'use strict';
    // Compute dt in seconds
    var time = Date.now();
    var dt = (time - this._last_time) / 1000;

    var de = 0;
    if (this._last_time !== 0) {
        // Compute de (error derivation)
        if (this._last_error < Infinity) {
            de = (e - this._last_error) / dt;
        }

        // Integrate error
        this._error_sum += e * dt;
    }

    // Update our trackers
    this._last_time = time;
    this._last_error = e;

    // Compute commands
    var command = this._kp * e
                + this._ki * this._error_sum
                + this._kd * de;

    return command;
};
