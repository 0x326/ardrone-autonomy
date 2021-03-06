var autonomy = exports;
var ardrone = require('ar-drone');

exports.EKF = require('./lib/EKF');
exports.Camera = require('./lib/Camera');
exports.Controller = require('./lib/Controller');
exports.Mission = require('./lib/Mission');

exports.control = function(client, options) {
    return new autonomy.Controller(client, options);
}

exports.createMission = function(options, client) {
    var client  = client || ardrone.createClient(options);
    if (typeof options.droneConfiguration != "undefined" && Array.isArray(options.droneConfiguration))
        options.droneConfiguration.forEach(function (configuration) {
            this.config(configuration.key, configuration.value, configuration.callback);
        }, client);
    var control = new autonomy.Controller(client, options);
    var mission = new autonomy.Mission(client, control, options);

    return mission;
}

