var df = require('dateformat')
  , autonomy = require('../')
  , arDrone = require('ar-drone')
  , arDroneConstants = require('ar-drone/lib/constants')
  ;
var chalk = require("chalk");

// Land on ctrl-c
var exiting = false;
process.on('SIGINT', function() {
    if (exiting) {
        process.exit(0);
    } else {
        console.log('Got SIGINT. Landing, press Control-C again to force exit.');
        exiting = true;
        mission.control().disable();
        mission.client().land(function() {
            process.exit(0);
        });
    }
});

var options = undefined;
/*var options = {pid: { x_axis:   {p_constant: 0.75, i_constant: 0.3, d_constant: 0.35}, 
                      y_axis:   {p_constant: 0.75, i_constant: 0.3, d_constant: 0.35}, 
                      z_axis:   {p_constant: 0.8, i_constant: 0.0, d_constant: 0.35}, 
                      yaw_axis: {p_constant: 1.0, i_constant: 0.0, d_constant: 0.30}}};
*/
var mission  = autonomy.createMission(options);
console.log("mission options: " + JSON.stringify(options));

function navdata_option_mask(c) {
  return 1 << c;
}

// From the SDK.
var navdata_options = (
    navdata_option_mask(arDroneConstants.options.DEMO)
  | navdata_option_mask(arDroneConstants.options.VISION_DETECT)
  | navdata_option_mask(arDroneConstants.options.MAGNETO)
  | navdata_option_mask(arDroneConstants.options.WIFI)
  | navdata_option_mask(arDroneConstants.options.ZIMMU_3000) // To send GPS data
);

// Connect and configure the drone
mission.client().config('general:navdata_demo', true);
mission.client().config('general:navdata_options', navdata_options);
mission.client().config('video:video_channel', 1);
mission.client().config('detect:detect_type', 12);

// Log mission for debugging purposes
var logFileName = "/tmp/mission-" + df(new Date(), "yyyy-mm-dd_hh-MM-ss") + ".txt"
console.log("Mission Log: " + logFileName);
mission.log(logFileName);
console.log(".zero()");
mission.zero()
    .altitude(1.3)
    .hover(250);
mission.run(function (err, result) {
        if (err) {
            console.trace("Oops, something bad happened: %s", err.message);
            mission.client().stop();
            mission.client().land();
        } else {
            console.log(chalk.inverse.green("====Mission complete===="));
            console.time(chalk.dim.blue("Call >> waterfall"));
            setImmediate(doOver);
        }
    });

function doOver () {
    // Plan mission
    console.log(chalk.green("====Planning mission===="));
    mission.takeoff()
        
        .go({x:0, y:0}).hover(500)
        .go({x:0, y:1}).hover(500)
        .go({x:-1, y:1}).hover(500)
        .go({x:0, y:1}).hover(500)
        .go({x:0, y:0}).hover(500)
        .go({x:1, y:0}).hover(500)
        .go({x:0, y:0}).hover(500)

    // Execute mission
    console.log(chalk.dim.blue("Running mission"));
    console.timeEnd(chalk.dim.blue("Call >> waterfall"));
}
