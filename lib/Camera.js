var sylvester    = require('sylvester');
var util         = require('util');


// TODO: Extend to support roll/pitch in back projection
// TODO: Add support for front-facing camera
// TODO: Make image aspect ratio configurable

// AR Drone 2.0 Bottom Camera Intrinsic Matrix
// https://github.com/tum-vision/ardrone_autonomy/blob/master/calibrations/ardrone2_bottom/cal.yml
// The code is under the 'BSD' license, which is assumed to be the following license
//    Copyright (c) 2012, Mani Monajjemi
//    All rights reserved.
//
//    Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
//
//    1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//
//    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
//
//    3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

var K_BOTTOM = $M([[686.994766, 0, 329.323208],
                   [0, 688.195055, 159.323007],
                   [0, 0, 1]]);

module.exports = Camera;
function Camera(options) {
    this._options = options || {};
    this._k = this._options.k || K_BOTTOM;

    // We need to compute the inverse of K to back-project 2D to 3D
    this._invK = this._k.inverse();
}

/*
 * Given (x,y) pixel coordinates (e.g. obtained from tag detection)
 * Returns a (X,Y) coordinate in drone space.
 */
Camera.prototype.p2m = function(xC, yC, altitude) {
    // See "7.1.3 Augmented reality data stream" of SDK 2.0.1 Documentation (Page 42)
    // Read the third bullet point "xc[i], yc[i]: ... "
    //
    // But our camera intrinsic is built for 640 x 360 pixel grid, so we must do some mapping.
    var xRatio = 640 / 1000;
    var yRatio = 360 / 1000;

    // Perform a simple back projection, we assume the drone is flat (no roll/pitch)
    // for the moment. We ignore the drone translation and yaw since we want X,Y in the
    // drone coordinate system.
    var p = $V([xC * xRatio, yC * yRatio, 1]);
    var P = this._invK.multiply(p).multiply(altitude);

    // X,Y are expressed in meters, in the drone coordinate system.
    // Which is:
    //          <--- front-facing camera
    //        |
    //       / \------- X
    //       \_/
    //        |
    //        |
    //        Y
    return {x: P.e(1), y: P.e(2)};
};

