var SECOND = 1000;

function convertDegreesToRadians (degrees) {
  return degrees * (Math.PI / 180);
}

var CANVAS_MID_TOP = window.innerHeight / 2;

// demo.js

var elemPx = document.getElementById("posX"),
    elemPy = document.getElementById("posY"),
    elemPz = document.getElementById("posZ"),
    elemVx = document.getElementById("velX"),
    elemVy = document.getElementById("velY"),
    elemVz = document.getElementById("velZ"),
    light  = document.getElementById("constraintLight");

var dt = 0.1; // multiply by second = hundred microseconds

var alphaR, betaR, gammaR;

var top = 0;

var period = dt * SECOND;

function initGyro() {
  var x, y, z;

  // set frequency of measurements in milliseconds
  gyro.frequency = period;

  var xaxis = 0;

  gyro.startTracking(function(o) {
    var u_k,
        accelWithoutGravity,
        position,
        linearVelocity,
        angularVelocity,
        linearAccel;

    if (o.x !== null) {
      ax = parseFloat(o.x.toFixed(5));
      ay = parseFloat(o.y.toFixed(5));
      az = parseFloat(o.z.toFixed(5));

      // angular rotation velocity

      alphaR = convertDegreesToRadians(
        parseFloat(o.alphaR.toFixed(5))
      ); // Z

      betaR = convertDegreesToRadians(
        parseFloat(o.betaR.toFixed(5))
      );  // X

      gammaR = convertDegreesToRadians(
        parseFloat(o.gammaR.toFixed(5))
      ); // Y

      // update changes to F matrix
      F_k.elements[3][4] = dt *  alphaR;
      F_k.elements[3][5] = dt * -gammaR;

      F_k.elements[4][3] = dt * -alphaR;
      F_k.elements[4][5] = dt *  betaR;

      F_k.elements[5][3] = dt *  gammaR;
      F_k.elements[5][4] = dt * -betaR;

      KM.F_k = F_k;

      // needs comment
      u_k = $V([ax, ay, az]);

      KM.predict(u_k);

      accelWithoutGravity = u_k;
      position = $V(KM.x_k.elements.slice(0,3));
      linearVelocity = $V(KM.x_k.elements.slice(3,6));
      angularVelocity = $V([betaR, gammaR, alphaR]);

      // console.log(linearVelocity.elements);
      var lvX = linearVelocity.elements[0],
          lvY = linearVelocity.elements[1],
          lvZ = linearVelocity.elements[2],
          avX = angularVelocity.elements[0]
          avY = angularVelocity.elements[1]
          avZ = angularVelocity.elements[2];

      // need to get rid of magic numbers
      xaxis += period / 1000;

      lineLVX.data.push({x: xaxis, y: lvX});
      lineLVY.data.push({x: xaxis, y: lvY});
      lineLVZ.data.push({x: xaxis, y: lvZ});

      lineAVX.data.push({x: xaxis, y: avX});
      lineAVY.data.push({x: xaxis, y: avY});
      lineAVZ.data.push({x: xaxis, y: avZ});

      // linePX.data.push({x: xaxis, y: px});
      // linePY.data.push({x: xaxis, y: py});
      // linePZ.data.push({x: xaxis, y: pz});

      linearAccel = accelWithoutGravity.subtract(
        angularVelocity.cross(linearVelocity)
      );

      // if zero-velocity constraint is applicable
      var tol = 0.15,
          sigma2velUpdate = 0.0001;

      if (Math.abs(u_k.modulus()) < tol) { // not much accel

        // apply zero-velocity constraint through an 'observation' of 0
        var z_k = $V([0,0,0]),
            R_k = Matrix.Diagonal([
              sigma2velUpdate, sigma2velUpdate, sigma2velUpdate
            ]),
            H_k = Matrix.Zero(3,3)
                    .augment(Matrix.I(3))
                    .augment(Matrix.Zero(3,3));

        KM.update(new KalmanObservation(z_k, H_k, R_k));

        light.style.display = "block";

      } else {
        light.style.display = "none";
      }

      var px = KM.x_k.elements[0],
          py = KM.x_k.elements[1],
          pz = KM.x_k.elements[2];


      // show position
      elemPx.innerHTML = px.toFixed(3);
      elemPy.innerHTML = py.toFixed(3);
      elemPz.innerHTML = pz.toFixed(3);

      // // show velocity
      // elemVx.innerHTML = vx.toFixed(3);
      // elemVy.innerHTML = vy.toFixed(3);
      // elemVz.innerHTML = vz.toFixed(3);

      // elemVelX.innerHTML = linearAccel.elements[0].toFixed(3);
      // elemVelY.innerHTML = linearAccel.elements[1].toFixed(3);
      // elemVelZ.innerHTML = linearAccel.elements[2].toFixed(3);
      // console.log(KM.x_k);
    }
  });
}

// State (3 initial velocities, 3 initial accl biases)
var x_0 = $V([0, 0, 0, 0, 0, 0, 0, 0, 0]);

// Covariance Matrix - uncertainity of state (initial error?)
var initPositionVariance = 0.001,
    initVelocityVariance = 0.001,
    initBiasVariance = 0;

var P_0 = Matrix.Diagonal([
  initPositionVariance, initPositionVariance, initPositionVariance,
  initVelocityVariance, initVelocityVariance, initVelocityVariance,
  initBiasVariance, initBiasVariance, initBiasVariance
]);

// Transition Matrix - how each variable is updated, update each timestep
var numStateVars = 9,
    numInputVars = 3;

F_k = Matrix.I(numStateVars);

F_k.elements[0][3] = dt;
F_k.elements[1][4] = dt;
F_k.elements[2][5] = dt;

F_k.elements[3][6] = dt;
F_k.elements[4][7] = dt;
F_k.elements[5][8] = dt;

// ~ Control Matrix - converts external inputs for updating state
var B_k = Matrix.Zero(numStateVars, numInputVars); //$M([[0]]);

B_k.elements[3][0] = dt;
B_k.elements[4][1] = dt;
B_k.elements[5][2] = dt;

// Prediction Noise Matrix, weights for prediction step, previous matrices
var pSigmaSquared = .00005,    // change later ?
    vSigmaSquared = .00005,    // change later ?
    bSigmaSquared = .00005;    // change later ?

var Q_k = Matrix.Diagonal([
  pSigmaSquared, pSigmaSquared, pSigmaSquared,
  vSigmaSquared, vSigmaSquared, vSigmaSquared,
  bSigmaSquared, bSigmaSquared, bSigmaSquared
]);

var KM = new KalmanModel(x_0, P_0, F_k, B_k, Q_k);

var canvas, ell;

var lineAX, lineAY, lineAZ, graphA;
var lineVX, lineVY, lineVZ, graphV;
var linePX, linePY, linePZ, graphP;

function addGraphs () {
  lineAVX = new Line({className: "lineX"});
  lineAVY = new Line({className: "lineY"});
  lineAVZ = new Line({className: "lineZ"});

  lineLVX = new Line({className: "lineX"});
  lineLVY = new Line({className: "lineY"});
  lineLVZ = new Line({className: "lineZ"});

  graphA = new Graph({
    rangeX: 10, // we expect duration of time in seconds
    maxY: 10,
    minY: -10,
    period: period
  });

  graphA.lines = [lineAVX, lineAVY, lineAVZ];

  // pass in id of container div
  // graphA.init("graphAcclAngular");

  // graphA.startDrawing();

  /////////////////////

  graphL = new Graph({
    rangeX: 10, // we expect duration of time in seconds
    maxY: 10,
    minY: -10,
    period: period
  });

  graphL.lines = [lineLVX, lineLVY, lineLVZ];

  // pass in id of container div
  graphL.init("graphAcclLinear");

  graphL.startDrawing();

}

addGraphs();

initGyro();
