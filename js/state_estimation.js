var SECOND = 1000;

function convertDegreesToRadians (degrees) {
  return degrees * (Math.PI / 180);
}

var CANVAS_MID_TOP = window.innerHeight / 2;

var elemPx  = document.getElementById("posX"),
    elemPy  = document.getElementById("posY"),
    elemPz  = document.getElementById("posZ"),
    elemOFx = document.getElementById("ofX"),
    elemOFy = document.getElementById("ofY"),
    elemVx  = document.getElementById("velX"),
    elemVy  = document.getElementById("velY"),
    elemVz  = document.getElementById("velZ"),
    light   = document.getElementById("constraintLight");

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

      // input vector
      u_k = $V([ax, ay, az]);

      KM.predict(u_k);

      accelWithoutGravity = u_k;
      position = $V(KM.x_k.elements.slice(0,3));
      linearVelocity = $V(KM.x_k.elements.slice(3,6));
      angularVelocity = $V([betaR, gammaR, alphaR]);

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
      var accelNoiseThreshold = 0.15;
      if (Math.abs(u_k.modulus()) < accelNoiseThreshold) { // not much accel

        // apply zero-velocity constraint through an 'observation' of 0
        KM.update(zeroVelocityConstraint);
        light.style.display = "block"; // visualize for debugging

      } else {
        light.style.display = "none";
      }
      

      updateDisplayedValues();

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
function updateDisplayedValues(){
      var px = KM.x_k.elements[0],
          py = KM.x_k.elements[1],
          pz = KM.x_k.elements[2];

      // show position
      elemPx.innerHTML = px.toFixed(3);
      elemPy.innerHTML = py.toFixed(3);
      elemPz.innerHTML = pz.toFixed(3);
}
// ---------- STATE ESTIMATION UPDATES FROM SENSOR MEASUREMENTS

function zeroVelocityConstraint(){
  var z_k = $V([0,0,0]),                // 'measurement' velocity of 0-vector
      sigma2velUpdate = 0.0001,         // variance of constraint   
      R_k = Matrix.Diagonal([
        sigma2velUpdate, sigma2velUpdate, sigma2velUpdate
      ]),
      H_k = Matrix.Zero(3,3)
            .augment(Matrix.I(3))
            .augment(Matrix.Zero(3,3)); // matrix to extract actual velocity for comparison

  return new KalmanObservation(z_k, H_k, R_k);

}

// Meausurement of direction of translational optic flow is compared with the direction of 
// the velocity estimate. As this is a nonlinear manipulation of the state vector, 
// a Jacobian is used for the measurement matrix (refer to the extended Kalman Filter)
translationalOFdirectionConstraint = function translationalOFdirectionConstraint(velocityEstimate, opticFlow, angularVelocity) {
  var translationalOF = derotateOF(opticFlow, angularVelocity);
  var translationalOFnorm = translationalOF.modulus();
  var z_k = $V([translationalOF.elements[0]/translationalOFnorm,
                translationalOF.elements[1]/translationalOFnorm]); // measurement (2D translational OF)

  var R_body_to_image = $M([[1, 0, 0], [0, 1, 0]]); // transformation from body frame to image frame (2D)
  var velocity_imageFrame = R_body_to_image.x(velocityEstimate);
  var velocity_imageFrame_norm = velocity_imageFrame.modulus();

  var H_k = Matrix.Zero(2, 9); // Jacbobian of (nonlinear) measurement function
  for (var i = 0; i < 2; i++) {
    for (var j = 0; j < 3; j++) {
      H_k.elements[i][3+j] = -R_body_to_image.elements[i][j] / velocity_imageFrame_norm + 
        ( velocity_imageFrame.elements[0]*R_body_to_image.elements[0][j] + velocity_imageFrame.elements[1]*R_body_to_image.elements[1][j] )
          * velocity_imageFrame.elements[i] / Math.pow(velocity_imageFrame_norm, 3);
    }
  }
  var alpha=1, beta=1
  var uncertainity = alpha * 1/translationalOFnorm + beta * angularVelocity.modulus();
  var R_k = Matrix.Diagonal([uncertainity, uncertainity]);
  return new KalmanObservation(z_k, H_k, R_k);
}

function derotateOF(opticFlow, angularVelocity) {
  var viewingDirection = $V([0, 0, -1]); // back camera looks towards -z axis
  var negRotationalOF = angularVelocity.cross(viewingDirection); // THIRD ELEMENT IS 0 RIGHT?
  var translationalOF = opticFlow.add(negRotationalOF);

  var ratioThreshold = 3;
  if( negRotationalOF.modulus()/translationalOF.modulus() > ratioThreshold) {
    console.log("Warning: derotation may not be accurate.")
  }
  return translationalOF
}


function initCamera() {
    var zoneSize = 10,
        videoElement = document.getElementById('videoOut'),
        videoWidth = videoElement.videoWidth,
        videoHeight = videoElement.videoHeight;
        webCamFlow = new oflow.WebCamFlow(videoElement, zoneSize, 'environment');

    webCamFlow.onCalculated( function (direction) {
      if(Math.abs(direction.u) > 0.1)
        elemOFx.innerHTML = direction.u.toFixed(3);
      if(Math.abs(direction.v) > 0.1)
        elemOFy.innerHTML = direction.v.toFixed(3);
    

      // update state velocity based on OF direction
      var opticFlow = $V([direction.u, direction.v, 0]),
          linearVelocity = $V(KM.x_k.elements.slice(3,6)),
          angularVelocity = $V([betaR, gammaR, alphaR]); // change so ensure current vals

      var opticFlowNoiseThreshold = 0.1;
      if (opticFlow.modulus() > opticFlowNoiseThreshold) {
        KM.update(translationalOFdirectionConstraint(linearVelocity, opticFlow, angularVelocity));
        updateDisplayedValues();
      }

    });
        webCamFlow.startCapture();
}


// State (3 initial positions, 3 initial velocities, 3 initial accl biases)
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

initCamera();

initGyro();
