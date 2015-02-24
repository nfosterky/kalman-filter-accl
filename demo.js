var SECOND = 1000;

function convertDegreesToRadians (degrees) {
  return degrees * (Math.PI / 180);
}

// demo.js
window.onload = function() {
  var elemPosX = document.getElementById("posX"),
      elemPosY = document.getElementById("posY"),
      elemPosZ = document.getElementById("posZ"),
      elemVelX = document.getElementById("velX"),
      elemVelY = document.getElementById("velY"),
      elemVelZ = document.getElementById("velZ"),
      light    = document.getElementById("constraintLight");

  var dt = 0.1; // multiply by second = hundred microseconds

  var alphaR, betaR, gammaR;

  function initGyro() {
    var posX = 0,
        posY = 0,
        posZ = 0;

    // set frequency of measurements in milliseconds
    gyro.frequency = dt * SECOND;

    gyro.startTracking(function(o) {
      var x, y, z;

      if (o.x !== null) {
        x = parseFloat(o.x.toFixed(5));
        y = parseFloat(o.y.toFixed(5));
        z = parseFloat(o.z.toFixed(5));

        // angular rotation velocity
        alphaR = convertDegreesToRadians(parseFloat(o.alphaR.toFixed(5))); // Z
        betaR  = convertDegreesToRadians(parseFloat(o.betaR.toFixed(5)));  // X
        gammaR = convertDegreesToRadians(parseFloat(o.gammaR.toFixed(5))); // Y

        // update changes to F matrix
        F_k.elements[0][1] = dt *  alphaR;
        F_k.elements[0][2] = dt * -gammaR;

        F_k.elements[1][0] = dt * -alphaR;
        F_k.elements[1][2] = dt *  betaR;

        F_k.elements[2][0] = dt *  gammaR;
        F_k.elements[2][1] = dt * -betaR;
        KM.F_k = F_k;

        var u_k = $V([x, y, z]);
        KM.predict(u_k);

        var accelWithoutGravity = u_k,
            linearVelocity = $V(KM.x_k.elements.slice(0,3)),
            angularVelocity = $V([betaR, gammaR, alphaR]);

        var linearAccel = accelWithoutGravity.subtract(
            angularVelocity.cross(linearVelocity));

        var velX = KM.x_k.elements[0],
            velY = KM.x_k.elements[1],
            velZ = KM.x_k.elements[2];
        // if KM.x_k.elements[0] is inside threshold it
        // if observation is available

        // if zero-velocity constraint is applicable
        var tol = 0.15,
            sigma2velUpdate = 0.1;

        if (Math.abs(u_k.modulus()) < tol) { // not much accel
          // apply zero-velocity constraint through an 'observation' of 0
          var z_k = $V([0,0,0]);
          var H_k = Matrix.I(3).augment(Matrix.Zero(3,3));
          var R_k = Matrix.Diagonal([
            sigma2velUpdate, sigma2velUpdate, sigma2velUpdate
          ]);
          var KO = new KalmanObservation(z_k, H_k, R_k);
          KM.update(KO);

          light.style.display = "block";

        } else {
          light.style.display = "none";
        }

        posX += dt * velX;
        posY += dt * velY;
        posZ += dt * velZ;

        elemPosX.innerHTML = posX.toFixed(3);
        elemPosY.innerHTML = posY.toFixed(3);
        elemPosZ.innerHTML = posZ.toFixed(3);
        elemVelX.innerHTML = velX.toFixed(3);
        elemVelY.innerHTML = velY.toFixed(3);
        elemVelZ.innerHTML = velZ.toFixed(3);

        // elemVelX.innerHTML = linearAccel.elements[0].toFixed(3);
        // elemVelY.innerHTML = linearAccel.elements[1].toFixed(3);
        // elemVelZ.innerHTML = linearAccel.elements[2].toFixed(3);
        // console.log(KM.x_k);
      }
    });
  }

  // State (3 initial velocities, 3 initial accl biases)
  var x_0 = $V([0, 0, 0, 0, 0, 0]);

  // Covariance Matrix - uncertainity of state (initial error?)
  var initPositionVariance = 0.01,
      initBiasVariance = 0;

  var P_0 = Matrix.Diagonal([
    initPositionVariance, initPositionVariance, initPositionVariance,
    initBiasVariance, initBiasVariance, initBiasVariance
  ]);

  // Transition Matrix - how each variable is updated, update each timestep
  var rows = 6,
      cols = 3;

  F_k = Matrix.I(rows);

  F_k.elements[0][3] = dt;
  F_k.elements[1][4] = dt;
  F_k.elements[2][5] = dt;

  // ~ Control Matrix - converts external inputs for updating state
  var B_k = Matrix.Zero(rows, cols); //$M([[0]]);

  B_k.elements[0][0] = dt;
  B_k.elements[1][1] = dt;
  B_k.elements[2][2] = dt;

  // Prediction Noise Matrix, weights for prediction step, previous matrices
  var vSigmaSquared = 5,    // change later ?
      bSigmaSquared = 5;    // change later ?

  var Q_k = Matrix.Diagonal([
    vSigmaSquared, vSigmaSquared, vSigmaSquared,
    bSigmaSquared, bSigmaSquared, bSigmaSquared
  ]);

  var KM = new KalmanModel(x_0, P_0, F_k, B_k, Q_k);

  // var z_k = $V([1]);
  // var H_k = $M([[1]]);
  // var R_k = $M([[4]]);
  // var KO = new KalmanObservation(z_k, H_k, R_k);

  // for (var i = 0; i < 200; i++){
  //   z_k = $V([0.5 + Math.random()]);
  //   KO.z_k = z_k;
  //   KM.predict($V([0]));
  //   //  KM.update(KO);
  //   console.log(JSON.stringify(KM.x_k.elements));
  // }

  initGyro();
};
