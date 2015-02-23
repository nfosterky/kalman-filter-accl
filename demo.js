var SECOND = 1000;

function convertDegreesToRadians (degrees) {
  return degrees * (Math.PI / 180);
}

// demo.js
window.onload = function() {
  var elemPosX = document.getElementById("posX"),
      elemPosY = document.getElementById("posY"),
      elemPosZ = document.getElementById("posZ");

  var dt = 0.1;

  var alphaR, betaR, gammaR;

  function initGyro() {
    // set frequency of measurements in milliseconds
    gyro.frequency = dt * SECOND;

    gyro.startTracking(function(o) {
      var x = parseFloat(o.x.toFixed(5)),
          y = parseFloat(o.y.toFixed(5)),
          z = parseFloat(o.z.toFixed(5));

      elemPosX.innerHTML = x;
      elemPosY.innerHTML = y;
      elemPosZ.innerHTML = z;

      alphaR = convertDegreesToRadians(o.alphaR);
      betaR  = convertDegreesToRadians(o.betaR);
      gammaR = convertDegreesToRadians(o.gammaR);

      F_k.elements[0][1] = betaR * dt;
      F_k.elements[0][2] = betaR * dt;
    });
  }

  // State (3 initial velocities, 3 initial accl biases)
  var x_0 = $V([0, 0, 0, .1, .1, .1]);

  // Covariance Matrix - uncertainity of state (initial error?)
  var initPositionVariance = 0.1,
    initBiasVariance = 1;

  var P_0 = Matrix.Diagonal([
        initPositionVariance, initPositionVariance, initPositionVariance,
        initBiasVariance, initBiasVariance, initBiasVariance
      ]);

  // Transition Matrix - how each variable is updated, update each timestep
  var rows = 6,
    cols = 3;
  var F_k = Matrix.I(rows);
  F_k.elements[0][3] = dt;
  F_k.elements[1][4] = dt;
  F_k.elements[2][5] = dt;

  // ~ Control Matrix - converts external inputs for updating state
  var B_k = Matrix.Zero(rows, cols); //$M([[0]]);

  B_k.elements[0][0] = dt;
  B_k.elements[1][1] = dt;
  B_k.elements[2][2] = dt;

  // Prediction Noise Matrix, weights for prediction step, previous matrices
  var vSigmaSquared = 1,  // change later ?
    bSigmaSquared = 1;    // change later ?

  var Q_k = Matrix.Diagonal([
    vSigmaSquared, vSigmaSquared, vSigmaSquared,
    bSigmaSquared, bSigmaSquared, bSigmaSquared
  ]);

  var KM = new KalmanModel(x_0, P_0, F_k, B_k, Q_k);

  var z_k = $V([1]);
  var H_k = $M([[1]]);
  var R_k = $M([[4]]);
  var KO = new KalmanObservation(z_k, H_k, R_k);

  // for (var i = 0; i < 200; i++){
  //   z_k = $V([0.5 + Math.random()]);
  //   KO.z_k = z_k;
  //   KM.predict($V([0]));
  //   //  KM.update(KO);
  //   console.log(JSON.stringify(KM.x_k.elements));
  // }

  initGyro();
};
