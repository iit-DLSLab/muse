
"use strict";

let resetEstimator = require('./resetEstimator.js')
let getActiveEstimators = require('./getActiveEstimators.js')
let restartEstimator = require('./restartEstimator.js')
let stopEstimator = require('./stopEstimator.js')
let getBlacklist = require('./getBlacklist.js')
let getEstimatorDescription = require('./getEstimatorDescription.js')
let startEstimator = require('./startEstimator.js')
let pauseEstimator = require('./pauseEstimator.js')
let resumeEstimator = require('./resumeEstimator.js')
let getWhitelist = require('./getWhitelist.js')
let listAllEstimators = require('./listAllEstimators.js')

module.exports = {
  resetEstimator: resetEstimator,
  getActiveEstimators: getActiveEstimators,
  restartEstimator: restartEstimator,
  stopEstimator: stopEstimator,
  getBlacklist: getBlacklist,
  getEstimatorDescription: getEstimatorDescription,
  startEstimator: startEstimator,
  pauseEstimator: pauseEstimator,
  resumeEstimator: resumeEstimator,
  getWhitelist: getWhitelist,
  listAllEstimators: listAllEstimators,
};
