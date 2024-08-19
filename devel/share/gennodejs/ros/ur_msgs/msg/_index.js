
"use strict";

let Analog = require('./Analog.js');
let MasterboardDataMsg = require('./MasterboardDataMsg.js');
let IOStates = require('./IOStates.js');
let Digital = require('./Digital.js');
let ToolDataMsg = require('./ToolDataMsg.js');
let RobotStateRTMsg = require('./RobotStateRTMsg.js');
let RobotModeDataMsg = require('./RobotModeDataMsg.js');

module.exports = {
  Analog: Analog,
  MasterboardDataMsg: MasterboardDataMsg,
  IOStates: IOStates,
  Digital: Digital,
  ToolDataMsg: ToolDataMsg,
  RobotStateRTMsg: RobotStateRTMsg,
  RobotModeDataMsg: RobotModeDataMsg,
};
