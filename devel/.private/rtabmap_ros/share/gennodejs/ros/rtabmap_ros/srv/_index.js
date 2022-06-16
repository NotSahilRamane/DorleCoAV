
"use strict";

let SetGoal = require('./SetGoal.js')
let PublishMap = require('./PublishMap.js')
let ListLabels = require('./ListLabels.js')
let GetMap = require('./GetMap.js')
let RemoveLabel = require('./RemoveLabel.js')
let LoadDatabase = require('./LoadDatabase.js')
let DetectMoreLoopClosures = require('./DetectMoreLoopClosures.js')
let GetNodeData = require('./GetNodeData.js')
let GlobalBundleAdjustment = require('./GlobalBundleAdjustment.js')
let GetPlan = require('./GetPlan.js')
let CleanupLocalGrids = require('./CleanupLocalGrids.js')
let SetLabel = require('./SetLabel.js')
let GetNodesInRadius = require('./GetNodesInRadius.js')
let AddLink = require('./AddLink.js')
let ResetPose = require('./ResetPose.js')
let GetMap2 = require('./GetMap2.js')

module.exports = {
  SetGoal: SetGoal,
  PublishMap: PublishMap,
  ListLabels: ListLabels,
  GetMap: GetMap,
  RemoveLabel: RemoveLabel,
  LoadDatabase: LoadDatabase,
  DetectMoreLoopClosures: DetectMoreLoopClosures,
  GetNodeData: GetNodeData,
  GlobalBundleAdjustment: GlobalBundleAdjustment,
  GetPlan: GetPlan,
  CleanupLocalGrids: CleanupLocalGrids,
  SetLabel: SetLabel,
  GetNodesInRadius: GetNodesInRadius,
  AddLink: AddLink,
  ResetPose: ResetPose,
  GetMap2: GetMap2,
};
