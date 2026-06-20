const PAGE_BUILD_ID = document.querySelector('meta[name="app-build-id"]')?.content || "unknown";
const { RobotView } = await import(`/static/robot_view.js?v=${encodeURIComponent(PAGE_BUILD_ID)}`);

const state = {
  config: null,
  robotState: null,
  lastRobotStateUpdatedAt: 0,
  ws: null,
  commandTimer: null,
  pendingAngles: null,
  draftAngles: null,
  commandedAngles: null,
  lastSentAngles: null,
  intentBasePoseRevision: null,
  view: null,
  activeTab: "joint",
  previewId: null,
  latestPreview: null,
  previewAngles: null,
  previewBasePoseRevision: null,
  viewPreviewSource: null,
  ikPreviewTimer: null,
  ikPreviewInFlight: false,
  ikPreviewQueued: false,
  ikPreviewWantedSeq: 0,
  lastIkPreviewMs: 0,
  ikPreviewSeq: 0,
  ikUserEdited: false,
  liveTargetTimer: null,
  liveTargetInFlight: false,
  liveTargetQueued: false,
  pendingLiveTarget: null,
  programWaypoints: [],
  programSelectedId: null,
  programRevision: 0,
  programPreviewRevision: null,
  programValidationRevision: null,
  programPreview: null,
  programPreviewFailure: null,
  programPreviewPending: false,
  programHasPreviewed: false,
  programExecutionActive: false,
  programExecutionAwaitingStart: false,
  programExecutionFailed: false,
  programExecutionError: "",
  programLastEditReason: "",
  programNextId: 1,
  hardwareDraftDirty: false,
  settingsDirtyScopes: new Set(),
  taskPreviewId: null,
  taskPreviewCreatedAt: null,
  taskLocalStatusAt: 0,
  lastTaskPreview: null,
  selectedDetectionIds: new Set(),
  selectedSerialPort: null,
  latestDetections: [],
  taskDetectionsCapturedAt: null,
  activeTaskStep: "setup",
  taskProgressStep: "setup",
  taskColorProfilesDraft: null,
  taskDropZonesDraft: null,
  unsavedColorProfiles: new Set(),
  cameraTimer: null,
  cameraInFlight: false,
  cameraLive: false,
  workspaceProjectionTimer: null,
  workspaceProjectionInFlight: false,
  linkDraft: null,
  dhDraftRows: null,
  toolSliderDraftValue: null,
  toolSliderEditing: false,
  toolSliderSendTimer: null,
  toolSliderSendInFlight: false,
  toolSliderInFlightValue: null,
  toolSliderQueuedValue: null,
  toolSliderLastCommandValue: null,
  cartesianJogTimer: null,
  cartesianJogInFlight: false,
  cartesianJogQueued: false,
  cartesianJogVelocity: { vx_mm_s: 0, vy_mm_s: 0, vz_mm_s: 0, vphi_deg_s: 0 },
  cartesianJogLastSentMs: 0,
  cartesianJogActiveAxes: new Set(),
  cartesianJogEpoch: 0,
  cartesianJogStopPending: false,
  cartesianJogStopInFlight: false,
  workspaceCalibrationBusy: false,
  workspaceCalibrationStatus: null,
  tcpCalibrationTargets: [],
  tcpCalibrationMove: null,
  tcpCalibrationMeasurementSource: { xy: "manual", z: "manual" },
  versionTimer: null,
};

const geometryDimensionFields = [
  ["L_1", "L1", "Base vertical section"],
  ["L_2", "L2", "Base side offset"],
  ["L_3", "L3", "Shoulder height section"],
  ["L_4", "L4", "Shoulder lateral offset"],
  ["L_5", "L5", "Upper arm length"],
  ["L_6", "L6", "Elbow lateral offset"],
  ["L_7", "L7", "Forearm length"],
  ["L_8", "L8", "Wrist lateral offset"],
  ["L_9", "L9", "Wrist link length"],
];

const geometrySignFields = [
  ["s4", "Shoulder offset direction"],
  ["s6", "Elbow offset direction"],
  ["s8", "Wrist offset direction"],
];

const targetDefs = [
  ["x", "X", "x_mm", "mm", 1],
  ["y", "Y", "y_mm", "mm", 1],
  ["z", "Z", "z_mm", "mm", 1],
  ["phi", "Phi", "phi_deg", "deg", 0.5],
];

const $ = (selector) => document.querySelector(selector);

const elements = {
  appLayout: $("#appLayout"),
  mainPanel: document.querySelector(".main-panel"),
  appTabs: document.querySelectorAll("[data-app-tab]"),
  appTabPanels: {
    joint: $("#jointTab"),
    operate: $("#operateTab"),
    ik: $("#ikTab"),
    program: $("#programTab"),
    settings: $("#settingsTab"),
  },
  panelResizer: $("#panelResizer"),
  tabHeader: $("#tabHeader"),
  collapsePanelBtn: $("#collapsePanelBtn"),
  jointControls: $("#jointControls"),
  connectionBadge: $("#connectionBadge"),
  modeBadge: $("#modeBadge"),
  motionBadge: $("#motionBadge"),
  armedBadge: $("#armedBadge"),
  commandRate: $("#commandRate"),
  eeCompact: $("#eeCompact"),
  targetHud: $("#targetHud"),
  pathHud: $("#pathHud"),
  motionHud: $("#motionHud"),
  progressHud: $("#progressHud"),
  statusPill: $("#statusPill"),
  fkX: $("#fkX"),
  fkY: $("#fkY"),
  fkZ: $("#fkZ"),
  fkPitch: $("#fkPitch"),
  lastCommand: $("#lastCommand"),
  lastError: $("#lastError"),
  portStatus: $("#portStatus"),
  serialModal: $("#serialModal"),
  serialPortList: $("#serialPortList"),
  refreshPortsBtn: $("#refreshPortsBtn"),
  closeSerialModalBtn: $("#closeSerialModalBtn"),
  connectSelectedSerialBtn: $("#connectSelectedSerialBtn"),
  baudRate: $("#baudRate"),
  resetViewBtn: $("#resetViewBtn"),
  togglePreviewBtn: $("#togglePreviewBtn"),
  togglePathBtn: $("#togglePathBtn"),
  toggleFramesBtn: $("#toggleFramesBtn"),
  connectSimBtn: $("#connectSimBtn"),
  connectSerialBtn: $("#connectSerialBtn"),
  disconnectBtn: $("#disconnectBtn"),
  homeBtn: $("#homeBtn"),
  setPoseBtn: $("#setPoseBtn"),
  stopBtn: $("#stopBtn"),
  diagnosticsBtn: $("#diagnosticsBtn"),
  diagnosticsDrawer: $("#diagnosticsDrawer"),
  diagnosticsSummary: $("#diagnosticsSummary"),
  closeDiagnosticsBtn: $("#closeDiagnosticsBtn"),
  liveJogToggle: $("#liveJogToggle"),
  liveRealToggle: $("#liveRealToggle"),
  cartesianJogToggle: $("#cartesianJogToggle"),
  cartesianJogSpeedInput: $("#cartesianJogSpeedInput"),
  cartesianJogPhiSpeedInput: $("#cartesianJogPhiSpeedInput"),
  cartesianJogStatus: $("#cartesianJogStatus"),
  buildStatus: $("#buildStatus"),
  buildStatusText: $("#buildStatusText"),
  applyJointPreviewBtn: $("#applyJointPreviewBtn"),
  resetJointPreviewBtn: $("#resetJointPreviewBtn"),
  hardwareArmToggle: $("#hardwareArmToggle"),
  globalSpeedInput: $("#globalSpeedInput"),
  globalAccelInput: $("#globalAccelInput"),
  waypointRateInput: $("#waypointRateInput"),
  cartesianStepInput: $("#cartesianStepInput"),
  plannerTypeSelect: $("#plannerTypeSelect"),
  jerkPercentInput: $("#jerkPercentInput"),
  blendPercentInput: $("#blendPercentInput"),
  perJointTuning: $("#perJointTuning"),
  linkCalibration: $("#linkCalibration"),
  jointCalibration: $("#jointCalibration"),
  hardwareIo: $("#hardwareIo"),
  hardwareStatus: $("#hardwareStatus"),
  syncHardwareBtn: $("#syncHardwareBtn"),
  addDropPresetBtn: $("#addDropPresetBtn"),
  dropPresetEditor: $("#dropPresetEditor"),
  colorProfileEditor: $("#colorProfileEditor"),
  saveCalibrationBtn: $("#saveCalibrationBtn"),
  discardSettingsBtn: $("#discardSettingsBtn"),
  settingsSaveStatus: $("#settingsSaveStatus"),
  settingsSaveIndicator: $("#settingsSaveIndicator"),
  settingsSaveBar: $(".settings-save-bar"),
  settingsSectionNav: $(".settings-section-nav"),
  calibrationStatus: $("#calibrationStatus"),
  ikTargetControls: $("#ikTargetControls"),
  sliderRangeControls: $("#sliderRangeControls"),
  ikModeSelect: $("#ikModeSelect"),
  ikBranchSelect: $("#ikBranchSelect"),
  ikAutoPhiToggle: $("#ikAutoPhiToggle"),
  previewIkBtn: $("#previewIkBtn"),
  executeIkBtn: $("#executeIkBtn"),
  ikStopBtn: $("#ikStopBtn"),
  ikCandidateList: $("#ikCandidateList"),
  ikPathSummary: $("#ikPathSummary"),
  previewStatus: $("#previewStatus"),
  programWorkflow: $("#programWorkflow"),
  programStatusDetail: $("#programStatusDetail"),
  programStepSource: $("#programStepSource"),
  programSourceItemField: $("#programSourceItemField"),
  programSourceItem: $("#programSourceItem"),
  programInsertPosition: $("#programInsertPosition"),
  programAddHint: $("#programAddHint"),
  addProgramStepBtn: $("#addProgramStepBtn"),
  clearProgramBtn: $("#clearProgramBtn"),
  previewProgramBtn: $("#previewProgramBtn"),
  executeProgramBtn: $("#executeProgramBtn"),
  programList: $("#programList"),
  programStatus: $("#programStatus"),
  programStepCount: $("#programStepCount"),
  programEstimatedDuration: $("#programEstimatedDuration"),
  programInspectorSection: $("#programInspectorSection"),
  programInspectorTitle: $("#programInspectorTitle"),
  programInspector: $("#programInspector"),
  programPreviewSummary: $("#programPreviewSummary"),
  programExecuteHint: $("#programExecuteHint"),
  namedPositionsList: $("#namedPositionsList"),
  eventLog: $("#eventLog"),
  toolStatus: $("#toolStatus"),
  toolSelect: $("#toolSelect"),
  toolValueSlider: $("#toolValueSlider"),
  gripperControls: $("#gripperControls"),
  gripperSliderLabel: $("#gripperSliderLabel"),
  magnetControls: $("#magnetControls"),
  toolOpenBtn: $("#toolOpenBtn"),
  toolCloseBtn: $("#toolCloseBtn"),
  toolOnBtn: $("#toolOnBtn"),
  toolOffBtn: $("#toolOffBtn"),
  taskModeSelect: $("#taskModeSelect"),
  dropZoneSelect: $("#dropZoneSelect"),
  sortColorSelect: $("#sortColorSelect"),
  taskWorkflowStepper: $("#taskWorkflowStepper"),
  taskStepPanels: document.querySelectorAll("[data-task-panel]"),
  taskSetupChecklist: $("#taskSetupChecklist"),
  executionStrategySelect: $("#executionStrategySelect"),
  objectSelectionPolicySelect: $("#objectSelectionPolicySelect"),
  maxObjectsInput: $("#maxObjectsInput"),
  minConfidenceInput: $("#minConfidenceInput"),
  includeColorsSelect: $("#includeColorsSelect"),
  colorPresetMapping: $("#colorPresetMapping"),
  colorPriorityInput: $("#colorPriorityInput"),
  missingDropzonePolicySelect: $("#missingDropzonePolicySelect"),
  unknownColorPolicySelect: $("#unknownColorPolicySelect"),
  placementPolicySelect: $("#placementPolicySelect"),
  pickupZInput: $("#pickupZInput"),
  dropoffZInput: $("#dropoffZInput"),
  approachClearanceInput: $("#approachClearanceInput"),
  dropApproachClearanceInput: $("#dropApproachClearanceInput"),
  orientationPolicySelect: $("#orientationPolicySelect"),
  pickupPhiInput: $("#pickupPhiInput"),
  dropPhiInput: $("#dropPhiInput"),
  transferModeSelect: $("#transferModeSelect"),
  pickupDescentModeSelect: $("#pickupDescentModeSelect"),
  liftModeSelect: $("#liftModeSelect"),
  dropDescentModeSelect: $("#dropDescentModeSelect"),
  captureSettleInput: $("#captureSettleInput"),
  toolSettleInput: $("#toolSettleInput"),
  objectProfilesInput: $("#objectProfilesInput"),
  previewTaskBtn: $("#previewTaskBtn"),
  executeTaskBtn: $("#executeTaskBtn"),
  taskStopBtn: $("#taskStopBtn"),
  taskStatus: $("#taskStatus"),
  taskSummary: $("#taskSummary"),
  taskPlanPreview: $("#taskPlanPreview"),
  taskRunMonitor: $("#taskRunMonitor"),
  viewCameraBtn: $("#viewCameraBtn"),
  detectVisionBtn: $("#detectVisionBtn"),
  cameraPopup: $("#cameraPopup"),
  cameraPopupHandle: $("#cameraPopupHandle"),
  cameraPopupStatus: $("#cameraPopupStatus"),
  cameraPopupRefreshBtn: $("#cameraPopupRefreshBtn"),
  cameraLiveToggle: $("#cameraLiveToggle"),
  closeCameraPopupBtn: $("#closeCameraPopupBtn"),
  cameraFrame: $("#cameraFrame"),
  cameraPlaceholder: $("#cameraPlaceholder"),
  detectionList: $("#detectionList"),
  visionProfileList: $("#visionProfileList"),
  visionSummary: $("#visionSummary"),
  dhTableEditor: $("#dhTableEditor"),
  dhTableStatus: $("#dhTableStatus"),
  geometryPresetEditor: $("#geometryPresetEditor"),
  toolCalibration: $("#toolCalibration"),
  encoderCalibration: $("#encoderCalibration"),
  workspaceCalibrationStatus: $("#workspaceCalibrationStatus"),
  cameraEnabledInput: $("#cameraEnabledInput"),
  cameraSourceInput: $("#cameraSourceInput"),
  cameraWidthInput: $("#cameraWidthInput"),
  cameraHeightInput: $("#cameraHeightInput"),
  workspaceProjectionInput: $("#workspaceProjectionInput"),
  workspaceArucoEnabledInput: $("#workspaceArucoEnabledInput"),
  workspaceArucoInvertInput: $("#workspaceArucoInvertInput"),
  workspaceArucoFallbackInput: $("#workspaceArucoFallbackInput"),
  cameraFxInput: $("#cameraFxInput"),
  cameraFyInput: $("#cameraFyInput"),
  cameraCxInput: $("#cameraCxInput"),
  cameraCyInput: $("#cameraCyInput"),
  cameraDistortionInput: $("#cameraDistortionInput"),
  calibrateWorkspaceBtn: $("#calibrateWorkspaceBtn"),
  verifyWorkspaceCalibrationBtn: $("#verifyWorkspaceCalibrationBtn"),
  workspaceCalibrationFrame: $("#workspaceCalibrationFrame"),
  workspaceCalibrationPlaceholder: $("#workspaceCalibrationPlaceholder"),
  workspaceCalibrationMetrics: $("#workspaceCalibrationMetrics"),
  workspaceCalibrationDetections: $("#workspaceCalibrationDetections"),
  tcpCalibrationStatus: $("#tcpCalibrationStatus"),
  tcpCalibrationWorkspaceStatus: $("#tcpCalibrationWorkspaceStatus"),
  tcpCalibrationModelStatus: $("#tcpCalibrationModelStatus"),
  tcpCalRowsInput: $("#tcpCalRowsInput"),
  tcpCalColumnsInput: $("#tcpCalColumnsInput"),
  tcpCalMarginInput: $("#tcpCalMarginInput"),
  tcpCalTargetZInput: $("#tcpCalTargetZInput"),
  tcpCalTargetPhiInput: $("#tcpCalTargetPhiInput"),
  tcpCalGenerateBtn: $("#tcpCalGenerateBtn"),
  tcpCalibrationTargetList: $("#tcpCalibrationTargetList"),
  tcpCalXInput: $("#tcpCalXInput"),
  tcpCalYInput: $("#tcpCalYInput"),
  tcpCalZInput: $("#tcpCalZInput"),
  tcpCalPhiInput: $("#tcpCalPhiInput"),
  tcpCalPreviewFitBtn: $("#tcpCalPreviewFitBtn"),
  tcpCalPreviewValidationBtn: $("#tcpCalPreviewValidationBtn"),
  tcpCalExecuteBtn: $("#tcpCalExecuteBtn"),
  tcpCalibrationMoveStatus: $("#tcpCalibrationMoveStatus"),
  tcpCalMarkerLabelInput: $("#tcpCalMarkerLabelInput"),
  tcpCalMeasuredXInput: $("#tcpCalMeasuredXInput"),
  tcpCalMeasuredYInput: $("#tcpCalMeasuredYInput"),
  tcpCalMeasuredZInput: $("#tcpCalMeasuredZInput"),
  tcpCalRoleSelect: $("#tcpCalRoleSelect"),
  tcpCalQualityInput: $("#tcpCalQualityInput"),
  tcpCalSurfaceZInput: $("#tcpCalSurfaceZInput"),
  tcpCalContactOffsetInput: $("#tcpCalContactOffsetInput"),
  tcpCalCaptureXyBtn: $("#tcpCalCaptureXyBtn"),
  tcpCalUseTouchOffBtn: $("#tcpCalUseTouchOffBtn"),
  tcpCalSaveSampleBtn: $("#tcpCalSaveSampleBtn"),
  tcpCalibrationMeasurementStatus: $("#tcpCalibrationMeasurementStatus"),
  tcpCalModelSelect: $("#tcpCalModelSelect"),
  tcpCalEnableInput: $("#tcpCalEnableInput"),
  tcpCalFitBtn: $("#tcpCalFitBtn"),
  tcpCalApplyEnableBtn: $("#tcpCalApplyEnableBtn"),
  tcpCalibrationMetrics: $("#tcpCalibrationMetrics"),
  tcpCalibrationSamples: $("#tcpCalibrationSamples"),
};

function renderBuildStatus(version) {
  if (!elements.buildStatus || !elements.buildStatusText) return;
  const currentFrontendBuild = version?.frontend_build_id || version?.running_build_id;
  const runningBackendBuild = version?.running_backend_build_id || version?.running_build_id;
  const diskBackendBuild = version?.disk_backend_build_id || version?.disk_build_id;
  const versionAvailable = Boolean(version?.ok && currentFrontendBuild);
  const browserStale = Boolean(versionAvailable && PAGE_BUILD_ID !== currentFrontendBuild);
  const serverStale = Boolean(version?.backend_restart_required ?? version?.restart_required);
  const configStale = Boolean(version?.config_reload_required);
  const remoteDiffers = Boolean(version?.remote_differs ?? version?.pull_required);
  const localStale = !versionAvailable || browserStale || serverStale || configStale;
  elements.buildStatus.classList.toggle("stale", localStale);
  elements.buildStatus.classList.toggle("current", !localStale);
  elements.buildStatus.dataset.action = !versionAvailable
    ? "check"
    : serverStale
      ? "restart"
      : configStale
        ? "reload-config"
        : browserStale
          ? "reload"
          : remoteDiffers
            ? "remote"
            : "current";
  if (!versionAvailable) {
    elements.buildStatusText.textContent = "Version check unavailable";
  } else if (serverStale) {
    elements.buildStatusText.textContent = "Backend outdated - restart localhost";
  } else if (configStale) {
    elements.buildStatusText.textContent = "Settings file changed - restart localhost";
  } else if (browserStale) {
    elements.buildStatusText.textContent = "Browser outdated - click to reload";
  } else {
    const commit = version?.git_commit ? ` ${version.git_commit.slice(0, 7)}` : "";
    elements.buildStatusText.textContent = remoteDiffers
      ? `Current localhost${commit} - remote differs`
      : `Current localhost${commit}`;
  }
  elements.buildStatus.title = [
    ...(version?.reasons || []),
    `Local freshness: ${localStale ? "stale" : "current"}`,
    remoteDiffers
      ? "Remote status: local HEAD differs from origin/main"
      : `Remote status: ${version?.origin_main_commit ? "matches origin/main" : "unavailable"}`,
    `Browser assets: ${PAGE_BUILD_ID}`,
    `Frontend files: ${currentFrontendBuild || "unknown"}`,
    `Running backend: ${runningBackendBuild || "unknown"}`,
    `Backend files: ${diskBackendBuild || "unknown"}`,
    `Runtime config: ${version?.running_config_id || "unknown"}`,
    `Config on disk: ${version?.disk_config_id || "unknown"}`,
    `Server commit: ${version?.git_commit || "unknown"}`,
    `Local HEAD: ${version?.current_git_commit || "unknown"}`,
    `origin/main: ${version?.origin_main_commit || "unavailable"}`,
    version?.checkout_path ? `Checkout: ${version.checkout_path}` : "",
    version?.started_at ? `Server started: ${version.started_at}` : "",
  ].filter(Boolean).join("\n");
}

async function checkAppVersion() {
  try {
    const response = await fetch(`/api/version?t=${Date.now()}`, { cache: "no-store" });
    renderBuildStatus(await response.json());
  } catch {
    if (elements.buildStatusText) elements.buildStatusText.textContent = "Version check unavailable";
    elements.buildStatus?.classList.add("stale");
    elements.buildStatus?.classList.remove("current");
    if (elements.buildStatus) elements.buildStatus.dataset.action = "check";
  }
}

function format(value, decimals = 1) {
  return Number(value || 0).toFixed(decimals);
}

function readNumber(input, fallback = 0) {
  const value = Number(input?.value);
  return Number.isFinite(value) ? value : fallback;
}

function readRequiredNumber(input, label, options = {}) {
  const raw = String(input?.value ?? "").trim();
  if (!raw) throw new Error(`${label} is required`);
  const value = Number(raw);
  if (!Number.isFinite(value)) throw new Error(`${label} must be a finite number`);
  if (options.integer && !Number.isInteger(value)) throw new Error(`${label} must be an integer`);
  if (options.min !== undefined && value < options.min) {
    throw new Error(`${label} must be ${options.min} or greater`);
  }
  if (options.max !== undefined && value > options.max) {
    throw new Error(`${label} must be ${options.max} or less`);
  }
  return value;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

async function postJson(url, body = {}) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const payload = await response.json();
  if (!response.ok && !payload.error) {
    payload.ok = false;
    payload.error = formatApiError(payload, response.status);
  }
  if (!payload.ok && payload.error) showLocalError(payload.error);
  return payload;
}

function formatApiError(payload, status) {
  const details = Array.isArray(payload.detail) ? payload.detail : [];
  if (!details.length) return `Request failed (${status})`;
  return details
    .map((item) => {
      const path = Array.isArray(item.loc) ? item.loc.filter((part) => part !== "body").join(".") : "";
      return `${path ? `${path}: ` : ""}${item.msg || "invalid value"}`;
    })
    .join("; ");
}

function showLocalError(message) {
  if (elements.lastError) elements.lastError.textContent = message || "-";
  if (elements.statusPill && message) elements.statusPill.textContent = message;
  if (elements.previewStatus && message) elements.previewStatus.textContent = message;
}

function isMoveEnabled() {
  if (!state.robotState) return false;
  return (
    (state.robotState.connected || state.robotState.simulation) &&
    state.robotState.motion_state !== "estop" &&
    state.robotState.motion_state !== "fault"
  );
}

function linkDefaults() {
  const tcpReach = activeToolTcpReach();
  const links = state.linkDraft;
  if (links) {
    return {
      l1: links.base_height || 0,
      l2: links.upper_arm || 0,
      l3: links.forearm || 0,
      l4: (links.wrist || 0) + (links.tool || 0) + tcpReach,
    };
  }
  const configLinks = state.config?.links_mm || {};
  return {
    l1: configLinks.base_height_mm ?? configLinks.base_height ?? 0,
    l2: configLinks.upper_arm_mm ?? configLinks.upper_arm ?? 0,
    l3: configLinks.forearm_mm ?? configLinks.forearm ?? 0,
    l4: (configLinks.wrist_mm ?? configLinks.wrist ?? 0) + (configLinks.tool_mm ?? configLinks.tool ?? 0) + tcpReach,
  };
}

function tcpOffsetReach(offset = {}) {
  const x = Number(offset.x ?? offset.x_mm ?? 0);
  const y = Number(offset.y ?? offset.y_mm ?? 0);
  const z = Number(offset.z ?? offset.z_mm ?? 0);
  return Math.hypot(Number.isFinite(x) ? x : 0, Number.isFinite(y) ? y : 0, Number.isFinite(z) ? z : 0);
}

function activeToolTcpReach() {
  const linksOffset = state.config?.links_mm?.tool_tcp_offset_mm;
  if (linksOffset) return tcpOffsetReach(linksOffset);
  const tools = state.config?.tools || {};
  const active = tools.active || state.robotState?.active_tool || state.config?.tool?.active || "gripper";
  const preset = tools.presets?.[active] || {};
  return tcpOffsetReach(preset.tcp_offset_mm || state.config?.tool?.tcp_offset_mm || {});
}

function buildJointControls() {
  elements.jointControls.innerHTML = "";
  state.config.joints.forEach((joint, index) => {
    const row = document.createElement("div");
    row.className = "joint-row";
    row.innerHTML = `
      <div>
        <div class="joint-name">${joint.name}</div>
        <div class="joint-meta">${format(joint.min_deg)}..${format(joint.max_deg)} deg</div>
      </div>
      <div>
        <input class="joint-slider" type="range" min="${joint.min_deg}" max="${joint.max_deg}" step="0.1" data-index="${index}" />
        <div class="joint-values">
          <span>Preview <strong id="target-${index}">0.0 deg</strong></span>
          <span>Reported <strong id="reported-${index}">0.0 deg</strong></span>
        </div>
      </div>
      <input class="angle-input" type="number" min="${joint.min_deg}" max="${joint.max_deg}" step="0.1" data-index="${index}" aria-label="${joint.name} angle" />
    `;
    elements.jointControls.appendChild(row);
  });
}

function buildPerJointTuning() {
  elements.perJointTuning.innerHTML = "";
  state.config.joints.forEach((joint, index) => {
    const row = document.createElement("div");
    row.className = "tuning-row";
    row.innerHTML = `
      <div class="calibration-joint"><strong>${joint.name}</strong><span>J${index + 1}</span></div>
      <label>Maximum speed
        <span class="input-with-unit"><input class="joint-speed-limit" data-index="${index}" type="number" min="0.1" step="1" value="${format(joint.max_speed_deg_s, 1)}" /><span>deg/s</span></span>
      </label>
      <label>Maximum acceleration
        <span class="input-with-unit"><input class="joint-accel-limit" data-index="${index}" type="number" min="0.1" step="5" value="${format(joint.max_accel_deg_s2, 1)}" /><span>deg/s²</span></span>
      </label>
    `;
    elements.perJointTuning.appendChild(row);
  });
}

function clonePlain(value) {
  return JSON.parse(JSON.stringify(value || {}));
}

const robotSettingsScopes = new Set(["geometry", "joints", "motion", "tooling", "hardware", "task_presets"]);

function savedSettingsDetail() {
  if (state.robotState?.simulation) return "Saved locally. Controller sync is not required in simulation.";
  if (!state.robotState?.connected) return "Saved locally. Connect the controller to sync hardware settings.";
  const change = state.robotState?.config_change || {};
  if (change.pose_revalidation_required) {
    return state.robotState?.config_sync_status === "synced"
      ? "Controller synced. Use Set Pose while disarmed to revalidate the current physical pose."
      : "Saved locally. Disarm, sync the controller, then use Set Pose to revalidate the current physical pose.";
  }
  const syncStatus = state.robotState?.config_sync_status || "unknown";
  if (syncStatus === "synced") return "Saved locally and synced to the controller.";
  return `Saved locally. Controller sync: ${syncStatus}.`;
}

function updateSettingsSaveBar(options = {}) {
  if (!elements.settingsSaveBar) return;
  const dirty = state.settingsDirtyScopes.size > 0;
  const mode = options.mode || (dirty ? "dirty" : "saved");
  elements.settingsSaveBar.classList.toggle("dirty", mode === "dirty");
  elements.settingsSaveBar.classList.toggle("saving", mode === "saving");
  elements.settingsSaveBar.classList.toggle("error", mode === "error");
  if (elements.settingsSaveStatus) {
    elements.settingsSaveStatus.textContent =
      options.title || (dirty ? "Unsaved settings changes" : "All settings saved");
  }
  if (elements.calibrationStatus) {
    elements.calibrationStatus.textContent =
      options.detail ||
      (dirty
        ? "Save all settings to apply the current drafts, or discard them to reload the saved configuration."
        : savedSettingsDetail());
  }
  if (elements.saveCalibrationBtn) {
    elements.saveCalibrationBtn.disabled = !dirty || mode === "saving";
  }
  if (elements.discardSettingsBtn) {
    elements.discardSettingsBtn.disabled = !dirty || mode === "saving";
  }
}

function markSettingsDirty(scope, detail = null) {
  state.settingsDirtyScopes.add(scope);
  if (["motion", "geometry", "tooling", "hardware", "camera"].includes(scope)) {
    invalidateTaskDetections(`${scope} settings changed - refresh detections`);
  }
  updateSettingsSaveBar({
    mode: "dirty",
    detail: detail || `${scope} settings changed. Save all settings to persist the draft.`,
  });
}

function clearSettingsDirty(scope = null) {
  if (scope) state.settingsDirtyScopes.delete(scope);
  else state.settingsDirtyScopes.clear();
  updateSettingsSaveBar();
}

function activeGeometryPreset() {
  const geometry = state.config?.geometry || {};
  const presets = geometry.presets || {};
  const names = Object.keys(presets);
  const active = geometry.active_preset || names[0] || "matlab_prototype";
  return { geometry, active, preset: presets[active] || {} };
}

function buildGeometryPresetEditor() {
  if (!elements.geometryPresetEditor) return;
  const { geometry, active, preset } = activeGeometryPreset();
  const presets = geometry.presets || {};
  const names = Object.keys(presets).length ? Object.keys(presets) : [active];
  const dimensions = preset.dimensions_mm || {};
  const signs = preset.signs || {};
  const presetControl = names.length > 1
    ? `
      <label>Geometry preset
        <select id="geometryPresetSelect">
          ${names.map((name) => `<option value="${name}" ${name === active ? "selected" : ""}>${presets[name]?.label || name}</option>`).join("")}
        </select>
      </label>
    `
    : `<select id="geometryPresetSelect" hidden><option value="${active}" selected>${active}</option></select>`;
  elements.geometryPresetEditor.innerHTML = `
    <div class="geometry-header">
      ${presetControl}
    </div>
    <div class="geometry-grid">
      ${geometryDimensionFields.map(([key, label, description]) => `
        <label>
          <span class="geometry-field-label"><strong>${label}</strong><small>${description}</small></span>
          <input data-geometry-dimension="${key}" type="number" min="0" step="0.01" value="${format(dimensions[key], 2)}" />
        </label>
      `).join("")}
    </div>
    <div class="geometry-sign-grid">
      ${geometrySignFields.map(([key, label]) => `
        <label>${label} (${key})
          <select data-geometry-sign="${key}">
            <option value="1" ${Number(signs[key] ?? 1) === 1 ? "selected" : ""}>Positive (+)</option>
            <option value="-1" ${Number(signs[key] ?? 1) === -1 ? "selected" : ""}>Negative (-)</option>
          </select>
        </label>
      `).join("")}
      <button id="applyGeometryPresetBtn" class="ghost" type="button">Preview geometry</button>
    </div>
  `;
}

function readGeometryPayload() {
  const geometry = clonePlain(state.config?.geometry || {});
  geometry.presets = geometry.presets || {};
  const select = $("#geometryPresetSelect");
  const active = select?.value || geometry.active_preset || Object.keys(geometry.presets)[0] || "matlab_prototype";
  geometry.active_preset = active;
  const preset = clonePlain(geometry.presets[active]);
  preset.label = preset.label || active;
  preset.status = preset.status || "working_assumption";
  preset.units = preset.units || { length: "mm", angle: "deg" };
  preset.dimensions_mm = preset.dimensions_mm || {};
  document.querySelectorAll("[data-geometry-dimension]").forEach((input) => {
    preset.dimensions_mm[input.dataset.geometryDimension] = readNumber(input, 0);
  });
  preset.signs = preset.signs || {};
  document.querySelectorAll("[data-geometry-sign]").forEach((input) => {
    preset.signs[input.dataset.geometrySign] = Number(input.value) === -1 ? -1 : 1;
  });
  geometry.presets[active] = preset;
  return geometry;
}

function dhRowsFromGeometryPreset(preset) {
  const dimensions = preset.dimensions_mm || {};
  const signs = preset.signs || {};
  const limits = preset.joint_limits_deg || {};
  const length = (name) => Number(dimensions[name] || 0);
  const sign = (name) => (Number(signs[name] ?? 1) === -1 ? -1 : 1);
  const limit = (name, side, fallback) => Number(limits[name]?.[side] ?? fallback);
  return [
    {
      joint: 1,
      theta_offset_deg: 0.0,
      d_mm: length("L_1") + length("L_3"),
      a_mm: 0.0,
      alpha_deg: 90.0,
      joint_type: "revolute",
      min_deg: limit("theta1", "min", -180.0),
      max_deg: limit("theta1", "max", 180.0),
      zero_offset_deg: 0.0,
      direction_sign: 1,
    },
    {
      joint: 2,
      theta_offset_deg: 0.0,
      d_mm: sign("s4") * length("L_4"),
      a_mm: length("L_5"),
      alpha_deg: 0.0,
      joint_type: "revolute",
      min_deg: limit("theta2", "min", -90.0),
      max_deg: limit("theta2", "max", 160.0),
      zero_offset_deg: 0.0,
      direction_sign: 1,
    },
    {
      joint: 3,
      theta_offset_deg: 0.0,
      d_mm: sign("s6") * length("L_6"),
      a_mm: length("L_7"),
      alpha_deg: 0.0,
      joint_type: "revolute",
      min_deg: limit("theta3", "min", -160.0),
      max_deg: limit("theta3", "max", 160.0),
      zero_offset_deg: 0.0,
      direction_sign: 1,
    },
    {
      joint: 4,
      theta_offset_deg: 0.0,
      d_mm: sign("s8") * length("L_8"),
      a_mm: length("L_9"),
      alpha_deg: 0.0,
      joint_type: "revolute",
      min_deg: limit("theta4", "min", -180.0),
      max_deg: limit("theta4", "max", 180.0),
      zero_offset_deg: 0.0,
      direction_sign: 1,
    },
  ];
}

function linksFromGeometryPreset(preset) {
  const dimensions = preset.dimensions_mm || {};
  return {
    base_height: Number(dimensions.L_1 || 0) + Number(dimensions.L_3 || 0),
    base_side_offset: Number(dimensions.L_2 || 0),
    upper_arm: Number(dimensions.L_5 || 0),
    forearm: Number(dimensions.L_7 || 0),
    wrist: Number(dimensions.L_9 || 0),
    tool: 0.0,
  };
}

function geometryDraft() {
  const geometry = readGeometryPayload();
  const preset = geometry.presets?.[geometry.active_preset] || {};
  const links = linksFromGeometryPreset(preset);
  const dhRows = dhRowsFromGeometryPreset(preset);
  return { geometry, preset, links, dhRows };
}

function readLinkPayload() {
  return geometryDraft().links;
}

function readDhRowsFromEditor() {
  return geometryDraft().dhRows;
}

function validateDhDraft() {
  if (!elements.dhTableEditor) return { ok: true, rows: [], errors: [] };
  const rows = readDhRowsFromEditor();
  const errors = [];
  rows.forEach((row, index) => {
    const rowElement = elements.dhTableEditor.querySelector(`[data-dh-row="${index}"]`);
    const numericFields = ["theta_offset_deg", "d_mm", "a_mm", "alpha_deg", "min_deg", "max_deg", "zero_offset_deg"];
    const badNumber = numericFields.some((field) => !Number.isFinite(Number(row[field])));
    const badRange = Number(row.min_deg) >= Number(row.max_deg);
    const badDirection = ![-1, 1].includes(Number(row.direction_sign));
    const bad = badNumber || badRange || badDirection;
    rowElement?.classList.toggle("invalid", bad);
    if (badNumber) errors.push(`J${index + 1} has a non-numeric value`);
    if (badRange) errors.push(`J${index + 1} min must be below max`);
    if (badDirection) errors.push(`J${index + 1} direction must be +1 or -1`);
  });
  if (elements.dhTableStatus) {
    elements.dhTableStatus.textContent = errors.length
      ? errors.join("; ")
      : "Derived model is valid. Save to use it for FK and IK.";
  }
  return { ok: errors.length === 0, rows, errors };
}

function previewDhDraft() {
  const validation = validateDhDraft();
  if (!validation.ok) return;
  const draftConfig = clonePlain(state.config);
  const draft = geometryDraft();
  draftConfig.geometry = draft.geometry;
  draftConfig.links_mm = {
    ...(draftConfig.links_mm || {}),
    ...draft.links,
    dh_rows: validation.rows,
  };
  state.view.setConfig(draftConfig);
  state.view.setAngles(jointControlAngles() || state.view.angles);
  if (elements.dhTableStatus) elements.dhTableStatus.textContent = "Model draft is shown in the viewport. Backend FK/IK changes after Save.";
}

function renderDerivedModelSummary(links, preset = {}) {
  if (!elements.linkCalibration) return;
  const dimensions = preset.dimensions_mm || {};
  const signs = preset.signs || {};
  const length = (name) => Number(dimensions[name] || 0);
  const sign = (name) => (Number(signs[name] ?? 1) === -1 ? -1 : 1);
  elements.linkCalibration.innerHTML = `
    <div class="log-line"><span>Base view</span><code>L1 ${format(length("L_1"), 2)} mm -> L2 bend ${format(length("L_2"), 2)} mm -> L3 ${format(length("L_3"), 2)} mm</code></div>
    <div class="log-line"><span>d1 / side</span><code>${format(links.base_height, 2)} mm = L1 + L3, ${format(length("L_2"), 2)} mm side = L2</code></div>
    <div class="log-line"><span>d2 / a2</span><code>${format(sign("s4") * length("L_4"), 2)} mm = s4*L4, ${format(links.upper_arm, 2)} mm = L5</code></div>
    <div class="log-line"><span>d3 / a3</span><code>${format(sign("s6") * length("L_6"), 2)} mm = s6*L6, ${format(links.forearm, 2)} mm = L7</code></div>
    <div class="log-line"><span>d4 / a4</span><code>${format(sign("s8") * length("L_8"), 2)} mm = s8*L8, ${format(links.wrist, 2)} mm = L9</code></div>
  `;
}

function renderDhRows(rows) {
  if (!elements.dhTableEditor) return;
  elements.dhTableEditor.innerHTML = `
    <div class="dh-table-wrap">
      <table class="dh-grid dh-grid-readonly">
        <thead>
          <tr>
            <th>Joint</th>
            <th>Theta offset deg</th>
            <th>d mm</th>
            <th>a mm</th>
            <th>Alpha deg</th>
            <th>Min deg</th>
            <th>Max deg</th>
            <th>Zero deg</th>
            <th>Dir</th>
          </tr>
        </thead>
        <tbody>
          ${rows.map((row, index) => `
            <tr data-dh-row="${index}">
              <td><strong>J${index + 1}</strong></td>
              <td><code>${format(row.theta_offset_deg, 2)}</code></td>
              <td><code>${format(row.d_mm, 2)}</code></td>
              <td><code>${format(row.a_mm, 2)}</code></td>
              <td><code>${format(row.alpha_deg, 2)}</code></td>
              <td><code>${format(row.min_deg, 1)}</code></td>
              <td><code>${format(row.max_deg, 1)}</code></td>
              <td><code>${format(row.zero_offset_deg, 2)}</code></td>
              <td><code>${Number(row.direction_sign) === -1 ? "-1" : "+1"}</code></td>
            </tr>
          `).join("")}
        </tbody>
      </table>
    </div>
  `;
}

function refreshDerivedModelDraft() {
  const draft = geometryDraft();
  state.linkDraft = draft.links;
  state.dhDraftRows = draft.dhRows;
  renderDerivedModelSummary(draft.links, draft.preset);
  renderDhRows(draft.dhRows);
  return validateDhDraft();
}

function applyGeometryPresetToDhDraft() {
  const validation = refreshDerivedModelDraft();
  if (validation.ok) previewDhDraft();
  markSettingsDirty("geometry", "Geometry preview updated in the viewport. Save all settings to use it for FK and IK.");
}

function toolTypeLabel(type) {
  if (type === "electromagnet") return "Magnet";
  if (type === "servo_gripper") return "Gripper";
  return "Generic";
}

function toolPresetIsMagnet(name, preset = null) {
  const activePreset = preset || state.config?.tools?.presets?.[name] || {};
  return name === "magnet" || activePreset.type === "electromagnet";
}

function renderToolSelectOptions(activeOverride = null) {
  if (!elements.toolSelect) return;
  const tools = state.config?.tools || { active: "gripper", presets: {} };
  const presets = tools.presets || {};
  const active = activeOverride || tools.active || Object.keys(presets)[0] || elements.toolSelect.value || "gripper";
  elements.toolSelect.innerHTML = "";
  const entries = Object.keys(presets).length
    ? Object.entries(presets)
    : [
        ["gripper", { type: "servo_gripper" }],
        ["magnet", { type: "electromagnet" }],
      ];
  entries.forEach(([name, preset]) => {
    const option = document.createElement("option");
    option.value = name;
    option.textContent = preset.label || toolTypeLabel(preset.type) || name;
    elements.toolSelect.appendChild(option);
  });
  if (!entries.some(([name]) => name === active)) {
    const option = document.createElement("option");
    option.value = active;
    option.textContent = active;
    elements.toolSelect.appendChild(option);
  }
  elements.toolSelect.value = active;
}

function renderToolEditor() {
  if (!elements.toolCalibration || !state.config) return;
  const tools = state.config.tools || { active: "gripper", presets: {} };
  const active = tools.active || "gripper";
  const presets = tools.presets || {};
  elements.toolCalibration.classList.add("tool-editor");
  elements.toolCalibration.innerHTML = Object.entries(presets)
    .map(([name, preset]) => {
      const tcp = preset.tcp_offset_mm || {};
      const io = preset.io || {};
      const type = preset.type || "generic";
      const isMagnet = type === "electromagnet";
      const specificFields = isMagnet
        ? `
          <label>Magnet GPIO <input data-tool-preset="${name}" data-tool-io-field="pin" type="number" step="1" value="${io.pin ?? -1}" /></label>
          <label class="toggle-label compact-toggle">
            <input data-tool-preset="${name}" data-tool-io-field="active_high" type="checkbox" ${io.active_high !== false ? "checked" : ""} />
            <span>Active high</span>
          </label>
        `
        : `
          <label>Open value <input data-tool-preset="${name}" data-tool-field="open_value" type="number" min="0" max="1" step="0.01" value="${format(preset.open_value ?? 0, 2)}" /></label>
          <label>Closed value <input data-tool-preset="${name}" data-tool-field="closed_value" type="number" min="0" max="1" step="0.01" value="${format(preset.closed_value ?? 1, 2)}" /></label>
          <label>PWM GPIO <input data-tool-preset="${name}" data-tool-io-field="pwm_pin" type="number" step="1" value="${io.pwm_pin ?? -1}" /></label>
          <label>Min us <input data-tool-preset="${name}" data-tool-io-field="pulse_min_us" type="number" min="100" step="10" value="${io.pulse_min_us ?? 500}" /></label>
          <label>Max us <input data-tool-preset="${name}" data-tool-io-field="pulse_max_us" type="number" min="100" step="10" value="${io.pulse_max_us ?? 2500}" /></label>
          <label>Frequency <input data-tool-preset="${name}" data-tool-io-field="pwm_frequency_hz" type="number" min="1" step="1" value="${io.pwm_frequency_hz ?? 50}" /></label>
        `;
      return `
        <div class="tool-card ${name === active ? "active" : ""}" data-tool-card="${name}">
          <div class="hardware-title">
            <strong>${preset.label || name}</strong>
            <span class="badge">${name === active ? "active" : type}</span>
          </div>
          <div class="tool-grid">
            <label>Name <input data-tool-preset="${name}" data-tool-field="label" type="text" value="${preset.label || name}" /></label>
            <label>Type
              <select data-tool-preset="${name}" data-tool-field="type">
                <option value="servo_gripper" ${type === "servo_gripper" ? "selected" : ""}>Gripper</option>
                <option value="electromagnet" ${type === "electromagnet" ? "selected" : ""}>Magnet</option>
                <option value="generic" ${type === "generic" ? "selected" : ""}>Generic</option>
              </select>
            </label>
            <label>TCP X mm <input data-tool-preset="${name}" data-tool-tcp-field="x" type="number" step="0.1" value="${format(tcp.x ?? 0, 1)}" /></label>
            <label>TCP Y mm <input data-tool-preset="${name}" data-tool-tcp-field="y" type="number" step="0.1" value="${format(tcp.y ?? 0, 1)}" /></label>
            <label>TCP Z mm <input data-tool-preset="${name}" data-tool-tcp-field="z" type="number" step="0.1" value="${format(tcp.z ?? 0, 1)}" /></label>
            ${specificFields}
          </div>
        </div>
      `;
    })
    .join("");
}

function readToolsPayload() {
  const current = clonePlain(state.config?.tools || { active: "gripper", presets: {} });
  current.presets = current.presets || {};
  current.active = elements.toolSelect?.value || current.active || "gripper";
  elements.toolCalibration?.querySelectorAll("[data-tool-card]").forEach((card) => {
    const name = card.dataset.toolCard;
    const preset = clonePlain(current.presets[name] || {});
    preset.tcp_offset_mm = preset.tcp_offset_mm || {};
    preset.io = preset.io || {};
    card.querySelectorAll("[data-tool-field]").forEach((input) => {
      const field = input.dataset.toolField;
      if (field === "label" || field === "type") {
        preset[field] = input.value;
      } else {
        preset[field] = readNumber(input, preset[field] ?? 0);
      }
    });
    card.querySelectorAll("[data-tool-tcp-field]").forEach((input) => {
      preset.tcp_offset_mm[input.dataset.toolTcpField] = readNumber(input, 0);
    });
    card.querySelectorAll("[data-tool-io-field]").forEach((input) => {
      const field = input.dataset.toolIoField;
      preset.io[field] = input.type === "checkbox" ? input.checked : readNumber(input, 0);
    });
    current.presets[name] = preset;
  });
  return current;
}

function buildCalibrationEditors() {
  buildGeometryPresetEditor();
  refreshDerivedModelDraft();

  elements.jointCalibration.innerHTML = `
    <div class="calibration-row calibration-header" aria-hidden="true">
      <span>Joint</span>
      <span>Safe operating range</span>
      <span>Home angle</span>
      <span>Zero offset</span>
      <span>Positive direction</span>
      <span>Check</span>
    </div>
  `;
  state.config.joints.forEach((joint, index) => {
    const row = document.createElement("div");
    row.className = "calibration-row";
    row.dataset.calibrationIndex = String(index);
    row.innerHTML = `
      <div class="calibration-joint"><strong>${joint.name}</strong><span>J${index + 1}</span></div>
      <div class="calibration-range">
        <input aria-label="${joint.name} minimum safe angle" title="Minimum safe angle in degrees" data-joint-index="${index}" data-calib-limit="min" type="number" step="0.1" value="${format(joint.min_deg, 1)}" />
        <span>to</span>
        <input aria-label="${joint.name} maximum safe angle" title="Maximum safe angle in degrees" data-joint-index="${index}" data-calib-limit="max" type="number" step="0.1" value="${format(joint.max_deg, 1)}" />
      </div>
      <label class="calibration-cell">
        <span>Home angle</span>
        <input aria-label="${joint.name} home angle" title="Expected joint angle at the home pose" data-joint-index="${index}" data-calib-field="home_deg" type="number" step="0.1" value="${format(joint.home_deg, 1)}" />
      </label>
      <label class="calibration-cell">
        <span>Zero offset</span>
        <input aria-label="${joint.name} zero offset" title="Offset between mechanism zero and software zero" data-joint-index="${index}" data-calib-field="zero_offset_deg" type="number" step="0.1" value="${format(joint.zero_offset_deg, 1)}" />
      </label>
      <label class="calibration-cell">
        <span>Positive direction</span>
        <select aria-label="${joint.name} positive direction" title="Invert if positive commands move toward negative joint angles" data-joint-index="${index}" data-calib-field="direction_sign">
          <option value="1" ${joint.direction_sign === 1 ? "selected" : ""}>Normal (+)</option>
          <option value="-1" ${joint.direction_sign === -1 ? "selected" : ""}>Inverted (-)</option>
        </select>
      </label>
      <span class="calibration-check">Valid</span>
    `;
    elements.jointCalibration.appendChild(row);
  });
  validateJointCalibrationDraft();

  renderToolEditor();

  if (elements.encoderCalibration) {
    const encoders = state.config.encoders || {};
    elements.encoderCalibration.innerHTML = (encoders.axes || [])
      .map((axis, index) => {
        const reported = state.robotState?.encoder_angles_deg?.[index];
        const err = state.robotState?.encoder_errors_deg?.[index];
        return `<div class="log-line"><span>${axis.name || `J${index + 1}`} encoder</span><code>cs ${axis.cs_pin}, zero ${format(axis.zero_offset_deg, 2)}, angle ${reported == null ? "-" : format(reported, 2)}, err ${err == null ? "-" : format(err, 2)}</code></div>`;
      })
      .join("");
  }
}

function validateJointCalibrationDraft() {
  if (!elements.jointCalibration || !state.config) return { ok: true, errors: [] };
  const errors = [];
  state.config.joints.forEach((joint, index) => {
    const minInput = $(`[data-joint-index="${index}"][data-calib-limit="min"]`);
    const maxInput = $(`[data-joint-index="${index}"][data-calib-limit="max"]`);
    const homeInput = $(`[data-joint-index="${index}"][data-calib-field="home_deg"]`);
    const zeroInput = $(`[data-joint-index="${index}"][data-calib-field="zero_offset_deg"]`);
    const row = elements.jointCalibration.querySelector(`[data-calibration-index="${index}"]`);
    const check = row?.querySelector(".calibration-check");
    const minimum = Number(minInput?.value);
    const maximum = Number(maxInput?.value);
    const home = Number(homeInput?.value);
    const zero = Number(zeroInput?.value);
    let message = "Valid";
    if (![minimum, maximum, home, zero].every(Number.isFinite)) {
      message = "Enter numbers";
    } else if (minimum >= maximum) {
      message = "Min ≥ max";
    } else if (home < minimum || home > maximum) {
      message = "Home outside range";
    }
    const valid = message === "Valid";
    row?.classList.toggle("invalid", !valid);
    if (check) check.textContent = message;
    if (!valid) errors.push(`${joint.name}: ${message}`);
  });
  return { ok: errors.length === 0, errors };
}

function buildHardwareIoEditors() {
  elements.hardwareIo.innerHTML = "";
  state.config.joints.forEach((joint, index) => {
    const row = document.createElement("div");
    row.className = "hardware-row";
    row.dataset.hardwareIndex = String(index);
    const axisState = state.robotState?.hardware_axis_states?.[index] || "simulated";
    if (joint.actuator === "servo") {
      const servo = joint.hardware?.servo || {};
      row.innerHTML = `
        <div class="hardware-title">
          <strong>${joint.name} servo</strong>
          <span class="badge">${axisState}</span>
        </div>
        <label class="toggle-label compact-toggle">
          <input data-hw-index="${index}" data-hw-kind="servo" data-hw-field="enabled" type="checkbox" ${servo.enabled ? "checked" : ""} />
          <span>Enabled</span>
        </label>
        <div class="hardware-grid">
          <label>PWM GPIO <input data-hw-index="${index}" data-hw-kind="servo" data-hw-field="pwm_pin" type="number" step="1" value="${servo.pwm_pin ?? -1}" /></label>
          <label>Min us <input data-hw-index="${index}" data-hw-kind="servo" data-hw-field="pulse_min_us" type="number" min="100" step="10" value="${servo.pulse_min_us ?? 500}" /></label>
          <label>Max us <input data-hw-index="${index}" data-hw-kind="servo" data-hw-field="pulse_max_us" type="number" min="100" step="10" value="${servo.pulse_max_us ?? 2500}" /></label>
          <label>Frequency <input data-hw-index="${index}" data-hw-kind="servo" data-hw-field="pwm_frequency_hz" type="number" min="1" step="1" value="${servo.pwm_frequency_hz ?? 50}" /></label>
          <label>Range deg <input data-hw-index="${index}" data-hw-kind="servo" data-hw-field="servo_range_deg" type="number" min="1" step="1" value="${servo.servo_range_deg ?? 270}" /></label>
          <label>Neutral deg <input data-hw-index="${index}" data-hw-kind="servo" data-hw-field="neutral_deg" type="number" step="0.1" value="${servo.neutral_deg ?? 135}" /></label>
          <label>Gear ratio <input data-hw-index="${index}" data-hw-kind="servo" data-hw-field="gear_ratio" type="number" min="0.0001" step="0.001" value="${servo.gear_ratio ?? 1}" /></label>
        </div>
      `;
    } else {
      const stepper = joint.hardware?.stepper || {};
      row.innerHTML = `
        <div class="hardware-title">
          <strong>${joint.name} stepper</strong>
          <span class="badge">${axisState}</span>
        </div>
        <div class="button-row">
          <label class="toggle-label compact-toggle">
            <input data-hw-index="${index}" data-hw-kind="stepper" data-hw-field="enabled" type="checkbox" ${stepper.enabled ? "checked" : ""} />
            <span>Enabled</span>
          </label>
          <label class="toggle-label compact-toggle">
            <input data-hw-index="${index}" data-hw-kind="stepper" data-hw-field="enable_active_low" type="checkbox" ${stepper.enable_active_low !== false ? "checked" : ""} />
            <span>Enable active low</span>
          </label>
        </div>
        <div class="hardware-grid">
          <label>STEP GPIO <input data-hw-index="${index}" data-hw-kind="stepper" data-hw-field="step_pin" type="number" step="1" value="${stepper.step_pin ?? -1}" /></label>
          <label>DIR GPIO <input data-hw-index="${index}" data-hw-kind="stepper" data-hw-field="dir_pin" type="number" step="1" value="${stepper.dir_pin ?? -1}" /></label>
          <label>ENABLE GPIO <input data-hw-index="${index}" data-hw-kind="stepper" data-hw-field="enable_pin" type="number" step="1" value="${stepper.enable_pin ?? -1}" /></label>
          <label>Driver <input data-hw-index="${index}" data-hw-kind="stepper" data-hw-field="driver_model" type="text" value="${stepper.driver_model || "TB6600"}" /></label>
          <label>Full steps/rev <input data-hw-index="${index}" data-hw-kind="stepper" data-hw-field="motor_full_steps_per_rev" type="number" min="1" step="1" value="${stepper.motor_full_steps_per_rev ?? 200}" /></label>
          <label>Microsteps <input data-hw-index="${index}" data-hw-kind="stepper" data-hw-field="microsteps" type="number" min="1" step="1" value="${stepper.microsteps ?? 16}" /></label>
          <label>Gear ratio <input data-hw-index="${index}" data-hw-kind="stepper" data-hw-field="gear_ratio" type="number" min="0.0001" step="0.001" value="${stepper.gear_ratio ?? 1}" /></label>
        </div>
      `;
    }
    elements.hardwareIo.appendChild(row);
  });
}

function renderHardwareStatus(robotState = state.robotState) {
  if (!elements.hardwareStatus || !robotState) return;
  const axisText = (robotState.hardware_axis_states || []).map((value, index) => `J${index + 1}:${value}`).join(" ");
  const change = robotState.config_change || {};
  const categories = Array.isArray(change.categories) ? change.categories.join(", ") : "";
  elements.hardwareStatus.innerHTML = `
    <div class="log-line"><span>Coverage</span><code>${robotState.hardware_mode || "simulated"} (${robotState.hardware_enabled_axes || "0000"})</code></div>
    <div class="log-line"><span>Sync</span><code>${robotState.config_sync_status || "unknown"}</code></div>
    <div class="log-line"><span>Axes</span><code>${axisText || "-"}</code></div>
    <div class="log-line"><span>Message</span><code>${robotState.config_sync_message || "-"}</code></div>
    <div class="log-line"><span>Last config impact</span><code>${categories || "none"}</code></div>
    <div class="log-line"><span>Pose revalidation</span><code>${change.pose_revalidation_required ? "required - use Set Pose while disarmed" : "not required"}</code></div>
  `;
  if (state.hardwareDraftDirty) {
    renderHardwareDraftBadges();
    return;
  }
  elements.hardwareIo.querySelectorAll("[data-hardware-index]").forEach((row) => {
    const index = Number(row.dataset.hardwareIndex);
    const badge = row.querySelector(".hardware-title .badge");
    if (badge) badge.textContent = robotState.hardware_axis_states?.[index] || "simulated";
  });
}

function invalidateTaskPreview(reason = "") {
  state.taskPreviewId = null;
  state.taskPreviewCreatedAt = null;
  state.lastTaskPreview = null;
  state.taskLocalStatusAt = Date.now() / 1000;
  if (elements.executeTaskBtn) elements.executeTaskBtn.disabled = true;
  if (elements.taskStatus && reason) elements.taskStatus.textContent = reason;
  if (elements.taskPlanPreview) elements.taskPlanPreview.innerHTML = "";
  state.view?.setTaskPreview?.(null);
}

function invalidateTaskDetections(reason = "Refresh detections before planning") {
  state.latestDetections = [];
  state.taskDetectionsCapturedAt = null;
  state.selectedDetectionIds.clear();
  state.view?.setObjectDetections([]);
  renderDetectionList([]);
  invalidateTaskPreview(reason);
}

function normalizedColorName(value) {
  return String(value || "").trim().toLowerCase();
}

function detectionColor(detection) {
  return normalizedColorName(detection?.label || detection?.color);
}

function ensureTaskPresetDrafts() {
  if (!state.config) return;
  if (!state.taskColorProfilesDraft) {
    state.taskColorProfilesDraft = clonePlain(state.config.color_profiles || {});
  }
  if (!state.taskDropZonesDraft) {
    state.taskDropZonesDraft = clonePlain(state.config.drop_zones || {});
  }
}

function taskColorProfiles() {
  ensureTaskPresetDrafts();
  return state.taskColorProfilesDraft || {};
}

function taskDropZones() {
  ensureTaskPresetDrafts();
  return state.taskDropZonesDraft || {};
}

function defaultDropZoneName() {
  const zones = taskDropZones();
  return Object.keys(zones)[0] || "";
}

function taskPresetDraftChanged() {
  if (!state.config) return false;
  return (
    JSON.stringify(state.taskColorProfilesDraft || state.config.color_profiles || {}) !== JSON.stringify(state.config.color_profiles || {})
    || JSON.stringify(state.taskDropZonesDraft || state.config.drop_zones || {}) !== JSON.stringify(state.config.drop_zones || {})
    || state.unsavedColorProfiles.size > 0
  );
}

function ensureDetectedColorDrafts(detections = state.latestDetections, options = {}) {
  ensureTaskPresetDrafts();
  const profiles = taskColorProfiles();
  let changed = false;
  (detections || []).forEach((detection) => {
    const color = detectionColor(detection);
    if (!color || profiles[color]) return;
    profiles[color] = {
      enabled: true,
      drop_zone: "",
      draft: true,
    };
    state.unsavedColorProfiles.add(color);
    changed = true;
  });
  if (changed) {
    if (options.markDirty) {
      markSettingsDirty("task_presets", "Detected new colors. Assign drop presets and save before running.");
    }
    renderColorPresetMapping();
    renderTaskPresetEditors();
  }
  return changed;
}

function colorProfileOverridesPayload() {
  const profiles = taskColorProfiles();
  return Object.fromEntries(
    Object.entries(profiles).map(([name, profile]) => [
      normalizedColorName(name),
      {
        enabled: profile.enabled !== false,
        drop_zone: profile.drop_zone || "",
        draft: Boolean(profile.draft || state.unsavedColorProfiles.has(name)),
      },
    ])
  );
}

function taskDraftBlocksRun() {
  if (taskPresetDraftChanged()) return true;
  const zones = taskDropZones();
  return Object.entries(taskColorProfiles()).some(([, profile]) => {
    if (profile.enabled === false) return false;
    return !profile.drop_zone || !zones[profile.drop_zone];
  });
}

function selectedColorFilters() {
  const checked = [...document.querySelectorAll("[data-color-enabled]")]
    .filter((input) => input.checked)
    .map((input) => normalizedColorName(input.dataset.colorEnabled))
    .filter((color) => Boolean(taskColorProfiles()[color]?.drop_zone))
    .filter(Boolean);
  if (elements.colorPresetMapping?.querySelector("[data-color-row]")) return checked.length ? checked : ["__none__"];
  return [...(elements.includeColorsSelect?.selectedOptions || [])].map((option) => option.value);
}

function objectProfilesPayload() {
  const raw = elements.objectProfilesInput?.value?.trim();
  if (!raw) return {};
  try {
    const parsed = JSON.parse(raw);
    return parsed && typeof parsed === "object" && !Array.isArray(parsed) ? parsed : {};
  } catch (error) {
    throw new Error(`Object profile overrides must be valid JSON: ${error.message}`);
  }
}

function taskSettingsPayload() {
  return {
    execution_strategy: elements.executionStrategySelect?.value || "closed_loop",
    max_objects: readRequiredNumber(elements.maxObjectsInput, "Max objects", { min: 1, integer: true }),
    filters: {
      min_confidence: readRequiredNumber(elements.minConfidenceInput, "Min confidence", { min: 0, max: 1 }),
      min_area_px: 0,
      include_colors: selectedColorFilters(),
      require_robot_coordinates: true,
    },
    ordering: {
      policy: elements.objectSelectionPolicySelect?.value || "nearest_to_safe",
      color_priority: (elements.colorPriorityInput?.value || "")
        .split(",")
        .map((item) => item.trim())
        .filter(Boolean),
    },
    missing_drop_zone_policy: elements.missingDropzonePolicySelect?.value || "error",
    unknown_color_policy: elements.unknownColorPolicySelect?.value || "ignore",
    placement_policy: elements.placementPolicySelect?.value || "fixed",
    pickup_z_mm: readRequiredNumber(elements.pickupZInput, "Pickup Z", { min: 0 }),
    dropoff_z_mm: readRequiredNumber(elements.dropoffZInput, "Dropoff Z", { min: 0 }),
    approach_clearance_mm: readRequiredNumber(elements.approachClearanceInput, "Pickup clearance", { min: 0 }),
    drop_approach_clearance_mm: readRequiredNumber(elements.dropApproachClearanceInput, "Drop clearance", { min: 0 }),
    orientation_policy: elements.orientationPolicySelect?.value || "prefer_downward",
    downward_phi_deg: -90,
    pickup_preferred_phi_deg: -90,
    drop_preferred_phi_deg: -90,
    pickup_phi_deg: readRequiredNumber(elements.pickupPhiInput, "Pickup phi"),
    drop_phi_deg: readRequiredNumber(elements.dropPhiInput, "Drop phi"),
    motion_modes: {
      transfer: elements.transferModeSelect?.value || "joint",
      pickup_approach: "linear",
      pickup_descent: elements.pickupDescentModeSelect?.value || "linear",
      lift: elements.liftModeSelect?.value || "linear",
      drop_approach: "linear",
      drop_descent: elements.dropDescentModeSelect?.value || "linear",
    },
    capture_settle_ms: readRequiredNumber(elements.captureSettleInput, "Capture settle", { min: 0, integer: true }),
    tool_settle_ms: readRequiredNumber(elements.toolSettleInput, "Tool settle", { min: 0, integer: true }),
    object_profiles: objectProfilesPayload(),
    color_profile_overrides: colorProfileOverridesPayload(),
    _has_unsaved_color_profiles: taskDraftBlocksRun(),
  };
}

function applyTaskSettingsControls() {
  const defaults = state.config?.tasks?.color_sorting || {};
  const filters = defaults.filters || {};
  const ordering = defaults.ordering || {};
  const modes = defaults.motion_modes || {};
  const setValue = (element, value) => {
    if (!element || value === undefined || value === null) return;
    element.value = String(value);
  };
  setValue(elements.executionStrategySelect, defaults.execution_strategy || "closed_loop");
  setValue(elements.objectSelectionPolicySelect, ordering.policy || "nearest_to_safe");
  setValue(elements.maxObjectsInput, defaults.max_objects ?? 10);
  setValue(elements.minConfidenceInput, filters.min_confidence ?? 0);
  setValue(elements.colorPriorityInput, (ordering.color_priority || []).join(", "));
  setValue(elements.missingDropzonePolicySelect, defaults.missing_drop_zone_policy || "error");
  setValue(elements.unknownColorPolicySelect, defaults.unknown_color_policy || "ignore");
  setValue(elements.placementPolicySelect, defaults.placement_policy || "fixed");
  setValue(elements.pickupZInput, defaults.pickup_z_mm ?? 25);
  setValue(elements.dropoffZInput, defaults.dropoff_z_mm ?? 45);
  setValue(elements.approachClearanceInput, defaults.approach_clearance_mm ?? 55);
  setValue(elements.dropApproachClearanceInput, defaults.drop_approach_clearance_mm ?? 35);
  setValue(elements.orientationPolicySelect, defaults.orientation_policy || "prefer_downward");
  setValue(elements.pickupPhiInput, defaults.pickup_phi_deg ?? 0);
  setValue(elements.dropPhiInput, defaults.drop_phi_deg ?? 0);
  setValue(elements.transferModeSelect, modes.transfer || "joint");
  setValue(elements.pickupDescentModeSelect, modes.pickup_descent || "linear");
  setValue(elements.liftModeSelect, modes.lift || "linear");
  setValue(elements.dropDescentModeSelect, modes.drop_descent || "linear");
  setValue(elements.captureSettleInput, defaults.capture_settle_ms ?? 250);
  setValue(elements.toolSettleInput, defaults.tool_settle_ms ?? 150);
  if (elements.objectProfilesInput) {
    const objectProfiles = defaults.object_profiles || {};
    elements.objectProfilesInput.value = Object.keys(objectProfiles).length
      ? JSON.stringify(objectProfiles, null, 2)
      : "";
  }
}

function renderSetupChecklist() {
  if (!elements.taskSetupChecklist) return;
  const camera = state.config?.camera || {};
  const calibration = state.config?.calibration || {};
  const robot = state.robotState || {};
  const profiles = state.config?.color_profiles || {};
  const zones = state.config?.drop_zones || {};
  const enabledProfiles = Object.entries(profiles).filter(([, profile]) => profile.enabled !== false);
  const relevantZones = enabledProfiles
    .map(([, profile]) => profile.drop_zone)
    .filter(Boolean);
  const missingZones = relevantZones.filter((zone) => !zones[zone]);
  const checks = [
    ["Robot connection", Boolean(robot.connected || robot.simulation), robot.simulation ? "simulation" : robot.connected ? "connected" : "not connected"],
    ["Known pose", Boolean(robot.known_pose || robot.simulation), robot.pose_source || "-"],
    ["Config sync", Boolean(robot.simulation || robot.config_sync_status === "synced"), robot.simulation ? "simulation" : robot.config_sync_status || "unknown"],
    ["Active tool", Boolean(robot.active_tool), `${robot.active_tool || "-"} / ${robot.tool_type || "-"}`],
    ["Camera", Boolean(camera.enabled), camera.enabled ? `enabled index ${camera.source_index}` : "disabled"],
    ["Tool calibration", Boolean(calibration.tool_dimensions_validated), calibration.tool_dimensions_validated ? "validated" : "not validated"],
    ["Safe pose", !state.config?.validation?.named_position_errors?.safe, state.config?.validation?.named_position_errors?.safe?.join("; ") || "valid"],
    ["Drop zones", missingZones.length === 0, missingZones.length ? `missing ${missingZones.join(", ")}` : `${Object.keys(zones).length} configured`],
  ];
  elements.taskSetupChecklist.innerHTML = checks.map(([label, ok, detail]) => `
    <div class="setup-check ${ok ? "ok" : "warn"}">
      <span>${ok ? "✓" : "!"}</span>
      <div><strong>${label}</strong><small>${detail}</small></div>
    </div>
  `).join("");
}

function colorStatusBadge(color, profile, zones) {
  const savedProfile = state.config?.color_profiles?.[color];
  const draft = profile?.draft || state.unsavedColorProfiles.has(color)
    || JSON.stringify(savedProfile || null) !== JSON.stringify(profile || null);
  if (!profile?.drop_zone || !zones[profile.drop_zone]) {
    return `<span class="missing-badge">Needs preset</span>`;
  }
  return draft ? `<span class="draft-badge">Draft</span>` : `<span class="saved-badge">Saved</span>`;
}

function zoneOptionsHtml(selected = "") {
  const zones = taskDropZones();
  const options = [`<option value="">Choose preset</option>`];
  Object.keys(zones).sort().forEach((name) => {
    options.push(`<option value="${name}" ${name === selected ? "selected" : ""}>${name}</option>`);
  });
  return options.join("");
}

function renderColorPresetMapping() {
  if (!elements.colorPresetMapping || !state.config) return;
  ensureDetectedColorDrafts(state.latestDetections);
  const profiles = taskColorProfiles();
  const zones = taskDropZones();
  const colors = new Set(Object.keys(profiles));
  state.latestDetections.forEach((detection) => {
    const color = detectionColor(detection);
    if (color) colors.add(color);
  });

  if (elements.includeColorsSelect) {
    elements.includeColorsSelect.innerHTML = "";
  }
  const sortedColors = [...colors].sort();
  if (!sortedColors.length) {
    elements.colorPresetMapping.innerHTML = `<div class="empty-state">No color profiles or detections yet.</div>`;
    return;
  }

  elements.colorPresetMapping.innerHTML = sortedColors.map((color) => {
    const profile = profiles[color] || { enabled: true, drop_zone: "" };
    const enabled = profile.enabled !== false;
    if (elements.includeColorsSelect) {
      const option = document.createElement("option");
      option.value = color;
      option.textContent = color;
      option.selected = enabled && Boolean(profile.drop_zone);
      elements.includeColorsSelect.appendChild(option);
    }
    const detectedCount = state.latestDetections.filter((detection) => detectionColor(detection) === color).length;
    return `
      <div class="color-map-row" data-color-row="${color}">
        <label class="toggle-label compact-toggle">
          <input type="checkbox" data-color-enabled="${color}" ${enabled ? "checked" : ""} />
          <span class="toggle-text">${color}</span>
        </label>
        <select data-color-drop-zone="${color}">
          ${zoneOptionsHtml(profile.drop_zone || "")}
        </select>
        <div class="color-map-title">
          <strong>${detectedCount ? `${detectedCount} detected` : "Configured"}</strong>
          <small>${profile.drop_zone || "no preset assigned"}</small>
        </div>
        ${colorStatusBadge(color, profile, zones)}
      </div>
    `;
  }).join("");
}

function renderTaskPresetEditors() {
  if (!state.config) return;
  ensureTaskPresetDrafts();
  const zones = taskDropZones();
  if (elements.dropPresetEditor) {
    const names = Object.keys(zones).sort();
    elements.dropPresetEditor.innerHTML = names.length
      ? names.map((name) => {
          const zone = zones[name] || {};
          const grid = zone.grid || {};
          return `
            <div class="drop-preset-row" data-drop-preset="${name}">
              <div class="drop-preset-title">
                <strong>${name}</strong>
                <small>${grid.rows && grid.columns ? `${grid.rows}x${grid.columns} grid` : "fixed anchor"}</small>
              </div>
              <label>X <input type="number" step="1" data-drop-zone-field="x_mm" data-drop-zone="${name}" value="${format(zone.x_mm ?? 0, 1)}" /></label>
              <label>Y <input type="number" step="1" data-drop-zone-field="y_mm" data-drop-zone="${name}" value="${format(zone.y_mm ?? 0, 1)}" /></label>
              <label>Z <input type="number" step="1" data-drop-zone-field="z_mm" data-drop-zone="${name}" value="${format(zone.z_mm ?? 0, 1)}" /></label>
              <div class="drop-grid-fields">
                <label>Rows <input type="number" min="0" step="1" data-drop-zone-field="grid.rows" data-drop-zone="${name}" value="${grid.rows ?? ""}" /></label>
                <label>Cols <input type="number" min="0" step="1" data-drop-zone-field="grid.columns" data-drop-zone="${name}" value="${grid.columns ?? ""}" /></label>
                <label>dX <input type="number" step="1" data-drop-zone-field="grid.x_spacing_mm" data-drop-zone="${name}" value="${grid.x_spacing_mm ?? ""}" /></label>
                <label>dY <input type="number" step="1" data-drop-zone-field="grid.y_spacing_mm" data-drop-zone="${name}" value="${grid.y_spacing_mm ?? ""}" /></label>
              </div>
              <button class="danger ghost" type="button" data-drop-zone-delete="${name}">Delete</button>
            </div>
          `;
        }).join("")
      : `<div class="empty-state">No drop presets configured.</div>`;
  }

  if (elements.colorProfileEditor) {
    const profiles = taskColorProfiles();
    const colors = Object.keys(profiles).sort();
    elements.colorProfileEditor.innerHTML = colors.length
      ? colors.map((color) => {
          const profile = profiles[color] || {};
          return `
            <div class="color-profile-row" data-color-profile="${color}">
              <label class="toggle-label compact-toggle">
                <input type="checkbox" data-settings-color-enabled="${color}" ${profile.enabled === false ? "" : "checked"} />
                <span class="toggle-text">${color}</span>
              </label>
              <select data-settings-color-drop-zone="${color}">
                ${zoneOptionsHtml(profile.drop_zone || "")}
              </select>
              ${colorStatusBadge(color, profile, taskDropZones())}
            </div>
          `;
        }).join("")
      : `<div class="empty-state">No color profiles yet. Run detection to create drafts.</div>`;
  }
}

function markTaskPresetDraftDirty(detail = "Task presets changed. Save all settings before running.") {
  markSettingsDirty("task_presets", detail);
  invalidateTaskDetections("Task presets changed - refresh detections");
  renderColorPresetMapping();
  renderTaskPresetEditors();
}

function updateColorProfileDraft(color, patch, detail = "Color preset mapping changed. Save all settings before running.") {
  const name = normalizedColorName(color);
  if (!name) return;
  const profiles = taskColorProfiles();
  profiles[name] = {
    ...(profiles[name] || { enabled: true }),
    ...patch,
  };
  if (!state.config?.color_profiles?.[name]) {
    profiles[name].draft = true;
    state.unsavedColorProfiles.add(name);
  }
  markTaskPresetDraftDirty(detail);
}

function updateDropZoneDraft(name, field, rawValue) {
  const zones = taskDropZones();
  if (!zones[name]) return;
  if (field.startsWith("grid.")) {
    const key = field.slice("grid.".length);
    const value = String(rawValue || "").trim() === "" ? null : Number(rawValue);
    if (value === null || !Number.isFinite(value) || value <= 0) {
      if (zones[name].grid) delete zones[name].grid[key];
    } else {
      zones[name].grid = zones[name].grid || { order: "row_major" };
      zones[name].grid[key] = key === "rows" || key === "columns" ? Math.round(value) : value;
    }
    if (zones[name].grid && (!zones[name].grid.rows || !zones[name].grid.columns)) {
      delete zones[name].grid;
    }
  } else {
    const value = Number(rawValue);
    zones[name][field] = Number.isFinite(value) ? value : 0;
  }
  markSettingsDirty("task_presets", "Drop preset changed. Save all settings before running.");
  invalidateTaskDetections("Drop presets changed - refresh detections");
  renderColorPresetMapping();
}

function addDropPreset() {
  const zones = taskDropZones();
  let index = Object.keys(zones).length + 1;
  let name = `preset_${index}`;
  while (zones[name]) {
    index += 1;
    name = `preset_${index}`;
  }
  const fk = state.robotState?.fk || {};
  zones[name] = {
    x_mm: Number(fk.x_mm ?? 0),
    y_mm: Number(fk.y_mm ?? 180),
    z_mm: Number(fk.z_mm ?? 45),
  };
  markTaskPresetDraftDirty("Added a drop preset. Set coordinates and save all settings before running.");
}

function deleteDropPreset(name) {
  const zones = taskDropZones();
  delete zones[name];
  Object.values(taskColorProfiles()).forEach((profile) => {
    if (profile.drop_zone === name) profile.drop_zone = "";
  });
  markTaskPresetDraftDirty("Deleted a drop preset. Review color mappings and save before running.");
}

function renderWorkflowStepper(phase = "setup") {
  if (!elements.taskWorkflowStepper) return;
  const order = ["setup", "detect", "plan", "run"];
  const aliases = {
    queued: "run",
    running: "run",
    capturing: "run",
    planning: "run",
    executing: "run",
    waiting_for_selection: "run",
    completed: "run",
    failed: "run",
    stopped: "run",
    preview: "plan",
  };
  const progress = aliases[phase] || phase;
  if (order.includes(progress)) state.taskProgressStep = progress;
  if (!order.includes(state.activeTaskStep)) state.activeTaskStep = state.taskProgressStep || "setup";
  const active = state.activeTaskStep;
  const activeIndex = Math.max(0, order.indexOf(active));
  const progressIndex = Math.max(0, order.indexOf(state.taskProgressStep || active));
  [...elements.taskWorkflowStepper.children].forEach((item, index) => {
    const step = item.dataset.taskStep || order[index];
    item.classList.toggle("active", index === activeIndex);
    item.classList.toggle("done", index < progressIndex);
    item.setAttribute("aria-current", step === active ? "step" : "false");
  });
  elements.taskStepPanels?.forEach((panel) => {
    panel.hidden = panel.dataset.taskPanel !== active;
  });
}

function renderOperatorPanels() {
  if (!state.config) return;
  ensureTaskPresetDrafts();
  const positions = state.config.named_positions || {};
  if (elements.namedPositionsList) {
    elements.namedPositionsList.innerHTML = "";
    Object.entries(positions).forEach(([name, position]) => {
      const item = document.createElement("div");
      item.className = "program-item";
      const kind = position.type || "joint";
      const target = position.target || {};
      const label =
        kind === "joint"
          ? (position.angles_deg || []).map((value) => format(value, 1)).join(", ")
          : formatCartesianTarget(target);
      item.innerHTML = `
        <div class="program-title"><span>${name}</span><span>${kind}</span></div>
        <code>${label}</code>
        <div class="button-row">
          <button type="button" class="ghost" data-named-preview="${name}">Preview</button>
          <button type="button" data-named-apply="${name}">Move</button>
        </div>
      `;
      elements.namedPositionsList.appendChild(item);
    });
  }

  if (elements.dropZoneSelect) {
    elements.dropZoneSelect.innerHTML = "";
    Object.keys(taskDropZones()).forEach((name) => {
      const option = document.createElement("option");
      option.value = name;
      option.textContent = name;
      elements.dropZoneSelect.appendChild(option);
    });
  }

  if (elements.sortColorSelect) elements.sortColorSelect.innerHTML = "";
  if (elements.includeColorsSelect) elements.includeColorsSelect.innerHTML = "";
  if (elements.visionProfileList) elements.visionProfileList.innerHTML = "";
  if (elements.visionProfileList) {
    const camera = state.config.camera || {};
    const workspace = camera.calibration?.workspace_aruco || {};
    const line = document.createElement("div");
    line.className = "log-line";
    const dictionaries = workspace.dictionary_candidates?.length
      ? workspace.dictionary_candidates.join(", ")
      : workspace.dictionary || "no workspace tags";
    line.innerHTML = `<span>Task vision</span><code>${camera.detection?.provider || "workspace_color"} | tags ${dictionaries}</code>`;
    elements.visionProfileList.appendChild(line);
  }
  ensureDetectedColorDrafts(state.latestDetections);
  Object.entries(taskColorProfiles()).forEach(([name, profile]) => {
    if (elements.sortColorSelect) {
      const option = document.createElement("option");
      option.value = name;
      option.textContent = name;
      elements.sortColorSelect.appendChild(option);
    }
    if (elements.includeColorsSelect) {
      const option = document.createElement("option");
      option.value = name;
      option.textContent = name;
      option.selected = profile.enabled !== false;
      elements.includeColorsSelect.appendChild(option);
    }
    if (elements.visionProfileList) {
      const line = document.createElement("div");
      line.className = "log-line";
      line.innerHTML = `<span>${name}</span><code>${profile.enabled === false ? "off" : "on"} -> ${profile.drop_zone || "-"}</code>`;
      elements.visionProfileList.appendChild(line);
    }
  });
  renderColorPresetMapping();
  renderTaskPresetEditors();
  applyTaskSettingsControls();
  renderSetupChecklist();
  renderToolControls();
  renderTcpCalibration();
}

function renderToolControls(activeOverride = null) {
  if (!elements.toolSelect) return;
  const tools = state.config?.tools || {};
  renderToolSelectOptions(activeOverride);
  const active = activeOverride || tools.active || state.robotState?.active_tool || elements.toolSelect.value || "gripper";
  elements.toolSelect.value = active;
  const preset = tools.presets?.[active] || {};
  const isMagnet = toolPresetIsMagnet(active, preset);
  if (elements.gripperControls) elements.gripperControls.hidden = isMagnet;
  if (elements.gripperSliderLabel) elements.gripperSliderLabel.hidden = isMagnet;
  if (elements.magnetControls) elements.magnetControls.hidden = !isMagnet;
  if (elements.toolValueSlider) {
    if (state.toolSliderDraftValue != null) {
      elements.toolValueSlider.value = String(state.toolSliderDraftValue);
    } else if (state.robotState?.tool_value != null && !state.toolSliderEditing) {
      elements.toolValueSlider.value = String(state.robotState.tool_value);
    }
  }
  const connected = Boolean(state.robotState?.simulation || state.robotState?.connected);
  const realHardwareReady = Boolean(state.robotState?.simulation || state.robotState?.hardware_armed);
  const notFaulted = !["estop", "fault"].includes(state.robotState?.motion_state);
  const canCommand = connected && realHardwareReady && notFaulted;
  if (elements.toolValueSlider) elements.toolValueSlider.disabled = isMagnet || !canCommand;
  [elements.toolOpenBtn, elements.toolCloseBtn].forEach((button) => {
    if (button) button.disabled = isMagnet || !canCommand;
  });
  [elements.toolOnBtn, elements.toolOffBtn].forEach((button) => {
    if (button) button.disabled = !isMagnet || !canCommand;
  });
}

function clearToolSliderLiveState() {
  if (state.toolSliderSendTimer) window.clearTimeout(state.toolSliderSendTimer);
  state.toolSliderSendTimer = null;
  state.toolSliderInFlightValue = null;
  state.toolSliderQueuedValue = null;
  state.toolSliderLastCommandValue = null;
  state.toolSliderDraftValue = null;
  state.toolSliderEditing = false;
}

async function saveActiveTool(active) {
  clearToolSliderLiveState();
  if (!state.config?.tools) {
    renderToolControls(active);
    return;
  }
  if (state.config?.tools) state.config.tools.active = active;
  if (state.robotState) state.robotState.active_tool = active;
  renderToolControls(active);
  const tools = state.config?.tools || { active, presets: {} };
  tools.active = active;
  const payload = await postJson("/api/tools", { active, presets: tools.presets || {} });
  if (payload.ok && payload.config) applyConfig(payload.config);
  if (payload.state) renderState(payload.state);
}

function setSerialModalVisible(visible) {
  if (!elements.serialModal) return;
  elements.serialModal.hidden = !visible;
}

async function refreshSerialPorts() {
  if (!elements.serialPortList) return;
  const response = await fetch("/api/serial/ports");
  const payload = await response.json();
  const ports = payload.ports || [];
  state.selectedSerialPort = state.selectedSerialPort || payload.last_port || ports[0]?.device || null;
  elements.serialPortList.innerHTML = "";
  if (!ports.length) {
    const item = document.createElement("div");
    item.className = "program-item";
    item.textContent = "No serial ports detected.";
    elements.serialPortList.appendChild(item);
    return;
  }
  ports.forEach((port) => {
    const item = document.createElement("button");
    item.type = "button";
    item.className = `port-option ${state.selectedSerialPort === port.device ? "selected" : ""}`;
    item.dataset.port = port.device;
    item.innerHTML = `<strong>${port.device}</strong><span>${port.description || port.name || ""}</span><code>${port.hwid || ""}</code>`;
    elements.serialPortList.appendChild(item);
  });
}

async function openSerialModal() {
  setSerialModalVisible(true);
  await refreshSerialPorts();
}

async function connectSelectedSerial() {
  if (!state.selectedSerialPort) {
    showLocalError("choose a serial port first");
    return;
  }
  const payload = await postJson("/api/connect", {
    simulation: false,
    port: state.selectedSerialPort,
    baud_rate: Number(elements.baudRate.value),
  });
  if (payload.ok) setSerialModalVisible(false);
}

async function refreshDiagnostics() {
  if (!elements.diagnosticsDrawer || elements.diagnosticsDrawer.hidden) return;
  const response = await fetch("/api/diagnostics");
  const payload = await response.json();
  const robotState = payload.state || {};
  const enc = robotState.encoder_angles_deg || [];
  const err = robotState.encoder_errors_deg || [];
  const pending = robotState.pending_motion || {};
  const configChange = robotState.config_change || {};
  const angleText = (values) => Array.isArray(values) ? values.map((value) => format(value, 2)).join(", ") : "-";
  const previewRevision = state.latestPreview?.start_pose_revision;
  elements.diagnosticsSummary.innerHTML = `
    <div class="log-line"><span>Pose source</span><code>${robotState.pose_source || "unknown"}</code></div>
    <div class="log-line"><span>Pose revision</span><code>${robotState.pose_revision ?? 0} (${robotState.pose_known_mask || "0000"})</code></div>
    <div class="log-line"><span>Reported</span><code>${angleText(robotState.reported_angles_deg)}</code></div>
    <div class="log-line"><span>Commanded</span><code>${angleText(robotState.commanded_target_deg || robotState.target_angles_deg)}</code></div>
    <div class="log-line"><span>Pending motion</span><code>${pending.run_id ? `${pending.source}/${pending.mode} - ${pending.status}` : "none"}</code></div>
    <div class="log-line"><span>Draft</span><code>${angleText(state.draftAngles)}</code></div>
    <div class="log-line"><span>Preview start</span><code>${previewRevision == null ? "none" : `revision ${previewRevision}: ${angleText(state.latestPreview?.start_reported_angles_deg)}`}</code></div>
    <div class="log-line"><span>Last rejection/error</span><code>${robotState.last_error || "-"}</code></div>
    <div class="log-line"><span>Config impact</span><code>${(configChange.categories || []).join(", ") || "none"}; pose invalidated=${Boolean(configChange.pose_invalidated)}</code></div>
    <div class="log-line"><span>Encoders</span><code>${robotState.encoder_available || "0000"}</code></div>
    <div class="log-line"><span>Encoder angles</span><code>${enc.map((value) => value == null ? "-" : format(value, 2)).join(", ")}</code></div>
    <div class="log-line"><span>Encoder errors</span><code>${err.map((value) => value == null ? "-" : format(value, 2)).join(", ")}</code></div>
    <div class="log-line"><span>Sync</span><code>${robotState.config_sync_status || "unknown"}</code></div>
  `;
  elements.eventLog.innerHTML = "";
  (payload.events || []).reverse().forEach((event) => {
    const line = document.createElement("div");
    line.className = "log-line";
    const ts = new Date((event.ts || 0) * 1000).toLocaleTimeString();
    line.innerHTML = `<span>${ts} ${event.source}</span><code>${event.message}</code>`;
    elements.eventLog.appendChild(line);
  });
}

async function refreshEvents() {
  if (!elements.eventLog) return;
  const response = await fetch("/api/events?limit=80");
  const payload = await response.json();
  elements.eventLog.innerHTML = "";
  (payload.events || []).reverse().forEach((event) => {
    const line = document.createElement("div");
    line.className = "log-line";
    const ts = new Date((event.ts || 0) * 1000).toLocaleTimeString();
    line.innerHTML = `<span>${ts} ${event.source}</span><code>${event.message}</code>`;
    elements.eventLog.appendChild(line);
  });
}

function namedPositionWaypoint(name) {
  const position = state.config?.named_positions?.[name];
  if (!position) return null;
  if ((position.type || "joint") === "joint") {
    return { type: "joint", mode: "joint", angles_deg: (position.angles_deg || []).map(Number) };
  }
  return { type: "cartesian", mode: "joint", target: position.target || position };
}

async function previewNamedPosition(name) {
  const waypoint = namedPositionWaypoint(name);
  if (!waypoint) return;
  const payload = await postJson("/api/path/preview", {
    mode: "program",
    branch: elements.ikBranchSelect.value,
    settings: pathSettings(),
    waypoints: [waypoint],
  });
  if (payload.ok) renderPreview(payload.preview);
  else renderPreviewFailure(payload);
}

async function moveNamedPosition(name) {
  await previewNamedPosition(name);
  if (state.previewId) await executePreview();
}

async function sendTool(action, value = null) {
  clearToolSliderLiveState();
  const payload = await postJson("/api/tool", { action, value, tool: state.config?.tools?.active });
  if (payload.state) renderState(payload.state);
  await refreshDiagnostics();
}

function updateToolSliderDraft() {
  if (!elements.toolValueSlider) return null;
  const value = clamp(Number(elements.toolValueSlider.value || 0), 0, 1);
  state.toolSliderDraftValue = Number.isFinite(value) ? value : 0;
  return state.toolSliderDraftValue;
}

function beginToolSliderEdit() {
  state.toolSliderEditing = true;
  updateToolSliderDraft();
}

function endToolSliderEdit() {
  updateToolSliderDraft();
  state.toolSliderEditing = false;
}

function canSendLiveToolSlider() {
  const tools = state.config?.tools || {};
  const active = tools.active || state.robotState?.active_tool || elements.toolSelect?.value || "gripper";
  const preset = tools.presets?.[active] || {};
  const connected = Boolean(state.robotState?.simulation || state.robotState?.connected);
  const realHardwareReady = Boolean(state.robotState?.simulation || state.robotState?.hardware_armed);
  const notFaulted = !["estop", "fault"].includes(state.robotState?.motion_state);
  return !toolPresetIsMagnet(active, preset) && connected && realHardwareReady && notFaulted;
}

function sameToolSliderValue(left, right) {
  return left != null && right != null && Math.abs(Number(left) - Number(right)) < 0.001;
}

function queueToolSliderLiveSet({ immediate = false } = {}) {
  const value = updateToolSliderDraft();
  if (value == null || !canSendLiveToolSlider()) return;
  if (sameToolSliderValue(value, state.toolSliderQueuedValue)) {
    if (immediate && state.toolSliderSendTimer) flushToolSliderLiveSet();
    return;
  }
  if (state.toolSliderSendInFlight && state.toolSliderQueuedValue == null && sameToolSliderValue(value, state.toolSliderInFlightValue)) {
    return;
  }
  if (!state.toolSliderEditing && sameToolSliderValue(value, state.toolSliderLastCommandValue)) return;
  state.toolSliderQueuedValue = value;
  if (immediate) {
    flushToolSliderLiveSet();
    return;
  }
  if (!state.toolSliderSendTimer && !state.toolSliderSendInFlight) {
    state.toolSliderSendTimer = window.setTimeout(flushToolSliderLiveSet, 75);
  }
}

async function flushToolSliderLiveSet() {
  if (state.toolSliderSendTimer) window.clearTimeout(state.toolSliderSendTimer);
  state.toolSliderSendTimer = null;
  if (state.toolSliderSendInFlight || state.toolSliderQueuedValue == null) return;
  const value = state.toolSliderQueuedValue;
  state.toolSliderQueuedValue = null;
  state.toolSliderSendInFlight = true;
  state.toolSliderInFlightValue = value;
  state.toolSliderLastCommandValue = value;
  try {
    const payload = await postJson("/api/tool", { action: "set", value, tool: state.config?.tools?.active });
    if (payload.state) renderState(payload.state);
    const reportedValue = Number(payload.state?.tool_value);
    if (payload.ok && !state.toolSliderEditing && state.toolSliderQueuedValue == null && Math.abs(reportedValue - value) < 0.001) {
      state.toolSliderDraftValue = null;
      renderToolControls();
    }
  } catch (error) {
    showLocalError(error?.message || String(error));
  } finally {
    state.toolSliderSendInFlight = false;
    state.toolSliderInFlightValue = null;
    if (state.toolSliderQueuedValue != null) {
      state.toolSliderSendTimer = window.setTimeout(flushToolSliderLiveSet, 0);
    }
  }
}

function renderTaskSummary(sequence, preview) {
  const steps = sequence?.steps || [];
  const trajectory = preview?.trajectory || {};
  const taskPreview = sequence?.task_preview || state.lastTaskPreview || {};
  const warnings = taskPreview.warnings || [];
  const normalized = taskPreview.normalized_settings || {};
  const motionSettings = preview?.settings || {};
  const calibrationApplied = Array.isArray(preview?.calibration)
    && preview.calibration.some((item) => item.applied);
  const orientation = normalized.orientation_policy === "fixed"
    ? `${format(normalized.pickup_phi_deg)}° / ${format(normalized.drop_phi_deg)}°`
    : normalized.orientation_policy || "-";
  elements.taskSummary.innerHTML = `
    <div class="log-line"><span>Steps</span><code>${steps.length}</code></div>
    <div class="log-line"><span>Moves</span><code>${sequence?.waypoints?.length || 0}</code></div>
    <div class="log-line"><span>Duration</span><code>${format(trajectory.duration_s, 2)} s</code></div>
    <div class="log-line"><span>TCP correction</span><code>${calibrationApplied ? "enabled for Cartesian task targets" : "not applied"}</code></div>
    <div class="log-line"><span>Strategy</span><code>${taskPreview.strategy || sequence?.strategy || "-"}</code></div>
    <div class="log-line"><span>Objects</span><code>${taskPreview.selected_objects?.length || sequence?.object_count || 0}</code></div>
    <div class="log-line"><span>TCP Z</span><code>pick ${format(normalized.pickup_z_mm)} / drop ${format(normalized.dropoff_z_mm)} mm</code></div>
    <div class="log-line"><span>Orientation</span><code>${orientation}</code></div>
    <div class="log-line"><span>Motion limits</span><code>${format(motionSettings.global_speed_deg_s)} deg/s / ${format(motionSettings.global_accel_deg_s2)} deg/s²</code></div>
    <div class="log-line"><span>Warnings</span><code>${warnings.length ? warnings.join("; ") : "-"}</code></div>
  `;
}

function renderTaskPlanPreview(taskPreview = {}, sequence = {}) {
  if (!elements.taskPlanPreview) return;
  const objects = taskPreview.selected_objects || sequence.objects || [];
  const ignored = taskPreview.ignored_detections || sequence.ignored_detections || [];
  const assigned = taskPreview.assigned_targets || [];
  const modes = taskPreview.motion_modes || {};
  if (!objects.length && !ignored.length) {
    elements.taskPlanPreview.innerHTML = `<div class="empty-state">No planned objects yet.</div>`;
    return;
  }
  elements.taskPlanPreview.innerHTML = `
    <div class="task-preview-grid">
      <div>
        <h3>Object order</h3>
        <div class="task-object-queue">
          ${objects.map((object) => `
            <div class="task-object-row">
              <span>${object.index || "-"}</span>
              <strong>${object.color || "object"}</strong>
              <code>${object.object_target ? `pick ${format(object.object_target.x_mm)}, ${format(object.object_target.y_mm)}, z ${format(object.object_target.z_mm)}` : "-"}</code>
              <code>${object.drop_target ? `drop ${format(object.drop_target.x_mm)}, ${format(object.drop_target.y_mm)}, z ${format(object.drop_target.z_mm)}` : object.drop_zone || "-"}</code>
              <small>${object.grid_slot ? `slot r${object.grid_slot.row + 1} c${object.grid_slot.column + 1}` : "fixed anchor"}</small>
            </div>
          `).join("") || `<div class="empty-state">Closed-loop preview is waiting for a manual selection.</div>`}
        </div>
      </div>
      <div>
        <h3>Motion modes</h3>
        <div class="path-summary compact-summary">
          <div class="log-line"><span>Transfer</span><code>${modes.transfer || "-"}</code></div>
          <div class="log-line"><span>Pickup</span><code>${modes.pickup_descent || modes.pickup_approach || "-"}</code></div>
          <div class="log-line"><span>Lift</span><code>${modes.lift || "-"}</code></div>
          <div class="log-line"><span>Drop</span><code>${modes.drop_descent || modes.drop_approach || "-"}</code></div>
        </div>
        <h3>Ignored</h3>
        <div class="ignored-list">
          ${ignored.slice(0, 8).map((item) => `<div><span>${item.color || item.detection_id}</span><code>${item.reason || item.reason_code}</code></div>`).join("") || `<div class="empty-state">None</div>`}
        </div>
      </div>
    </div>
    ${assigned.length ? `<div class="assigned-targets">${assigned.map((item) => `<span>${item.color || "object"} → ${item.drop_zone}${item.grid_slot ? ` / slot ${item.grid_slot.index + 1}` : ""}</span>`).join("")}</div>` : ""}
  `;
}

async function previewTask() {
  state.taskLocalStatusAt = Date.now() / 1000;
  elements.taskStatus.textContent = "Previewing...";
  const task = "color_sorting";
  if (elements.taskModeSelect) elements.taskModeSelect.value = "color_sorting";
  const selectedIds = [...state.selectedDetectionIds];
  let taskSettings;
  try {
    taskSettings = taskSettingsPayload();
  } catch (error) {
    elements.taskStatus.textContent = error.message;
    invalidateTaskPreview(error.message);
    return;
  }
  if (!state.taskDetectionsCapturedAt || !state.latestDetections.length) {
    invalidateTaskPreview("Refresh detections before previewing a task");
    return;
  }
  let motionSettings;
  try {
    motionSettings = taskPathSettingsPayload();
  } catch (error) {
    invalidateTaskPreview(error.message);
    return;
  }
  const request =
    task === "color_sorting"
      ? {
          task,
          detections: state.latestDetections,
          task_settings: taskSettings,
          selected_detection_ids: selectedIds,
          settings: motionSettings,
          branch: elements.ikBranchSelect.value,
        }
      : {
          task,
          object_target: ikTargetPayload(),
          drop_zone: elements.dropZoneSelect.value,
          task_settings: taskSettings,
          settings: motionSettings,
          branch: elements.ikBranchSelect.value,
        };
  const payload = await postJson("/api/task/preview", request);
  if (payload.ok) {
    renderPreview(payload.preview);
    state.taskPreviewId = payload.preview_id;
    state.taskPreviewCreatedAt = Date.now() / 1000;
    state.taskLocalStatusAt = state.taskPreviewCreatedAt;
    state.lastTaskPreview = payload.task_preview || payload.sequence?.task_preview || null;
    renderTaskSummary(payload.sequence, payload.preview);
    renderTaskPlanPreview(state.lastTaskPreview, payload.sequence);
    state.view?.setTaskPreview?.(state.lastTaskPreview);
    const blockedByDraft = taskDraftBlocksRun();
    elements.executeTaskBtn.disabled = blockedByDraft;
    elements.taskStatus.textContent = blockedByDraft ? "Preview ready - save mappings before Run" : "Preview ready";
    renderWorkflowStepper("preview");
  } else {
    renderPreviewFailure(payload);
    state.lastTaskPreview = payload.task_preview || payload.sequence?.task_preview || null;
    renderTaskPlanPreview(state.lastTaskPreview, payload.sequence);
    state.view?.setTaskPreview?.(state.lastTaskPreview);
    elements.taskStatus.textContent = payload.error || "Task preview failed";
    state.taskLocalStatusAt = Date.now() / 1000;
  }
  await refreshDiagnostics();
}

async function executeTask() {
  if (!state.taskPreviewId) return;
  const payload = await postJson("/api/task/execute", { preview_id: state.taskPreviewId });
  if (payload.ok) {
    releaseJointControlIntent();
    state.previewId = null;
    state.previewAngles = null;
    state.taskPreviewId = null;
    state.taskPreviewCreatedAt = null;
    state.taskDetectionsCapturedAt = null;
    state.latestDetections = [];
    state.selectedDetectionIds.clear();
    state.view?.setObjectDetections([]);
    state.ikUserEdited = false;
  } else if (/preview|configuration|model|start pose/i.test(payload.error || "")) {
    clearViewPreview();
    state.taskPreviewId = null;
    state.taskPreviewCreatedAt = null;
  }
  if (payload.state) renderState(payload.state);
  else syncJointControls();
  updateDisabledState();
  elements.taskStatus.textContent = payload.ok ? "Task running" : payload.error || "Task failed";
  state.taskLocalStatusAt = Date.now() / 1000;
  if (payload.ok) state.activeTaskStep = "run";
  renderWorkflowStepper(payload.ok ? "run" : "plan");
  await refreshDiagnostics();
}

async function stopTask() {
  const payload = await postJson("/api/task/stop", {});
  if (payload.state) renderState(payload.state);
  elements.taskStatus.textContent = payload.ok ? "Task stopped" : payload.error || "Stop failed";
  state.taskLocalStatusAt = Date.now() / 1000;
  await refreshDiagnostics();
}

async function selectRuntimeDetection(detectionId) {
  const runId = state.robotState?.task_execution?.run_id;
  if (!runId) return;
  const payload = await postJson("/api/task/select", { run_id: runId, detection_id: detectionId });
  if (payload.state) renderState(payload.state);
  if (!payload.ok) showLocalError(payload.error || "Manual selection failed.");
}

function setCameraPopupVisible(visible) {
  if (!elements.cameraPopup) return;
  elements.cameraPopup.hidden = !visible;
  if (visible) {
    elements.cameraPopupStatus.textContent = state.cameraLive ? "Live detection active" : "Ready";
  }
}

function cameraLiveIntervalMs() {
  return clamp(Number(state.config?.camera?.detection?.live_interval_ms || 450), 150, 5000);
}

function scheduleCameraFrame(delayMs = cameraLiveIntervalMs()) {
  window.clearTimeout(state.cameraTimer);
  state.cameraTimer = null;
  if (!state.cameraLive) return;
  state.cameraTimer = window.setTimeout(detectVision, Math.max(0, delayMs));
}

function setCameraLive(enabled) {
  state.cameraLive = Boolean(enabled);
  if (elements.cameraLiveToggle) elements.cameraLiveToggle.checked = state.cameraLive;
  if (state.cameraLive) {
    scheduleCameraFrame(0);
  } else {
    window.clearTimeout(state.cameraTimer);
    state.cameraTimer = null;
    if (elements.cameraPopupStatus) elements.cameraPopupStatus.textContent = "Stopped";
  }
}

function workspaceProjectionEnabled() {
  return Boolean(elements.workspaceProjectionInput?.checked);
}

function workspaceProjectionIntervalMs() {
  return clamp(
    Number(state.config?.camera?.display?.projection_interval_ms || 80),
    40,
    1000
  );
}

function scheduleWorkspaceProjection(delayMs = workspaceProjectionIntervalMs()) {
  window.clearTimeout(state.workspaceProjectionTimer);
  state.workspaceProjectionTimer = null;
  if (
    !workspaceProjectionEnabled()
    || state.workspaceCalibrationBusy
    || !state.config?.camera?.enabled
  ) {
    return;
  }
  state.workspaceProjectionTimer = window.setTimeout(
    refreshWorkspaceProjection,
    Math.max(0, delayMs)
  );
}

async function refreshWorkspaceProjection() {
  if (state.workspaceProjectionInFlight) return;
  state.workspaceProjectionInFlight = true;
  const startedAt = performance.now();
  try {
    const payload = await fetch(
      `/api/vision/workspace/live?t=${Date.now()}`,
      { cache: "no-store" }
    ).then((response) => response.json());
    if (!payload.ok) {
      state.view?.setWorkspaceCameraProjection(null, false);
      return;
    }
    state.view?.setWorkspaceCameraProjection(
      payload.workspace_projection,
      true
    );
  } catch {
    state.view?.setWorkspaceCameraProjection(null, false);
  } finally {
    state.workspaceProjectionInFlight = false;
    scheduleWorkspaceProjection(
      Math.max(0, workspaceProjectionIntervalMs() - (performance.now() - startedAt))
    );
  }
}

function updateCameraFrameAspect(image, imageSize) {
  if (!image?.parentElement || !imageSize?.width || !imageSize?.height) return;
  image.parentElement.style.aspectRatio = `${imageSize.width} / ${imageSize.height}`;
}

async function detectVision() {
  if (state.cameraInFlight) return;
  state.cameraInFlight = true;
  if (elements.cameraPopupStatus) elements.cameraPopupStatus.textContent = "Detecting...";
  elements.visionSummary.innerHTML = `<div class="log-line"><span>Status</span><code>Detecting...</code></div>`;
  try {
    const payload = await fetch(
      `/api/vision/frame?t=${Date.now()}`,
      { cache: "no-store" }
    ).then((response) => response.json());
    elements.visionSummary.innerHTML = "";
    if (!payload.ok) {
      elements.visionSummary.innerHTML = `<div class="log-line"><span>Error</span><code>${payload.error || "-"}</code></div>`;
      if (elements.cameraPlaceholder) elements.cameraPlaceholder.hidden = false;
      if (elements.cameraPopupStatus) elements.cameraPopupStatus.textContent = payload.error || "Detection failed";
      return;
    }
    state.latestDetections = payload.detections || [];
    state.taskDetectionsCapturedAt = Date.now() / 1000;
    ensureDetectedColorDrafts(state.latestDetections, { markDirty: true });
    state.selectedDetectionIds.clear();
    invalidateTaskPreview("Detections refreshed");
    if (payload.image_b64 && elements.cameraFrame) {
      elements.cameraFrame.src = payload.image_b64;
      elements.cameraFrame.hidden = false;
      if (elements.cameraPlaceholder) elements.cameraPlaceholder.hidden = true;
    }
    renderDetectionList(payload.detections || []);
    renderColorPresetMapping();
    renderTaskPresetEditors();
    if (state.view) {
      state.view.setObjectDetections(state.latestDetections.filter((detection) => detection.ok && detection.robot));
    }
    const workspace = payload.workspace || {};
    const visibleTags = (workspace.visible_ids || []).join(", ") || "none";
    const missingTags = (workspace.missing_ids || []).join(", ") || "none";
    const frameSize = workspace.image_size_px ? ` | ${workspace.image_size_px.width}x${workspace.image_size_px.height}` : "";
    const dictionary = workspace.dictionary || workspace.configured_dictionary || "no dictionary";
    const mode = workspace.detection_mode || "none";
    const tagsLine = document.createElement("div");
    tagsLine.className = "log-line";
    tagsLine.innerHTML = `<span>Reference tags</span><code>${dictionary} | ${mode} | visible ${visibleTags} | missing ${missingTags}${frameSize}</code>`;
    elements.visionSummary.appendChild(tagsLine);
    const calibrationLine = document.createElement("div");
    calibrationLine.className = "log-line";
    calibrationLine.innerHTML = `<span>Calibration</span><code>${workspace.message || workspace.error || payload.calibration_source || "unavailable"}</code>`;
    elements.visionSummary.appendChild(calibrationLine);
    if (workspace.workspace_polygon_source) {
      const workspaceLine = document.createElement("div");
      workspaceLine.className = "log-line";
      const detectionSource = workspace.detection_source ? ` | detection ${workspace.detection_source}` : "";
      workspaceLine.innerHTML = `<span>Workspace</span><code>${workspace.workspace_polygon_source}${detectionSource}</code>`;
      elements.visionSummary.appendChild(workspaceLine);
    }
    if (workspace.warning) {
      const warningLine = document.createElement("div");
      warningLine.className = "log-line";
      warningLine.innerHTML = `<span>Warning</span><code>${workspace.warning}</code>`;
      elements.visionSummary.appendChild(warningLine);
    }
    (payload.detections || []).forEach((detection) => {
      const line = document.createElement("div");
      line.className = "log-line";
      const center = detection.center_px ? `px ${format(detection.center_px.x, 0)}, ${format(detection.center_px.y, 0)}` : detection.reason || "-";
      const robot = detection.robot
        ? ` | x ${format(detection.robot.x_mm)} mm, y ${format(detection.robot.y_mm)} mm, z ${format(detection.robot.z_mm)} mm`
        : detection.projection_error ? ` | ${detection.projection_error}` : "";
      line.innerHTML = `<span>${detection.label || detection.color}</span><code>${detection.ok ? center + robot : detection.reason}</code>`;
      elements.visionSummary.appendChild(line);
    });
    if (elements.cameraPopupStatus) {
      const mode = state.cameraLive ? "Live" : "Frame";
      const calibrationStatus = workspace.status || payload.calibration_source || "uncalibrated";
      elements.cameraPopupStatus.textContent = `${mode} | ${state.latestDetections.length} object(s) | ${calibrationStatus}`;
    }
  } catch (error) {
    const message = error?.message || String(error);
    elements.visionSummary.innerHTML = `<div class="log-line"><span>Error</span><code>${message}</code></div>`;
    if (elements.cameraPopupStatus) elements.cameraPopupStatus.textContent = message;
  } finally {
    state.cameraInFlight = false;
    if (state.cameraLive) scheduleCameraFrame();
  }
}

function setWorkspaceCalibrationBusy(busy, status = null) {
  state.workspaceCalibrationBusy = Boolean(busy);
  [
    elements.calibrateWorkspaceBtn,
    elements.verifyWorkspaceCalibrationBtn,
  ].forEach((button) => {
    if (button) button.disabled = state.workspaceCalibrationBusy;
  });
  if (status && elements.workspaceCalibrationStatus) {
    elements.workspaceCalibrationStatus.textContent = status;
  }
  if (busy) {
    window.clearTimeout(state.workspaceProjectionTimer);
    state.workspaceProjectionTimer = null;
  } else {
    scheduleWorkspaceProjection(0);
  }
}

function renderCameraIntrinsics(camera = state.config?.camera || {}) {
  const resolution = camera.resolution || {};
  const intrinsics = camera.intrinsics || {};
  const display = camera.display || {};
  const workspaceAruco = camera.calibration?.workspace_aruco || {};
  if (elements.cameraEnabledInput) elements.cameraEnabledInput.checked = Boolean(camera.enabled);
  if (elements.cameraSourceInput) elements.cameraSourceInput.value = String(camera.source_index ?? 0);
  if (elements.cameraWidthInput) elements.cameraWidthInput.value = String(resolution.width ?? 1280);
  if (elements.cameraHeightInput) elements.cameraHeightInput.value = String(resolution.height ?? 720);
  if (elements.workspaceProjectionInput) {
    elements.workspaceProjectionInput.checked = Boolean(display.project_live_view);
  }
  if (elements.workspaceArucoEnabledInput) elements.workspaceArucoEnabledInput.checked = workspaceAruco.enabled !== false;
  if (elements.workspaceArucoInvertInput) elements.workspaceArucoInvertInput.checked = workspaceAruco.invert_first !== false;
  if (elements.workspaceArucoFallbackInput) {
    elements.workspaceArucoFallbackInput.checked = workspaceAruco.allow_normal_fallback !== false;
  }
  if (elements.cameraFxInput) elements.cameraFxInput.value = intrinsics.fx_px ?? "";
  if (elements.cameraFyInput) elements.cameraFyInput.value = intrinsics.fy_px ?? "";
  if (elements.cameraCxInput) elements.cameraCxInput.value = intrinsics.cx_px ?? "";
  if (elements.cameraCyInput) elements.cameraCyInput.value = intrinsics.cy_px ?? "";
  if (elements.cameraDistortionInput) {
    elements.cameraDistortionInput.value = (intrinsics.distortion_coefficients || [0, 0, 0, 0, 0]).join(", ");
  }
}

function cameraSettingsDraft() {
  const camera = clonePlain(state.config?.camera || {});
  const distortion = String(elements.cameraDistortionInput?.value || "")
    .split(",")
    .map((value) => Number(value.trim()))
    .filter((value) => Number.isFinite(value));
  const intrinsicInputs = [
    elements.cameraFxInput,
    elements.cameraFyInput,
    elements.cameraCxInput,
    elements.cameraCyInput,
  ];
  const hasAnyIntrinsics = intrinsicInputs.some((input) => String(input?.value || "").trim() !== "");
  const fx = readNumber(elements.cameraFxInput, NaN);
  const fy = readNumber(elements.cameraFyInput, NaN);
  const cx = readNumber(elements.cameraCxInput, NaN);
  const cy = readNumber(elements.cameraCyInput, NaN);
  if (hasAnyIntrinsics && (![fx, fy, cx, cy].every(Number.isFinite) || fx <= 0 || fy <= 0)) {
    throw new Error("Enter all four camera intrinsics, with positive fx/fy, or leave all four blank.");
  }
  if (![4, 5, 8, 12, 14].includes(distortion.length)) {
    throw new Error("Distortion must contain 4, 5, 8, 12, or 14 comma-separated values.");
  }
  camera.enabled = Boolean(elements.cameraEnabledInput?.checked);
  camera.source_index = Math.max(0, Math.round(readNumber(elements.cameraSourceInput, 0)));
  camera.resolution = {
    width: Math.max(1, Math.round(readNumber(elements.cameraWidthInput, 1280))),
    height: Math.max(1, Math.round(readNumber(elements.cameraHeightInput, 720))),
  };
  camera.display = {
    ...(camera.display || {}),
    project_live_view: Boolean(elements.workspaceProjectionInput?.checked),
  };
  delete camera.display.settings_live_preview;
  camera.calibration = camera.calibration || {};
  camera.calibration.workspace_aruco = {
    ...(camera.calibration.workspace_aruco || {}),
    enabled: Boolean(elements.workspaceArucoEnabledInput?.checked),
    invert_first: Boolean(elements.workspaceArucoInvertInput?.checked),
    allow_normal_fallback: Boolean(elements.workspaceArucoFallbackInput?.checked),
  };
  camera.intrinsics = {
    ...(camera.intrinsics || {}),
    source: hasAnyIntrinsics ? "manual" : "uncalibrated_workspace_homography",
    fx_px: hasAnyIntrinsics ? fx : null,
    fy_px: hasAnyIntrinsics ? fy : null,
    cx_px: hasAnyIntrinsics ? cx : null,
    cy_px: hasAnyIntrinsics ? cy : null,
    camera_matrix: hasAnyIntrinsics ? [[fx, 0, cx], [0, fy, cy], [0, 0, 1]] : null,
    distortion_coefficients: distortion,
  };
  return camera;
}

function renderWorkspaceCalibrationDetections(detections = []) {
  if (!elements.workspaceCalibrationDetections) return;
  elements.workspaceCalibrationDetections.innerHTML = "";
  if (!detections.length) {
    const empty = document.createElement("div");
    empty.className = "program-item";
    empty.textContent = "No configured workspace tags detected.";
    elements.workspaceCalibrationDetections.appendChild(empty);
    return;
  }
  detections.forEach((detection) => {
    const item = document.createElement("div");
    item.className = `program-item ${detection.configured ? "" : "invalid"}`;
    const center = detection.center_px || {};
    const corner = detection.workspace_corner_px || {};
    const robot = detection.robot_center_mm || {};
    const cornerText = Number.isFinite(corner.x) && Number.isFinite(corner.y)
      ? ` | outer corner ${format(corner.x, 1)}, ${format(corner.y, 1)}`
      : " | outer corner missing";
    const robotText = Number.isFinite(robot.x) && Number.isFinite(robot.y)
      ? `X ${format(robot.x, 1)} mm, Y ${format(robot.y, 1)} mm`
      : "not in workspace layout";
    item.innerHTML = `
      <div class="program-title"><span>Tag ${detection.id}</span><span>${robotText}</span></div>
      <code>center ${format(center.x, 1)}, ${format(center.y, 1)}${cornerText}</code>
    `;
    elements.workspaceCalibrationDetections.appendChild(item);
  });
}

function renderWorkspaceCalibration(payload = {}) {
  state.workspaceCalibrationStatus = payload;
  const result = payload.result || null;
  const saved = payload.saved_result || null;
  const session = payload.session || {};
  const comparison = payload.comparison || {};
  const metrics = result?.metrics || saved?.metrics || {};
  if (elements.workspaceCalibrationStatus) {
    elements.workspaceCalibrationStatus.textContent = payload.comparison
      ? payload.ok ? "Workspace verified" : "Verification incomplete"
      : payload.calibrated
        ? "Calibration saved"
        : result?.ok
          ? "Calibration complete"
          : saved?.ok
            ? "Workspace calibrated"
            : payload.error || result?.error || "Workspace uncalibrated";
  }
  const settings = payload.settings || state.config?.camera?.calibration?.workspace_aruco || {};
  const polygon = settings.workspace_polygon_robot_mm || [];
  const xValues = polygon.map((point) => Number(point?.[0])).filter(Number.isFinite);
  const yValues = polygon.map((point) => Number(point?.[1])).filter(Number.isFinite);
  const bounds = xValues.length && yValues.length
    ? `X ${format(Math.min(...xValues), 1)} to ${format(Math.max(...xValues), 1)} mm | Y ${format(Math.min(...yValues), 1)} to ${format(Math.max(...yValues), 1)} mm`
    : "not configured";
  const centerCounts = session.tag_observation_counts || {};
  const cornerCounts = session.corner_observation_counts || {};
  if (elements.workspaceCalibrationMetrics) {
    const rows = [
      [
        "Calibration",
        saved?.ok
          ? "Saved map active"
          : "No saved map",
      ],
      [
        "Last run",
        result?.frame_count
          ? `${result.frame_count} frames | ${
              (result.required_ids || []).map((id) => (
                `${id}:${centerCounts[id] || 0}/${cornerCounts[id] || 0}`
              )).join("  ")
            }`
          : "-",
      ],
      [
        "Map quality",
        metrics.rmse_mm != null
          ? `${format(metrics.rmse_mm, 2)} mm RMSE | ${format(metrics.max_error_mm, 2)} mm max`
          : saved?.ok ? "saved planar map available" : "waiting for all four tags",
      ],
      [
        "Fresh verification",
        comparison.rmse_mm != null
          ? `${format(comparison.rmse_mm, 2)} mm RMSE | ${format(comparison.max_error_mm, 2)} mm max`
          : "-",
      ],
      ["Robot workspace", bounds],
      ["Coordinates", "Robot X is sideways; robot Y is forward; workspace Z is 0 mm"],
      [
        "Status",
        payload.error
          || result?.error
          || (payload.calibrated
            ? "Calibration was solved and saved."
            : saved?.ok
              ? "Normal operation uses this saved map."
              : "Run workspace calibration."),
      ],
    ];
    elements.workspaceCalibrationMetrics.innerHTML = rows
      .map(([label, value]) => `<div class="log-line"><span>${label}</span><code>${value}</code></div>`)
      .join("");
  }
  if (payload.image_b64 && elements.workspaceCalibrationFrame) {
    elements.workspaceCalibrationFrame.src = payload.image_b64;
    elements.workspaceCalibrationFrame.hidden = false;
    updateCameraFrameAspect(
      elements.workspaceCalibrationFrame,
      payload.session?.image_size_px
        || payload.settings?.reference_resolution
        || payload.camera?.resolution
    );
    if (elements.workspaceCalibrationPlaceholder) {
      elements.workspaceCalibrationPlaceholder.hidden = true;
    }
  }
  renderWorkspaceCalibrationDetections(payload.detections || []);
}

async function loadWorkspaceCalibrationStatus() {
  const response = await fetch("/api/vision/workspace/status");
  const payload = await response.json();
  if (payload.ok) renderWorkspaceCalibration(payload);
}

async function calibrateWorkspace() {
  setWorkspaceCalibrationBusy(true, "Calibrating...");
  try {
    const saved = await saveAllSettings();
    if (!saved) return;
    const payload = await postJson("/api/vision/workspace/calibrate", {
      max_frames: 36,
      sample_interval_ms: 40,
    });
    if (payload.ok && payload.config) applyConfig(payload.config);
    renderWorkspaceCalibration(payload);
  } finally {
    setWorkspaceCalibrationBusy(false);
  }
}

async function verifyWorkspaceCalibration() {
  setWorkspaceCalibrationBusy(true, "Verifying workspace...");
  try {
    const saved = await saveAllSettings();
    if (!saved) return;
    const payload = await postJson("/api/vision/workspace/verify", {});
    renderWorkspaceCalibration(payload);
  } finally {
    setWorkspaceCalibrationBusy(false);
  }
}

function tcpCalibrationSummary() {
  return state.config?.kinematics_calibration || {};
}

function tcpCalibrationTarget() {
  return {
    x_mm: readRequiredNumber(elements.tcpCalXInput, "Calibration target X"),
    y_mm: readRequiredNumber(elements.tcpCalYInput, "Calibration target Y"),
    z_mm: readRequiredNumber(elements.tcpCalZInput, "Calibration target Z"),
    phi_deg: readRequiredNumber(elements.tcpCalPhiInput, "Calibration target phi"),
  };
}

function setTcpCalibrationTarget(target = {}) {
  if (!elements.tcpCalXInput) return;
  elements.tcpCalXInput.value = format(target.x_mm ?? 0, 2);
  elements.tcpCalYInput.value = format(target.y_mm ?? 0, 2);
  elements.tcpCalZInput.value = format(target.z_mm ?? 0, 2);
  elements.tcpCalPhiInput.value = format(target.phi_deg ?? 0, 2);
}

function tcpMetricText(metrics) {
  if (!metrics || !metrics.count) return "not run";
  return `XY ${format(metrics.xy_rmse_mm, 2)} RMSE / ${format(metrics.xy_max_mm, 2)} max; Z ${format(metrics.z_rmse_mm, 2)} RMSE / ${format(metrics.z_max_abs_mm, 2)} max`;
}

function renderTcpCalibrationTargets() {
  if (!elements.tcpCalibrationTargetList) return;
  elements.tcpCalibrationTargetList.innerHTML = "";
  if (!state.tcpCalibrationTargets.length) {
    elements.tcpCalibrationTargetList.innerHTML = `<div class="empty-state">Generate a workspace grid or enter one target manually.</div>`;
    return;
  }
  state.tcpCalibrationTargets.forEach((point, index) => {
    const target = point.intended_target || {};
    const item = document.createElement("div");
    item.className = `program-item ${point.reachable ? "" : "invalid"}`;
    item.innerHTML = `
      <div class="program-title">
        <span>Point ${index + 1}</span>
        <span>${point.reachable ? "reachable" : "IK blocked"}</span>
      </div>
      <code>x ${format(target.x_mm, 1)}, y ${format(target.y_mm, 1)}, z ${format(target.z_mm, 1)}, phi ${format(target.phi_deg, 1)}</code>
      <div class="button-row"><button type="button" class="ghost" data-tcp-cal-target="${index}">Load target</button></div>
    `;
    elements.tcpCalibrationTargetList.appendChild(item);
  });
}

function renderTcpCalibration() {
  if (!elements.tcpCalibrationStatus) return;
  const summary = tcpCalibrationSummary();
  const settings = summary.settings || {};
  const profile = summary.active_profile || {};
  const result = profile.result || null;
  const workspace = summary.workspace || {};
  const samples = Array.isArray(profile.samples) ? profile.samples : [];
  const enabled = Boolean(summary.enabled);
  elements.tcpCalibrationStatus.textContent = enabled ? "Enabled" : result ? "Fitted, disabled" : "Not fitted";
  elements.tcpCalibrationStatus.classList.toggle("ready", enabled);
  elements.tcpCalibrationStatus.classList.toggle("warning", !enabled && Boolean(result));
  elements.tcpCalEnableInput.checked = Boolean(settings.enabled && profile.enabled);
  elements.tcpCalModelSelect.value = profile.model_type || settings.default_model || "affine_xy_z_offset";

  elements.tcpCalibrationWorkspaceStatus.innerHTML = `
    <div class="log-line"><span>Workspace map</span><code>${workspace.calibrated ? "saved" : "not calibrated"}</code></div>
    <div class="log-line"><span>Source</span><code>${workspace.source || "-"}</code></div>
    <div class="log-line"><span>Tool profile</span><code>${summary.active_profile_key || state.config?.tools?.active || "-"}</code></div>
    <div class="log-line"><span>Frame</span><code>robot base XYZ, mm; +Z upward</code></div>
  `;
  elements.tcpCalibrationModelStatus.innerHTML = `
    <div class="log-line"><span>Model</span><code>${profile.model_type || "not fitted"}</code></div>
    <div class="log-line"><span>Fit samples</span><code>${samples.filter((sample) => sample.role !== "validation").length}</code></div>
    <div class="log-line"><span>Validation samples</span><code>${samples.filter((sample) => sample.role === "validation").length}</code></div>
    <div class="log-line"><span>Result</span><code>${result?.fit?.status || "not run"}</code></div>
  `;

  const fit = result?.fit || summary.fit_quality;
  const validation = result?.validation || summary.validation_quality;
  const diagnostics = result?.diagnostics || [];
  elements.tcpCalibrationMetrics.innerHTML = `
    <div class="log-line"><span>Before correction</span><code>${tcpMetricText(fit?.before)}</code></div>
    <div class="log-line"><span>Fit residual</span><code>${tcpMetricText(fit?.after_model)} (${fit?.status || "not run"})</code></div>
    <div class="log-line"><span>Validation landing</span><code>${tcpMetricText(validation?.landing)} (${validation?.status || "not run"})</code></div>
    <div class="log-line"><span>Worst fit</span><code>${(fit?.worst_samples || []).slice(0, 3).map((item) => `${item.id}: ${format(item.error_xy_mm, 1)} XY`).join(" | ") || "-"}</code></div>
    <div class="log-line"><span>Diagnostics</span><code>${diagnostics.join(" | ") || "No fitted diagnostics yet."}</code></div>
  `;

  elements.tcpCalibrationSamples.innerHTML = "";
  if (!samples.length) {
    elements.tcpCalibrationSamples.innerHTML = `<div class="empty-state">No TCP samples saved for the active tool.</div>`;
  } else {
    samples.slice().reverse().forEach((sample) => {
      const model = sample.residuals?.model_mm || {};
      const landing = sample.residuals?.landing_mm || {};
      const item = document.createElement("div");
      item.className = "program-item";
      item.innerHTML = `
        <div class="program-title"><span>${sample.role || "fit"} sample</span><span>q ${format(sample.quality, 2)}</span></div>
        <code>${sample.id} | model XY ${format(model.xy, 2)}, Z ${format(model.z, 2)} | landing XY ${format(landing.xy, 2)}</code>
        <div class="button-row"><button type="button" class="ghost danger" data-tcp-cal-delete="${sample.id}">Delete</button></div>
      `;
      elements.tcpCalibrationSamples.appendChild(item);
    });
  }
}

async function generateTcpCalibrationTargets() {
  const payload = await postJson("/api/kinematics-calibration/targets", {
    rows: readRequiredNumber(elements.tcpCalRowsInput, "Grid rows", { min: 1, max: 10, integer: true }),
    columns: readRequiredNumber(elements.tcpCalColumnsInput, "Grid columns", { min: 1, max: 10, integer: true }),
    margin_mm: readRequiredNumber(elements.tcpCalMarginInput, "Workspace margin", { min: 0 }),
    z_mm: readRequiredNumber(elements.tcpCalTargetZInput, "Target Z"),
    phi_deg: readRequiredNumber(elements.tcpCalTargetPhiInput, "Tool pitch"),
    apply_calibration: false,
  });
  if (!payload.ok) return;
  state.tcpCalibrationTargets = payload.points || [];
  renderTcpCalibrationTargets();
  const firstReachable = state.tcpCalibrationTargets.find((point) => point.reachable);
  if (firstReachable) setTcpCalibrationTarget(firstReachable.intended_target);
  elements.tcpCalibrationMoveStatus.textContent = `${payload.reachability?.reachable_count || 0} reachable; ${payload.reachability?.unreachable_count || 0} blocked by IK.`;
}

async function previewTcpCalibrationMove(applyCalibration, role) {
  let target;
  try {
    target = tcpCalibrationTarget();
  } catch (error) {
    showLocalError(error?.message || String(error));
    return;
  }
  elements.tcpCalibrationMoveStatus.textContent = "Previewing calibration move...";
  const payload = await postJson("/api/path/preview", {
    target,
    mode: "joint",
    branch: "auto",
    settings: pathSettings(),
    apply_calibration: Boolean(applyCalibration),
  });
  if (!payload.ok) {
    elements.tcpCalibrationMoveStatus.textContent = `${payload.diagnostic_category || "preview"}: ${payload.error || "failed"}`;
    return;
  }
  renderPreview(payload.preview);
  state.tcpCalibrationMove = {
    role,
    intended_target: payload.preview.target || target,
    command_target: payload.preview.command_target || target,
    calibration: payload.preview.calibration || {},
    preview_id: payload.preview_id,
    executed: false,
  };
  elements.tcpCalRoleSelect.value = role;
  elements.tcpCalExecuteBtn.disabled = false;
  elements.tcpCalibrationMoveStatus.textContent = applyCalibration
    ? "Validation preview uses the fitted correction. Execute when the path is safe."
    : "Fit preview is uncorrected. Execute when the path is safe.";
}

async function executeTcpCalibrationMove() {
  const move = state.tcpCalibrationMove;
  if (!move?.preview_id) return;
  const payload = await postJson("/api/path/execute", { preview_id: move.preview_id });
  if (!payload.ok) {
    elements.tcpCalibrationMoveStatus.textContent = payload.error || "Calibration move could not start.";
    return;
  }
  move.executed = true;
  elements.tcpCalExecuteBtn.disabled = true;
  elements.tcpCalibrationMoveStatus.textContent = "Move started. Wait for idle before capturing the sample.";
  if (payload.state) renderState(payload.state);
}

async function captureTcpCalibrationXy() {
  elements.tcpCalibrationMeasurementStatus.textContent = "Capturing TCP marker...";
  const response = await fetch(`/api/vision/frame?t=${Date.now()}`, { cache: "no-store" });
  const payload = await response.json();
  if (!payload.ok) {
    elements.tcpCalibrationMeasurementStatus.textContent = payload.error || "Vision capture failed.";
    return;
  }
  const requestedLabel = String(elements.tcpCalMarkerLabelInput.value || "").trim().toLowerCase();
  const candidates = (payload.detections || [])
    .filter((detection) => detection.ok && detection.robot)
    .filter((detection) => {
      if (!requestedLabel) return true;
      return [detection.id, detection.label, detection.color, detection.object_id]
        .some((value) => String(value ?? "").toLowerCase() === requestedLabel);
    })
    .sort((a, b) => Number(b.confidence || 0) - Number(a.confidence || 0));
  if (!candidates.length) {
    elements.tcpCalibrationMeasurementStatus.textContent = requestedLabel
      ? `No calibrated detection matched "${requestedLabel}".`
      : "No calibrated TCP marker detection was available.";
    return;
  }
  const marker = candidates[0];
  elements.tcpCalMeasuredXInput.value = format(marker.robot.x_mm, 3);
  elements.tcpCalMeasuredYInput.value = format(marker.robot.y_mm, 3);
  if (marker.confidence != null) elements.tcpCalQualityInput.value = String(clamp(Number(marker.confidence), 0.2, 1));
  state.tcpCalibrationMeasurementSource.xy = {
    type: "vision",
    provider: payload.provider,
    calibration_source: payload.calibration_source,
    detection_id: marker.id || marker.object_id,
    label: marker.label || marker.color,
  };
  elements.tcpCalibrationMeasurementStatus.textContent = `Captured ${marker.label || marker.id || "marker"} at X ${format(marker.robot.x_mm, 2)}, Y ${format(marker.robot.y_mm, 2)} mm.`;
}

function useTcpCalibrationTouchOff() {
  try {
    const surface = readRequiredNumber(elements.tcpCalSurfaceZInput, "Known surface Z");
    const offset = readRequiredNumber(elements.tcpCalContactOffsetInput, "TCP contact offset");
    const measuredZ = surface + offset;
    elements.tcpCalMeasuredZInput.value = format(measuredZ, 3);
    state.tcpCalibrationMeasurementSource.z = {
      type: "touch_off",
      surface_z_mm: surface,
      contact_offset_mm: offset,
    };
    elements.tcpCalibrationMeasurementStatus.textContent = `Touch-off Z set to ${format(measuredZ, 2)} mm. No automatic contact motion was performed.`;
  } catch (error) {
    showLocalError(error?.message || String(error));
  }
}

async function saveTcpCalibrationSample() {
  let intended;
  let measured;
  try {
    intended = state.tcpCalibrationMove?.intended_target || tcpCalibrationTarget();
    measured = {
      x_mm: readRequiredNumber(elements.tcpCalMeasuredXInput, "Measured X"),
      y_mm: readRequiredNumber(elements.tcpCalMeasuredYInput, "Measured Y"),
      z_mm: readRequiredNumber(elements.tcpCalMeasuredZInput, "Measured Z"),
    };
  } catch (error) {
    showLocalError(error?.message || String(error));
    return;
  }
  const payload = await postJson("/api/kinematics-calibration/samples", {
    intended_target: intended,
    command_target: state.tcpCalibrationMove?.command_target || intended,
    measured,
    role: elements.tcpCalRoleSelect.value,
    quality: readNumber(elements.tcpCalQualityInput, 1),
    measurement_source: state.tcpCalibrationMeasurementSource,
    joint_source: state.robotState?.encoder_available?.includes?.("1") ? "reported_encoder" : "reported_or_commanded",
    notes: state.tcpCalibrationMove?.executed ? "captured after calibration workflow move" : "captured without confirmed workflow execution",
  });
  if (!payload.ok) {
    elements.tcpCalibrationMeasurementStatus.textContent = payload.error || "Sample was rejected.";
    return;
  }
  if (payload.config) applyConfig(payload.config);
  state.tcpCalibrationMove = null;
  elements.tcpCalibrationMeasurementStatus.textContent = `Saved ${payload.sample.role} sample ${payload.sample.id}.`;
}

async function fitTcpCalibration() {
  elements.tcpCalibrationMetrics.innerHTML = `<div class="log-line"><span>Status</span><code>Fitting...</code></div>`;
  const payload = await postJson("/api/kinematics-calibration/fit", {
    model_type: elements.tcpCalModelSelect.value,
    enable_after_fit: Boolean(elements.tcpCalEnableInput.checked),
  });
  if (!payload.ok) {
    elements.tcpCalibrationMetrics.innerHTML = `<div class="log-line"><span>Fit error</span><code>${payload.error || "failed"}</code></div>`;
    return;
  }
  if (payload.config) applyConfig(payload.config);
}

async function applyTcpCalibrationEnableState() {
  const payload = await postJson("/api/kinematics-calibration/enable", {
    enabled: Boolean(elements.tcpCalEnableInput.checked),
  });
  if (payload.ok && payload.config) applyConfig(payload.config);
}

async function deleteTcpCalibrationSample(sampleId) {
  const response = await fetch(`/api/kinematics-calibration/samples/${encodeURIComponent(sampleId)}`, {
    method: "DELETE",
  });
  const payload = await response.json();
  if (!payload.ok) {
    showLocalError(payload.error || "Sample could not be deleted.");
    return;
  }
  if (payload.config) applyConfig(payload.config);
}

function renderDetectionList(detections) {
  if (!elements.detectionList) return;
  if (!detections.length) {
    elements.detectionList.innerHTML = "";
    const empty = document.createElement("div");
    empty.className = "empty-state";
    empty.textContent = "No colored objects detected inside the workspace.";
    elements.detectionList.appendChild(empty);
    renderProgramSourceOptions();
    return;
  }
  const waitingRun = state.robotState?.task_execution?.status === "waiting_for_selection";
  const candidateIds = new Set((state.robotState?.task_execution?.candidate_objects || []).map((item) => String(item.detection_id)));
  let table = elements.detectionList.querySelector("table");
  if (!table) {
    elements.detectionList.innerHTML = "";
    table = document.createElement("table");
    table.innerHTML = `
      <thead>
        <tr>
          <th>Use</th>
          <th>Color</th>
          <th>Conf</th>
          <th>Area</th>
          <th>Robot X/Y</th>
          <th>Eligibility</th>
          <th>Reason</th>
        </tr>
      </thead>
      <tbody></tbody>
    `;
    elements.detectionList.appendChild(table);
  }
  const tbody = table.querySelector("tbody");
  const seen = new Set();
  detections.forEach((detection, index) => {
    const detectionId = String(detection.id || detection.detection_id || detection.object_id || `detection-${index + 1}`);
    seen.add(detectionId);
    let row = tbody.querySelector(`tr[data-detection-row="${CSS.escape(detectionId)}"]`);
    if (!row) {
      row = document.createElement("tr");
      row.dataset.detectionRow = detectionId;
    }
    const label = detection.label || detection.color || "object";
    const area = detection.area_px ?? (detection.bbox_px ? Number(detection.bbox_px.width || 0) * Number(detection.bbox_px.height || 0) : null);
    const robot = detection.robot
      ? `${format(detection.robot.x_mm)}, ${format(detection.robot.y_mm)}`
      : "-";
    const eligible = Boolean(detection.ok && detection.robot);
    const selected = state.selectedDetectionIds.has(detectionId);
    row.className = `${eligible ? "" : "invalid"} ${selected ? "selected" : ""}`;
    row.innerHTML = `
      <td>
        <input type="checkbox" data-detection-select="${detectionId}" ${selected ? "checked" : ""} ${eligible ? "" : "disabled"} />
      </td>
      <td><strong>${label}</strong><small>${detectionId}</small></td>
      <td>${detection.confidence == null ? "-" : format(detection.confidence, 2)}</td>
      <td>${area == null ? "-" : format(area, 0)}</td>
      <td>${robot}</td>
      <td>${eligible ? "eligible" : "ignored"}</td>
      <td>${detection.projection_error || detection.reason || detection.coordinate_source || "-"}</td>
    `;
    if (waitingRun && candidateIds.has(detectionId)) {
      const action = document.createElement("button");
      action.type = "button";
      action.className = "ghost compact-action";
      action.textContent = "Pick";
      action.dataset.runtimeSelect = detectionId;
      row.children[0].appendChild(action);
    }
    tbody.appendChild(row);
  });
  [...tbody.querySelectorAll("tr[data-detection-row]")].forEach((row) => {
    if (!seen.has(row.dataset.detectionRow)) row.remove();
  });
  renderProgramSourceOptions();
}

function hardwareDraftState(index, actuator) {
  const patch = readHardwarePatch(index, actuator)[actuator] || {};
  if (!patch.enabled) return "draft simulated";
  if (actuator === "stepper") {
    const valid =
      patch.step_pin >= 0 &&
      patch.dir_pin >= 0 &&
      patch.motor_full_steps_per_rev > 0 &&
      patch.microsteps > 0 &&
      patch.gear_ratio > 0;
    return valid ? "draft hardware" : "draft invalid";
  }
  const valid =
    patch.pwm_pin >= 0 &&
    patch.pulse_min_us > 0 &&
    patch.pulse_max_us > patch.pulse_min_us &&
    patch.pwm_frequency_hz > 0 &&
    patch.servo_range_deg > 0 &&
    patch.gear_ratio > 0;
  return valid ? "draft hardware" : "draft invalid";
}

function renderHardwareDraftBadges() {
  if (!elements.hardwareIo || !state.config) return;
  elements.hardwareIo.querySelectorAll("[data-hardware-index]").forEach((row) => {
    const index = Number(row.dataset.hardwareIndex);
    const joint = state.config.joints[index];
    if (!joint) return;
    const badge = row.querySelector(".hardware-title .badge");
    row.classList.toggle("draft-dirty", state.hardwareDraftDirty);
    if (badge) badge.textContent = hardwareDraftState(index, joint.actuator);
  });
}

function markHardwareDraftDirty() {
  state.hardwareDraftDirty = true;
  renderHardwareDraftBadges();
  markSettingsDirty("hardware", "Hardware I/O changed. Save all settings, or use Save and sync controller below.");
}

function markJointCalibrationDraftDirty() {
  const validation = validateJointCalibrationDraft();
  markSettingsDirty(
    "joints",
    validation.ok
      ? "Joint limits or calibration changed. Save all settings to apply them."
      : `Fix joint calibration before saving: ${validation.errors.join("; ")}`
  );
}

function markToolDraftDirty() {
  markSettingsDirty("tooling", "Tool dimensions or I/O changed. Save all settings to apply them.");
}

function buildIkTargetControls() {
  elements.ikTargetControls.innerHTML = "";
  targetDefs.forEach(([key, label, , unit, step]) => {
    const row = document.createElement("div");
    row.className = "target-row";
    row.innerHTML = `
      <strong>${label}</strong>
      <input id="ik-${key}-slider" data-target-key="${key}" type="range" step="${step}" />
      <input id="ik-${key}-input" data-target-key="${key}" type="number" step="${step}" aria-label="${label} target" title="${unit}" />
    `;
    elements.ikTargetControls.appendChild(row);
  });
}

function buildSliderRanges() {
  const links = linkDefaults();
  const reach = links.l2 + links.l3 + links.l4;
  const ranges = {
    x: [-reach, reach],
    y: [-reach, reach],
    z: [Math.max(0, links.l1 - reach), links.l1 + reach],
    phi: [-180, 180],
  };
  elements.sliderRangeControls.innerHTML = "";
  targetDefs.forEach(([key, label]) => {
    const field = document.createElement("label");
    field.innerHTML = `${label} min/max
      <div class="range-row">
        <input data-range-key="${key}" data-range-side="min" type="number" step="1" value="${format(ranges[key][0], 0)}" />
        <input data-range-key="${key}" data-range-side="max" type="number" step="1" value="${format(ranges[key][1], 0)}" />
      </div>`;
    elements.sliderRangeControls.appendChild(field);
  });
  applySliderRanges();
}

function applySliderRanges() {
  targetDefs.forEach(([key]) => {
    const slider = $(`#ik-${key}-slider`);
    const number = $(`#ik-${key}-input`);
    const min = readNumber($(`[data-range-key="${key}"][data-range-side="min"]`), -100);
    const max = readNumber($(`[data-range-key="${key}"][data-range-side="max"]`), 100);
    if (!slider || !number) return;
    slider.min = String(min);
    slider.max = String(max);
    number.min = String(min);
    number.max = String(max);
    const next = clamp(readNumber(number, 0), min, max);
    slider.value = String(next);
    number.value = format(next, key === "phi" ? 1 : 0);
  });
}

function setIkTargetValue(key, value, userEdited = true) {
  const slider = $(`#ik-${key}-slider`);
  const input = $(`#ik-${key}-input`);
  if (!slider || !input) return;
  const decimals = key === "phi" ? 1 : 0;
  const next = clamp(Number(value), Number(slider.min), Number(slider.max));
  slider.value = String(next);
  input.value = format(next, decimals);
  if (userEdited) {
    state.ikUserEdited = true;
    if (state.intentBasePoseRevision == null) {
      state.intentBasePoseRevision = Number(state.robotState?.pose_revision ?? 0);
    }
  }
}

function setIkTargetFromFk(fk) {
  if (state.ikUserEdited || !fk) return;
  const values = {
    x: fk.x_mm || 0,
    y: fk.y_mm || 0,
    z: fk.z_mm || 0,
    phi: fk.tool_phi_deg ?? fk.tool_pitch_deg ?? 0,
  };
  Object.entries(values).forEach(([key, value]) => setIkTargetValue(key, value, false));
}

function ikAutoPhiEnabled() {
  return Boolean(elements.ikAutoPhiToggle?.checked);
}

function updatePhiControlState() {
  const autoPhi = ikAutoPhiEnabled();
  ["#ik-phi-slider", "#ik-phi-input"].forEach((selector) => {
    const input = $(selector);
    if (input) input.disabled = autoPhi;
  });
  document.querySelectorAll('[data-fader-key="phi"]').forEach((fader) => {
    fader.classList.toggle("disabled", autoPhi);
    fader.setAttribute("aria-disabled", autoPhi ? "true" : "false");
  });
}

function ikTargetPayload() {
  const autoPhi = ikAutoPhiEnabled();
  const payload = {};
  targetDefs.forEach(([key, , apiKey]) => {
    if (key === "phi" && autoPhi) return;
    const input = $(`#ik-${key}-input`);
    payload[apiKey] = readNumber(input, 0);
  });
  payload.phi_auto = autoPhi;
  return payload;
}

function formatCartesianTarget(target = {}) {
  const phiText = target.phi_auto
    ? `phi auto${target.phi_deg !== undefined && target.phi_deg !== null ? ` (${format(target.phi_deg, 1)} deg)` : ""}`
    : `phi ${format(target.phi_deg)}`;
  return `x ${format(target.x_mm)}, y ${format(target.y_mm)}, z ${format(target.z_mm)}, ${phiText}`;
}

function clearIkSolutionPreview() {
  state.previewId = null;
  state.latestPreview = null;
  state.previewAngles = null;
  state.previewBasePoseRevision = null;
  state.taskPreviewId = null;
  if (state.view) {
    state.view.setPreviewAngles(null);
    state.view.setPathWaypoints([]);
  }
  if (elements.ikCandidateList) elements.ikCandidateList.innerHTML = "";
  if (elements.ikPathSummary) {
    elements.ikPathSummary.innerHTML = `
      <h3>Path</h3>
      <div class="log-line"><span>Status</span><code>No preview</code></div>
    `;
  }
  elements.pathHud.textContent = "0 pts";
  elements.executeIkBtn.disabled = true;
  elements.executeProgramBtn.disabled = true;
  syncJointControls();
}

function updateIkTargetMarker(options = {}) {
  if (!state.view) return null;
  const target = ikTargetPayload();
  if (options.clearSolution !== false) clearIkSolutionPreview();
  state.viewPreviewSource = "ik-target";
  state.view.setTargetPoint(target);
  elements.targetHud.textContent = `x ${format(target.x_mm)}, y ${format(target.y_mm)}, z ${format(target.z_mm)}`;
  if (options.status !== false) elements.previewStatus.textContent = "Target set";
  updateDisabledState();
  return target;
}

function pathSettings() {
  return {
    global_speed_deg_s: readNumber(elements.globalSpeedInput, 25),
    global_accel_deg_s2: readNumber(elements.globalAccelInput, 120),
    waypoint_rate_hz: readNumber(elements.waypointRateInput, 12),
    cartesian_step_mm: readNumber(elements.cartesianStepInput, 10),
    planner_type: elements.plannerTypeSelect.value,
    jerk_percent: readNumber(elements.jerkPercentInput, 25),
    blend_percent: readNumber(elements.blendPercentInput, 0),
    per_joint_speed_deg_s: [...document.querySelectorAll(".joint-speed-limit")].map((input) => readNumber(input, 1)),
    per_joint_accel_deg_s2: [...document.querySelectorAll(".joint-accel-limit")].map((input) => readNumber(input, 1)),
  };
}

function taskPathSettingsPayload() {
  const settings = pathSettings();
  settings.global_speed_deg_s = readRequiredNumber(elements.globalSpeedInput, "Global speed", { min: 0.001 });
  settings.global_accel_deg_s2 = readRequiredNumber(elements.globalAccelInput, "Global acceleration", { min: 0.001 });
  settings.waypoint_rate_hz = readRequiredNumber(elements.waypointRateInput, "Waypoint rate", { min: 0.001 });
  settings.cartesian_step_mm = readRequiredNumber(elements.cartesianStepInput, "Cartesian step", { min: 0.001 });
  settings.jerk_percent = readRequiredNumber(elements.jerkPercentInput, "Jerk percent", { min: 0, max: 100 });
  settings.blend_percent = readRequiredNumber(elements.blendPercentInput, "Blend percent", { min: 0, max: 100 });
  settings.per_joint_speed_deg_s = [...document.querySelectorAll(".joint-speed-limit")]
    .map((input, index) => readRequiredNumber(input, `Joint ${index + 1} speed`, { min: 0.001 }));
  settings.per_joint_accel_deg_s2 = [...document.querySelectorAll(".joint-accel-limit")]
    .map((input, index) => readRequiredNumber(input, `Joint ${index + 1} acceleration`, { min: 0.001 }));
  return settings;
}

function normalizeJointAngles(values) {
  const expectedCount = state.config?.joints?.length || state.robotState?.reported_angles_deg?.length || 4;
  if (!Array.isArray(values) || values.length !== expectedCount) return null;
  const angles = values.map(Number);
  return angles.every(Number.isFinite) ? angles : null;
}

function previewEndpointAngles(preview = state.latestPreview) {
  const waypoints = preview?.trajectory?.waypoints;
  const lastWaypoint = Array.isArray(waypoints) && waypoints.length ? waypoints[waypoints.length - 1] : null;
  return normalizeJointAngles(lastWaypoint) || normalizeJointAngles(preview?.ik?.selected?.angles_deg);
}

function jointControlAngles(robotState = state.robotState) {
  return (
    normalizeJointAngles(state.draftAngles) ||
    normalizeJointAngles(state.pendingAngles) ||
    normalizeJointAngles(state.previewAngles) ||
    normalizeJointAngles(state.commandedAngles) ||
    normalizeJointAngles(robotState?.target_angles_deg) ||
    normalizeJointAngles(robotState?.reported_angles_deg) ||
    normalizeJointAngles(state.config?.joints?.map((joint) => joint.home_deg))
  );
}

function syncJointControls(robotState = state.robotState) {
  const targets = jointControlAngles(robotState);
  if (!targets) return;
  const reported = normalizeJointAngles(robotState?.reported_angles_deg) || targets;
  syncJointInputs(targets, reported);
}

function releaseJointControlIntent() {
  window.clearTimeout(state.commandTimer);
  state.commandTimer = null;
  state.pendingAngles = null;
  state.draftAngles = null;
  state.commandedAngles = null;
  state.lastSentAngles = null;
  state.intentBasePoseRevision = null;
  window.clearTimeout(state.liveTargetTimer);
  state.liveTargetTimer = null;
  state.pendingLiveTarget = null;
  state.liveTargetQueued = false;
}

function syncJointInputs(targets, reported) {
  targets.forEach((angle, index) => {
    const slider = elements.jointControls.querySelector(`.joint-slider[data-index="${index}"]`);
    const input = elements.jointControls.querySelector(`.angle-input[data-index="${index}"]`);
    const targetLabel = $(`#target-${index}`);
    const reportedLabel = $(`#reported-${index}`);
    if (!slider || !input || !targetLabel || !reportedLabel) return;
    if (document.activeElement !== slider) slider.value = angle;
    if (document.activeElement !== input) input.value = format(angle, 1);
    targetLabel.textContent = `${format(angle, 1)} deg`;
    reportedLabel.textContent = `${format(reported[index], 1)} deg`;
  });
}

function queueTarget(angles) {
  state.pendingAngles = angles.slice();
  const hz = state.config.motion.command_rate_limit_hz || 12;
  const delayMs = Math.max(25, 1000 / hz);
  if (state.commandTimer) return;
  state.commandTimer = window.setTimeout(sendPendingTarget, delayMs);
}

function sendPendingTarget() {
  if (state.pendingAngles) {
    state.lastSentAngles = state.pendingAngles.slice();
    const payload = { angles_deg: state.pendingAngles, settings: pathSettings() };
    if (state.ws?.readyState === WebSocket.OPEN) {
      state.ws.send(JSON.stringify({ command: "set_all_joint_targets", ...payload }));
    } else {
      postJson("/api/joints", payload);
    }
  }
  state.commandTimer = null;
  if (state.pendingAngles && !anglesAlmostEqual(state.pendingAngles, state.lastSentAngles)) queueTarget(state.pendingAngles);
}

function anglesAlmostEqual(a, b, tolerance = 0.01) {
  if (!a || !b || a.length !== b.length) return false;
  return a.every((value, index) => Math.abs(value - b[index]) <= tolerance);
}

function setBadge(el, text, type) {
  el.textContent = text;
  el.className = `badge ${type || ""}`.trim();
}

function renderMotionExecution(robotState) {
  const diagnostics = robotState.motion_diagnostics || {};
  const executionState = robotState.motion_execution_state || diagnostics.execution_state || robotState.motion_state || "idle";
  const progress = clamp(Number(diagnostics.progress_ratio ?? 0), 0, 1);
  const waypointTotal = Number(diagnostics.current_waypoint_total || diagnostics.waypoint_count || 0);
  const waypointIndex = Number(diagnostics.current_waypoint_index || 0);
  const stepTotal = Number(diagnostics.active_step_total || 0);
  const stepIndex = Number(diagnostics.active_step_index || 0);
  const stepLabel = diagnostics.active_step_label || "";
  const plannedCount = state.latestPreview?.trajectory?.waypoint_count || Number(state.view?.container?.dataset.pathWaypointCount || 0);
  const actualPath = Array.isArray(diagnostics.actual_tcp_path) ? diagnostics.actual_tcp_path : [];
  const actualCount = actualPath.length;
  const waypointText = waypointTotal ? `wp ${waypointIndex}/${waypointTotal}` : "-";
  const stepText = stepTotal ? `step ${stepIndex}/${stepTotal}${stepLabel ? ` ${stepLabel}` : ""}` : "";
  const progressText = `${format(progress * 100, 0)}%${waypointTotal ? ` - ${waypointText}` : ""}${stepText ? ` - ${stepText}` : ""}`;

  if (state.view) state.view.setActualTcpPath(actualPath);
  if (elements.motionHud) elements.motionHud.textContent = executionState;
  if (elements.progressHud) elements.progressHud.textContent = progressText;
  if (elements.pathHud) {
    elements.pathHud.textContent = actualCount
      ? `${plannedCount || 0} plan / ${actualCount} actual`
      : `${plannedCount || 0} pts`;
  }
  const progressLine = $("#motionProgressLine");
  if (progressLine) progressLine.textContent = `${executionState} - ${progressText}`;
}

function renderTaskExecution(robotState = state.robotState) {
  const execution = robotState?.task_execution || {};
  if (!elements.taskRunMonitor) return;
  const executionUpdatedAt = Number(execution.updated_at || 0);
  const localStatusIsNewer = state.taskLocalStatusAt > executionUpdatedAt;
  if (!execution.status) {
    elements.taskRunMonitor.innerHTML = `<div class="empty-state">No task running. Preview first, then start.</div>`;
    if (elements.taskStatus && !localStatusIsNewer) {
      const currentStatus = elements.taskStatus.textContent || "";
      const keepLocalStatus = /preview|task settings|detection|preset|mapping|save|ik|waypoint|unreachable|failed/i.test(currentStatus);
      if (state.taskPreviewId && !keepLocalStatus) {
        elements.taskStatus.textContent = "Preview ready";
      } else if (!keepLocalStatus) {
        elements.taskStatus.textContent = "No task";
      }
    }
    if (!localStatusIsNewer) renderWorkflowStepper(state.taskPreviewId ? "preview" : "setup");
    return;
  }
  const current = execution.current_object || {};
  const step = execution.current_step || {};
  const latest = execution.latest_capture || {};
  const candidates = execution.candidate_objects || [];
  const ignored = execution.ignored_objects || [];
  const warnings = execution.warnings || [];
  if (!localStatusIsNewer) {
    elements.taskStatus.textContent = `${execution.status}: ${execution.phase || "-"}`;
    renderWorkflowStepper(execution.status);
  }
  elements.taskRunMonitor.innerHTML = `
    <div class="task-run-grid">
      <div class="run-metric"><span>Status</span><strong>${execution.status}</strong><small>${execution.terminal_reason || execution.phase || "-"}</small></div>
      <div class="run-metric"><span>Completed</span><strong>${execution.completed_count || 0}</strong><small>remaining ${execution.remaining_count ?? "-"}</small></div>
      <div class="run-metric"><span>Current</span><strong>${current.color || "-"}</strong><small>${current.detection_id || current.drop_zone || "-"}</small></div>
      <div class="run-metric"><span>Step</span><strong>${step.index && step.total ? `${step.index}/${step.total}` : "-"}</strong><small>${step.label || "-"}</small></div>
      <div class="run-metric"><span>Capture</span><strong>${latest.detection_count ?? "-"}</strong><small>${latest.provider || latest.calibration_source || "-"}</small></div>
      <div class="run-metric"><span>Tool feedback</span><strong>${execution.tool_feedback?.status || "unknown"}</strong><small>${execution.holding_uncertain ? "holding uncertain" : "no hold feedback"}</small></div>
    </div>
    ${execution.status === "waiting_for_selection" ? `<div class="task-waiting">Manual mode: choose a candidate with the Pick button in the detection table.</div>` : ""}
    <div class="task-object-queue compact">
      ${candidates.slice(0, 8).map((item) => `<div class="task-object-row"><span>${item.detection_id}</span><strong>${item.color}</strong><code>x ${format(item.robot?.x_mm)}, y ${format(item.robot?.y_mm)}</code><small>${item.drop_zone || "-"}</small>${execution.status === "waiting_for_selection" ? `<button type="button" class="ghost compact-action" data-runtime-select="${item.detection_id}">Pick</button>` : ""}</div>`).join("") || `<div class="empty-state">No candidate queue exposed.</div>`}
    </div>
    <div class="ignored-list">
      ${ignored.slice(0, 6).map((item) => `<div><span>${item.color || item.detection_id}</span><code>${item.reason || item.reason_code}</code></div>`).join("")}
      ${warnings.map((warning) => `<div><span>warning</span><code>${warning}</code></div>`).join("")}
    </div>
  `;
  renderDetectionList(state.latestDetections);
}

function liveRealEnabled() {
  return Boolean(elements.liveRealToggle.checked && state.robotState?.live_motion_enabled);
}

function cartesianJogEnabled() {
  return Boolean(elements.cartesianJogToggle?.checked);
}

function cartesianJogCanRun() {
  if (!state.robotState || !isMoveEnabled()) return false;
  return Boolean(state.robotState.simulation || liveRealEnabled());
}

function cartesianJogRateMs() {
  const hz = clamp(Number(state.config?.motion?.command_rate_limit_hz || 20), 12, 30);
  return Math.max(33, Math.round(1000 / hz));
}

function setCartesianJogStatus(text) {
  if (!elements.cartesianJogStatus) return;
  if (cartesianJogEnabled()) {
    elements.cartesianJogStatus.textContent = `Velocity jog: ${text || "idle"}`;
  } else {
    elements.cartesianJogStatus.textContent = text && text !== "idle"
      ? `IK preview only - ${text}`
      : "IK preview only";
  }
}

function zeroCartesianJogVelocity() {
  state.cartesianJogVelocity = { vx_mm_s: 0, vy_mm_s: 0, vz_mm_s: 0, vphi_deg_s: 0 };
  state.cartesianJogActiveAxes.clear();
}

function axisVelocityPayload(key, velocity) {
  const field = {
    x: "vx_mm_s",
    y: "vy_mm_s",
    z: "vz_mm_s",
    phi: "vphi_deg_s",
  }[key];
  if (!field) return 0;
  const previous = Number(state.cartesianJogVelocity[field] || 0);
  state.cartesianJogVelocity[field] = velocity;
  return previous;
}

function cartesianJogPayload() {
  return {
    ...state.cartesianJogVelocity,
    tcp_speed_mm_s: readNumber(elements.cartesianJogSpeedInput, 60),
    phi_speed_deg_s: readNumber(elements.cartesianJogPhiSpeedInput, 45),
    settings: pathSettings(),
  };
}

function scheduleCartesianJog(key, velocity) {
  if (!cartesianJogEnabled()) return;
  if (!cartesianJogCanRun()) {
    elements.cartesianJogToggle.checked = false;
    zeroCartesianJogVelocity();
    showLocalError(state.robotState?.simulation ? "Cartesian jog is not available." : "Enable Live Real and Arm before hardware Cartesian jog.");
    return;
  }
  const previousVelocity = axisVelocityPayload(key, velocity);
  if (Math.abs(velocity) > 0.001) state.cartesianJogActiveAxes.add(key);
  if (Math.abs(velocity) <= 0.001 && Math.abs(previousVelocity) <= 0.001) return;
  if (state.cartesianJogStopPending || state.cartesianJogStopInFlight) {
    state.cartesianJogQueued = true;
    setCartesianJogStatus("starting");
    return;
  }
  const delayMs = cartesianJogRateMs();
  if (state.cartesianJogInFlight) {
    state.cartesianJogQueued = true;
    return;
  }
  if (state.cartesianJogTimer) return;
  state.cartesianJogTimer = window.setTimeout(sendCartesianJog, delayMs);
}

async function sendCartesianJog() {
  if (!cartesianJogEnabled() || !cartesianJogCanRun()) {
    state.cartesianJogTimer = null;
    return;
  }
  if (state.cartesianJogStopPending || state.cartesianJogStopInFlight) {
    state.cartesianJogQueued = true;
    state.cartesianJogTimer = null;
    return;
  }
  state.cartesianJogTimer = null;
  const requestEpoch = state.cartesianJogEpoch;
  state.cartesianJogLastSentMs = performance.now();
  state.cartesianJogInFlight = true;
  state.cartesianJogQueued = false;
  try {
    const payload = await postJson("/api/cartesian-jog", cartesianJogPayload());
    if (requestEpoch !== state.cartesianJogEpoch) return;
    if (payload.ok) {
      if (payload.state) renderState(payload.state);
      const notes = payload.jog?.notes || [];
      const blocked = payload.jog?.blocked;
      const failureReason = payload.jog?.failure_reason || notes[0] || "blocked";
      setCartesianJogStatus(blocked ? failureReason : "jogging");
    } else {
      elements.cartesianJogToggle.checked = false;
      zeroCartesianJogVelocity();
      setCartesianJogStatus(payload.error || "jog failed");
      updateDisabledState();
    }
  } catch (error) {
    if (requestEpoch !== state.cartesianJogEpoch) return;
    elements.cartesianJogToggle.checked = false;
    zeroCartesianJogVelocity();
    showLocalError(error?.message || "Cartesian jog request failed.");
    setCartesianJogStatus("jog failed");
    updateDisabledState();
  } finally {
    state.cartesianJogInFlight = false;
    if (state.cartesianJogStopPending) {
      void flushCartesianJogStop();
      return;
    }
    const moving = Object.values(state.cartesianJogVelocity).some((value) => Math.abs(value) > 0.001);
    if ((state.cartesianJogQueued || moving) && cartesianJogEnabled()) {
      state.cartesianJogQueued = false;
      state.cartesianJogTimer = window.setTimeout(sendCartesianJog, cartesianJogRateMs());
    }
  }
}

async function flushCartesianJogStop() {
  if (!state.cartesianJogStopPending || state.cartesianJogStopInFlight || state.cartesianJogInFlight) return;
  state.cartesianJogStopInFlight = true;
  try {
    const payload = await postJson("/api/cartesian-jog/stop");
    if (payload.state) renderState(payload.state);
    if (!payload.ok) {
      setCartesianJogStatus(payload.error || "stop failed");
      return;
    }
  } catch (error) {
    showLocalError(error?.message || "Cartesian jog stop failed.");
    setCartesianJogStatus("stop failed");
    return;
  } finally {
    state.cartesianJogStopInFlight = false;
    state.cartesianJogStopPending = false;
  }

  const moving = Object.values(state.cartesianJogVelocity).some((value) => Math.abs(value) > 0.001);
  if (moving && cartesianJogEnabled() && cartesianJogCanRun()) {
    state.cartesianJogQueued = false;
    state.cartesianJogLastSentMs = 0;
    setCartesianJogStatus("starting");
    state.cartesianJogTimer = window.setTimeout(sendCartesianJog, 0);
  } else {
    state.cartesianJogQueued = false;
    setCartesianJogStatus("idle");
  }
}

async function stopCartesianJog() {
  state.cartesianJogEpoch += 1;
  window.clearTimeout(state.cartesianJogTimer);
  state.cartesianJogTimer = null;
  state.cartesianJogQueued = false;
  zeroCartesianJogVelocity();
  state.cartesianJogLastSentMs = 0;
  state.cartesianJogStopPending = true;
  setCartesianJogStatus("stopping");
  await flushCartesianJogStop();
}

async function setCurrentPoseKnown() {
  const angles =
    normalizeJointAngles(state.draftAngles) ||
    normalizeJointAngles(state.robotState?.target_angles_deg) ||
    normalizeJointAngles(state.robotState?.reported_angles_deg) ||
    normalizeJointAngles(state.config?.joints?.map((joint) => joint.home_deg)) ||
    [];
  if (angles.length !== 4 || angles.some((angle) => !Number.isFinite(angle))) {
    showLocalError("Cannot set pose: joint angles are incomplete.");
    return;
  }
  if (!window.confirm(
    "Set Pose does not move or physically home the robot. It asserts that the displayed joint angles match the real arm. Continue only after checking every joint while hardware is disarmed?"
  )) return;
  elements.statusPill.textContent = "Setting known pose...";
  const payload = await postJson("/api/hardware/setpose", { angles_deg: angles });
  if (payload.ok) {
    invalidatePendingIkPreview();
    releaseJointControlIntent();
    clearViewPreview();
    state.ikUserEdited = false;
    elements.statusPill.textContent = "Pose marked known.";
  }
  if (payload.state) renderState(payload.state);
}

function programMotionGateReason(robotState = state.robotState) {
  if (!robotState) return "Robot state is not available.";
  if (robotState.motion_state === "estop") return "Clear the emergency stop before execution.";
  if (robotState.motion_state === "fault") return "Clear the robot fault before execution.";
  if (robotState.simulation) return "";
  if (!robotState.connected) return "Connect to the robot before execution.";
  if (!robotState.hardware_armed) return "Enable the Armed toggle before execution.";
  if (!robotState.known_pose) return "Set or read a known robot pose before execution.";
  if (robotState.hardware_mode === "invalid") return "Fix the hardware configuration before execution.";
  if (robotState.hardware_mode === "simulated") return "No hardware axes are enabled.";
  if (robotState.config_sync_status !== "synced") {
    return `Sync the hardware configuration before execution (${robotState.config_sync_status || "unknown"}).`;
  }
  return "";
}

function programPreviewIsFresh() {
  return Boolean(
    state.programPreview &&
    state.programPreviewRevision === state.programRevision &&
    state.previewId === state.programPreview.id &&
    state.latestPreview?.mode === "program"
  );
}

function programSelectedSourceIsReady() {
  const source = elements.programStepSource?.value;
  if (!["named_position", "vision_detection", "drop_zone"].includes(source)) return true;
  return Boolean(elements.programSourceItem?.value);
}

function updateDisabledState() {
  const enabled = isMoveEnabled();
  elements.jointControls.querySelectorAll("input").forEach((input) => {
    input.disabled = !enabled;
  });
  elements.applyJointPreviewBtn.disabled = !enabled || !state.draftAngles;
  elements.resetJointPreviewBtn.disabled = !state.draftAngles && !state.commandedAngles;
  elements.homeBtn.disabled = !enabled;
  elements.setPoseBtn.disabled =
    !state.robotState ||
    (!state.robotState.connected && !state.robotState.simulation) ||
    Boolean(state.previewAngles);
  elements.stopBtn.disabled = !state.robotState?.connected && !state.robotState?.simulation;
  if (elements.hardwareArmToggle) {
    elements.hardwareArmToggle.disabled =
      !state.robotState?.connected ||
      state.robotState?.simulation ||
      !state.robotState?.known_pose ||
      state.robotState?.config_sync_status !== "synced";
  }
  if (elements.syncHardwareBtn) {
    elements.syncHardwareBtn.disabled =
      !state.robotState?.connected ||
      state.robotState?.simulation ||
      Boolean(state.robotState?.hardware_armed) ||
      state.robotState?.motion_state === "moving";
    elements.syncHardwareBtn.title = state.robotState?.hardware_armed
      ? "Disarm hardware before syncing controller configuration"
      : "";
  }
  elements.executeIkBtn.disabled = !state.previewId || !enabled;
  const enabledProgramSteps = state.programWaypoints.filter((step) => step.enabled !== false).length;
  const programGateReason = programMotionGateReason();
  elements.previewProgramBtn.disabled =
    state.programPreviewPending ||
    state.programExecutionActive ||
    enabledProgramSteps === 0;
  elements.executeProgramBtn.disabled =
    !programPreviewIsFresh() ||
    Boolean(programGateReason) ||
    state.programPreviewPending ||
    state.programExecutionActive;
  elements.clearProgramBtn.disabled = state.programWaypoints.length === 0 || state.programExecutionActive;
  elements.addProgramStepBtn.disabled = state.programExecutionActive || !programSelectedSourceIsReady();
  elements.programStepSource.disabled = state.programExecutionActive;
  elements.programSourceItem.disabled = state.programExecutionActive;
  elements.programInsertPosition.disabled = state.programExecutionActive;
  document.querySelectorAll("[data-program-quick-source]").forEach((button) => {
    button.disabled = state.programExecutionActive;
  });
  if (elements.cartesianJogToggle) elements.cartesianJogToggle.disabled = !enabled;
  if (elements.cartesianJogSpeedInput) elements.cartesianJogSpeedInput.disabled = !enabled;
  if (elements.cartesianJogPhiSpeedInput) elements.cartesianJogPhiSpeedInput.disabled = !enabled;
  document.querySelectorAll(".target-fader").forEach((fader) => {
    const disabled = !enabled || (cartesianJogEnabled() && !cartesianJogCanRun()) || (fader.dataset.faderKey === "phi" && ikAutoPhiEnabled());
    fader.classList.toggle("disabled", disabled);
    fader.setAttribute("aria-disabled", disabled ? "true" : "false");
  });
  if (elements.previewTaskBtn) elements.previewTaskBtn.disabled = !state.config;
  if (elements.executeTaskBtn) elements.executeTaskBtn.disabled = !state.taskPreviewId || !enabled || taskDraftBlocksRun();
  if (elements.taskStopBtn) {
    const taskStatus = state.robotState?.task_execution?.status;
    elements.taskStopBtn.disabled = !["queued", "running", "capturing", "planning", "executing", "waiting_for_selection"].includes(taskStatus);
  }
}

function renderState(robotState) {
  const incomingUpdatedAt = Number(robotState?.updated_at || 0);
  if (incomingUpdatedAt && incomingUpdatedAt < state.lastRobotStateUpdatedAt) return;
  const previousPoseRevision = Number(state.robotState?.pose_revision ?? robotState?.pose_revision ?? 0);
  const incomingPoseRevision = Number(robotState?.pose_revision ?? 0);
  state.lastRobotStateUpdatedAt = Math.max(state.lastRobotStateUpdatedAt, incomingUpdatedAt);
  state.robotState = robotState;
  const poseRevisionChanged = incomingPoseRevision !== previousPoseRevision;
  const previewStart = normalizeJointAngles(state.latestPreview?.start_reported_angles_deg);
  const reported = normalizeJointAngles(robotState.reported_angles_deg);
  const previewPoseDelta = previewStart && reported
    ? Math.max(...reported.map((value, index) => Math.abs(value - previewStart[index])))
    : 0;
  const previewIsStale = Boolean(
    state.latestPreview &&
    poseRevisionChanged &&
    previewPoseDelta > 0.1
  );
  const localIntentIsStale = Boolean(
    !state.latestPreview &&
    poseRevisionChanged &&
    state.intentBasePoseRevision != null &&
    state.intentBasePoseRevision !== incomingPoseRevision
  );
  if (previewIsStale || localIntentIsStale) {
    const staleProgramPreview = state.latestPreview?.mode === "program" && state.latestPreview?.source !== "task";
    invalidatePendingIkPreview();
    releaseJointControlIntent();
    clearViewPreview();
    state.ikUserEdited = false;
    if (staleProgramPreview) {
      state.programPreviewRevision = null;
      state.programLastEditReason = "The robot pose changed after preview";
    }
  }
  setBadge(elements.connectionBadge, robotState.connected ? "Connected" : "Disconnected", robotState.connected ? "badge-ok" : "badge-danger");
  setBadge(elements.modeBadge, robotState.simulation ? "Simulation" : "Hardware", robotState.simulation ? "badge-warn" : "badge-ok");
  setBadge(
    elements.motionBadge,
    robotState.motion_state,
    robotState.motion_state === "estop" || robotState.motion_state === "fault" ? "badge-danger" : "",
  );
  setBadge(elements.armedBadge, robotState.hardware_armed ? "Armed" : "Disarmed", robotState.hardware_armed ? "badge-danger" : "");
  if (elements.toolStatus) elements.toolStatus.textContent = robotState.tool_state || "unknown";
  renderToolControls();

  if (!robotState.live_motion_enabled) elements.liveRealToggle.checked = false;
  if (!isMoveEnabled() || (!robotState.simulation && !liveRealEnabled())) {
    if (elements.cartesianJogToggle?.checked) {
      elements.cartesianJogToggle.checked = false;
      zeroCartesianJogVelocity();
      setCartesianJogStatus("idle");
    }
  }
  if (state.commandedAngles && anglesAlmostEqual(robotState.reported_angles_deg, state.commandedAngles, 0.15)) {
    state.commandedAngles = null;
    if (!state.draftAngles && state.viewPreviewSource === "joint") {
      clearViewPreview();
    }
  }
  const jointIntentAngles = state.draftAngles || state.commandedAngles || state.pendingAngles;
  syncJointControls(robotState);
  state.view.setAngles(robotState.reported_angles_deg);
  if (state.activeTab === "joint" && jointIntentAngles) {
    state.viewPreviewSource = "joint";
    state.view.setPreviewAngles(jointIntentAngles);
  } else if (state.viewPreviewSource === "joint" && !state.commandedAngles) {
    clearViewPreview();
  }

  const fk = robotState.fk || {};
  elements.fkX.textContent = `${format(fk.x_mm)} mm`;
  elements.fkY.textContent = `${format(fk.y_mm)} mm`;
  elements.fkZ.textContent = `${format(fk.z_mm)} mm`;
  elements.fkPitch.textContent = `${format(fk.tool_phi_deg ?? fk.tool_pitch_deg)} deg`;
  elements.eeCompact.textContent = `x ${format(fk.x_mm)}, y ${format(fk.y_mm)}, z ${format(fk.z_mm)} mm`;
  elements.lastCommand.textContent = robotState.last_command || "-";
  elements.lastError.textContent = robotState.last_error || "-";
  elements.portStatus.textContent = robotState.serial_port || (robotState.simulation ? "simulation" : "-");
  elements.hardwareArmToggle.checked = Boolean(robotState.hardware_armed);
  renderHardwareStatus(robotState);
  renderSetupChecklist();
  const hardwareSuffix = robotState.simulation ? "" : ` - ${robotState.hardware_mode}/${robotState.config_sync_status}`;
  elements.statusPill.textContent = robotState.last_error || `${robotState.motion_state}${robotState.live_motion_enabled ? " - live real" : ""}${hardwareSuffix}`;
  renderMotionExecution(robotState);
  renderTaskExecution(robotState);
  syncProgramExecutionState(robotState);
  renderProgramWorkflowStatus();
  renderProgramPreviewSummary();
  if (state.cartesianJogActiveAxes.size === 0) setIkTargetFromFk(fk);
  updateDisabledState();
  if (!state.settingsDirtyScopes.size) updateSettingsSaveBar();
}

function renderPreview(preview) {
  releaseJointControlIntent();
  state.taskPreviewId = null;
  state.previewId = preview.id;
  state.latestPreview = preview;
  state.previewAngles = previewEndpointAngles(preview);
  state.previewBasePoseRevision = Number(preview.start_pose_revision ?? state.robotState?.pose_revision ?? 0);
  state.intentBasePoseRevision = state.previewBasePoseRevision;
  state.viewPreviewSource = preview.mode === "program" ? "program" : "ik";
  const ik = preview.ik || {};
  const trajectory = preview.trajectory || {};
  const candidates = ik.candidates || [];
  elements.previewStatus.textContent = `Preview ready: ${preview.mode}`;
  elements.ikCandidateList.innerHTML = "";

  if (!candidates.length) {
    const empty = document.createElement("div");
    empty.className = "candidate";
    empty.textContent = preview.mode === "program" ? "Program preview ready." : (ik.notes || ["No IK candidates"]).join("; ");
    elements.ikCandidateList.appendChild(empty);
  }

  candidates.forEach((candidate) => {
    const item = document.createElement("div");
    item.className = `candidate ${candidate.valid ? "" : "invalid"} ${candidate.branch === ik.selected_branch ? "selected" : ""}`;
    const candidatePhi = candidate.target_phi_deg ?? candidate.fk?.tool_phi_deg;
    const phiDetail = candidate.auto_phi
      ? `chosen phi ${format(candidatePhi, 2)} deg`
      : `phi ${format(candidate.phi_error_deg, 3)} deg`;
    item.innerHTML = `
      <div class="candidate-title">
        <span>${candidate.branch.replace("_", " ")}</span>
        <span>${candidate.valid ? "valid" : "rejected"}</span>
      </div>
      <div class="angle-list">
        ${candidate.angles_deg.map((angle, index) => `<div><span>J${index + 1}</span><strong>${format(angle, 2)} deg</strong></div>`).join("")}
      </div>
      <div class="small">FK error ${format(candidate.position_error_mm, 3)} mm, ${phiDetail}</div>
      <div class="small">${candidate.reasons?.join("; ") || ""}</div>
    `;
    elements.ikCandidateList.appendChild(item);
  });

  const segments = trajectory.segments || [];
  const segmentText = segments.length ? segments.map((segment) => `${segment.type}/${segment.mode}:${segment.waypoint_count}`).join(", ") : "-";
  const calibrationApplied = Array.isArray(preview.calibration)
    ? preview.calibration.some((item) => item.applied)
    : Boolean(preview.calibration?.applied);
  const commandTarget = preview.command_target || {};
  elements.ikPathSummary.innerHTML = `
    <h3>Path</h3>
    <div class="log-line"><span>Mode</span><code>${trajectory.mode || preview.mode || "-"}</code></div>
    <div class="log-line"><span>Profile</span><code>${trajectory.profile || "-"}</code></div>
    <div class="log-line"><span>Path type</span><code>${pathLayerDescription(preview, trajectory)}</code></div>
    <div class="log-line"><span>Duration</span><code>${format(trajectory.duration_s, 2)} s</code></div>
    <div class="log-line"><span>Waypoints</span><code>${trajectory.waypoint_count || 0}</code></div>
    <div class="log-line"><span>Branch</span><code>${ik.selected_branch || "-"}</code></div>
    <div class="log-line"><span>Target phi</span><code>${preview.target?.phi_auto ? `auto -> ${format(preview.target.phi_deg, 2)} deg` : `${format(preview.target?.phi_deg, 2)} deg`}</code></div>
    <div class="log-line"><span>TCP calibration</span><code>${calibrationApplied ? "applied at Cartesian command layer" : "not applied"}</code></div>
    <div class="log-line"><span>Model command</span><code>${commandTarget.x_mm !== undefined ? `x ${format(commandTarget.x_mm, 2)}, y ${format(commandTarget.y_mm, 2)}, z ${format(commandTarget.z_mm, 2)}` : preview.mode === "program" ? "per Cartesian waypoint" : "-"}</code></div>
    <div class="log-line"><span>Execute</span><code id="motionProgressLine">idle - 0%</code></div>
    <div class="log-line"><span>Segments</span><code>${segmentText}</code></div>
  `;

  if (state.previewAngles) state.view.setPreviewAngles(state.previewAngles);

  const target = preview.target?.x_mm !== undefined ? preview.target : trajectory.cartesian_waypoints?.[trajectory.cartesian_waypoints.length - 1];
  state.view.setTargetPoint(target || null);
  state.view.setPathWaypoints(
    trajectory.waypoints || [],
    trajectory.physical_cartesian_waypoints || null
  );
  elements.targetHud.textContent = target ? `x ${format(target.x_mm)}, y ${format(target.y_mm)}, z ${format(target.z_mm)}` : "none";
  elements.pathHud.textContent = `${trajectory.waypoint_count || 0} pts`;
  syncJointControls();
  updateDisabledState();
}

function renderPreviewFailure(payload) {
  state.previewId = null;
  state.latestPreview = null;
  if (state.activeTab === "ik") {
    clearIkSolutionPreview();
    updateIkTargetMarker({ clearSolution: false, status: false });
  } else {
    clearViewPreview();
  }
  elements.executeIkBtn.disabled = true;
  elements.executeProgramBtn.disabled = true;
  elements.previewStatus.textContent = payload.error || "Preview failed";
  elements.ikCandidateList.innerHTML = "";
  const ik = payload.ik || {};
  (ik.candidates || []).forEach((candidate) => {
    const item = document.createElement("div");
    item.className = `candidate ${candidate.valid ? "" : "invalid"}`;
    item.textContent = `${candidate.branch}: ${candidate.reasons?.join("; ") || "not selected"}`;
    elements.ikCandidateList.appendChild(item);
  });
  if (!elements.ikCandidateList.children.length) {
    const item = document.createElement("div");
    item.className = "candidate invalid";
    item.textContent = payload.error || "Preview failed before IK candidates were generated.";
    elements.ikCandidateList.appendChild(item);
  }
  elements.ikPathSummary.innerHTML = `<h3>Path</h3><div class="log-line"><span>Error</span><code>${payload.error || "-"}</code></div>`;
}

function pathLayerDescription(preview, trajectory) {
  const mode = String(trajectory.mode || preview.mode || "path").toLowerCase();
  if (mode === "linear") return "planned linear TCP path";
  if (mode === "joint" || mode === "jog") return "joint-space TCP estimate";
  if (mode === "program") return "program TCP estimate";
  return `${mode} TCP estimate`;
}

async function previewIkPath() {
  window.clearTimeout(state.ikPreviewTimer);
  state.ikPreviewTimer = null;
  if (state.ikPreviewInFlight) {
    state.ikPreviewQueued = true;
    return;
  }

  state.ikPreviewInFlight = true;
  state.ikPreviewQueued = false;
  state.lastIkPreviewMs = performance.now();
  const requestSeq = state.ikPreviewWantedSeq;
  state.ikPreviewSeq = requestSeq;
  elements.previewStatus.textContent = "Previewing...";
  elements.executeIkBtn.disabled = true;

  try {
    const payload = await postJson("/api/path/preview", {
      target: ikTargetPayload(),
      mode: elements.ikModeSelect.value,
      branch: elements.ikBranchSelect.value,
      settings: pathSettings(),
    });
    if (requestSeq === state.ikPreviewWantedSeq) {
      if (payload.ok) renderPreview(payload.preview);
      else renderPreviewFailure(payload);
    }
  } finally {
    state.ikPreviewInFlight = false;
    if (state.ikPreviewQueued) scheduleIkPreview(0);
  }
}

function scheduleIkPreview(delayMs = 0) {
  if (!state.config || !state.robotState) return;
  state.ikPreviewWantedSeq += 1;
  window.clearTimeout(state.ikPreviewTimer);
  state.ikPreviewTimer = window.setTimeout(() => updateIkTargetMarker(), Math.max(0, delayMs));
}

function invalidatePendingIkPreview() {
  state.ikPreviewWantedSeq += 1;
  state.ikPreviewQueued = false;
  window.clearTimeout(state.ikPreviewTimer);
  state.ikPreviewTimer = null;
}

function scheduleLiveTarget(payload, delayMs = 90) {
  if (!liveRealEnabled()) return;
  state.pendingLiveTarget = payload;
  if (state.liveTargetInFlight) {
    state.liveTargetQueued = true;
    return;
  }
  window.clearTimeout(state.liveTargetTimer);
  state.liveTargetTimer = window.setTimeout(sendLiveTarget, delayMs);
}

async function sendLiveTarget() {
  if (!state.pendingLiveTarget || !liveRealEnabled()) return;
  const payload = state.pendingLiveTarget;
  state.pendingLiveTarget = null;
  state.liveTargetInFlight = true;
  state.liveTargetQueued = false;
  try {
    const response = await postJson("/api/live-target", { ...payload, settings: pathSettings() });
    if (response.state) renderState(response.state);
  } finally {
    state.liveTargetInFlight = false;
    if (state.liveTargetQueued && state.pendingLiveTarget) scheduleLiveTarget(state.pendingLiveTarget, 0);
  }
}

async function executePreview() {
  if (!state.previewId) return;
  const previewId = state.previewId;
  const payload = await postJson("/api/path/execute", { preview_id: previewId });
  if (payload.ok) {
    releaseJointControlIntent();
    state.previewId = null;
    state.previewAngles = null;
    state.taskPreviewId = null;
    state.ikUserEdited = false;
  } else if (/preview|configuration|model|start pose/i.test(payload.error || "")) {
    clearViewPreview();
    elements.previewStatus.textContent = payload.error || "Preview is stale";
  }
  if (payload.state) renderState(payload.state);
  else syncJointControls();
  updateDisabledState();
}

function nextProgramStepId() {
  const id = `program-step-${state.programNextId}`;
  state.programNextId += 1;
  return id;
}

function selectedProgramIndex() {
  return state.programWaypoints.findIndex((step) => step.id === state.programSelectedId);
}

function selectedProgramStep() {
  const index = selectedProgramIndex();
  return index >= 0 ? state.programWaypoints[index] : null;
}

function uniqueProgramLabel(baseLabel) {
  const base = String(baseLabel || "Motion step").trim() || "Motion step";
  const labels = new Set(state.programWaypoints.map((step) => String(step.label || "").toLowerCase()));
  if (!labels.has(base.toLowerCase())) return base;
  let suffix = 2;
  while (labels.has(`${base} ${suffix}`.toLowerCase())) suffix += 1;
  return `${base} ${suffix}`;
}

function programDetectionEntries() {
  return state.latestDetections
    .map((detection, index) => {
      const id = String(detection.id || detection.detection_id || detection.object_id || `detection-${index + 1}`);
      return { id, detection };
    })
    .filter(({ detection }) => Boolean(detection.ok && detection.robot));
}

function programSourceOptions(source = elements.programStepSource.value) {
  if (source === "named_position") {
    return Object.keys(state.config?.named_positions || {}).sort().map((name) => ({ value: name, label: name }));
  }
  if (source === "vision_detection") {
    return programDetectionEntries().map(({ id, detection }) => ({
      value: id,
      label: `${detection.label || detection.color || "object"} · ${id}`,
    }));
  }
  if (source === "drop_zone") {
    return Object.keys(taskDropZones()).sort().map((name) => ({ value: name, label: name }));
  }
  return [];
}

function programSourceHint(source = elements.programStepSource.value, optionCount = null) {
  const count = optionCount ?? programSourceOptions(source).length;
  if (source === "named_position") return count ? "Adds a saved joint or Cartesian named position." : "No named positions are configured.";
  if (source === "manual_cartesian") return "Starts from the current IK target. Edit X, Y, Z, phi, and move mode after adding.";
  if (source === "manual_joint") return "Starts from the reported joint pose. Edit each joint after adding.";
  if (source === "vision_detection") return count ? "Uses the selected detection's robot-frame coordinates." : "Refresh detections in Tasks before adding a vision target.";
  if (source === "drop_zone") return count ? "Adds a configured Cartesian drop-zone anchor." : "No drop zones are configured.";
  return "";
}

function renderProgramSourceOptions() {
  if (!elements.programStepSource || !state.config) return;
  const source = elements.programStepSource.value;
  const previous = elements.programSourceItem.value;
  const options = programSourceOptions(source);
  elements.programSourceItem.innerHTML = "";
  options.forEach(({ value, label }) => {
    const option = document.createElement("option");
    option.value = value;
    option.textContent = label;
    elements.programSourceItem.appendChild(option);
  });
  if (options.some((option) => option.value === previous)) elements.programSourceItem.value = previous;
  const needsItem = ["named_position", "vision_detection", "drop_zone"].includes(source);
  elements.programSourceItemField.hidden = !needsItem;
  elements.programAddHint.textContent = programSourceHint(source, options.length);
}

function reportedProgramAngles() {
  return (
    normalizeJointAngles(state.robotState?.reported_angles_deg) ||
    normalizeJointAngles(jointControlAngles()) ||
    normalizeJointAngles(state.config?.home_pose) ||
    []
  );
}

function createProgramStep(source, sourceItem = "") {
  const base = {
    id: nextProgramStepId(),
    enabled: true,
    source,
    source_label: sourceItem || source.replaceAll("_", " "),
    branch: elements.ikBranchSelect.value || "auto",
  };
  if (source === "current_pose" || source === "manual_joint") {
    const angles = reportedProgramAngles();
    if (!angles.length) return null;
    return {
      ...base,
      label: uniqueProgramLabel(source === "current_pose" ? "Current pose" : "Joint target"),
      type: "joint",
      mode: "joint",
      angles_deg: angles.slice(),
    };
  }
  if (source === "ik_target" || source === "manual_cartesian") {
    return {
      ...base,
      label: uniqueProgramLabel(source === "ik_target" ? "IK target" : "Cartesian target"),
      type: "cartesian",
      mode: source === "manual_cartesian" ? "linear" : (elements.ikModeSelect.value === "linear" ? "linear" : "joint"),
      target: clonePlain(ikTargetPayload()),
    };
  }
  if (source === "named_position") {
    const waypoint = namedPositionWaypoint(sourceItem);
    if (!waypoint) return null;
    return {
      ...base,
      ...clonePlain(waypoint),
      label: uniqueProgramLabel(sourceItem),
      mode: waypoint.type === "joint" ? "joint" : (waypoint.mode || "joint"),
    };
  }
  if (source === "vision_detection") {
    const entry = programDetectionEntries().find(({ id }) => id === sourceItem);
    const detection = entry?.detection;
    if (!detection?.robot) return null;
    const robot = detection.robot;
    const fallbackZ = Number(state.config?.task_defaults?.pickup_height_mm ?? 0);
    return {
      ...base,
      label: uniqueProgramLabel(`${detection.label || detection.color || "Object"} target`),
      type: "cartesian",
      mode: "linear",
      target: {
        x_mm: Number(robot.x_mm),
        y_mm: Number(robot.y_mm),
        z_mm: Number.isFinite(Number(robot.z_mm)) ? Number(robot.z_mm) : fallbackZ,
        phi_deg: Number.isFinite(Number(robot.phi_deg)) ? Number(robot.phi_deg) : 0,
      },
    };
  }
  if (source === "drop_zone") {
    const zone = taskDropZones()[sourceItem];
    if (!zone) return null;
    return {
      ...base,
      label: uniqueProgramLabel(sourceItem),
      type: "cartesian",
      mode: "joint",
      target: {
        x_mm: Number(zone.x_mm),
        y_mm: Number(zone.y_mm),
        z_mm: Number(zone.z_mm),
        phi_deg: Number(zone.phi_deg ?? 0),
      },
    };
  }
  return null;
}

function clearActiveProgramPreview() {
  if (state.latestPreview?.mode !== "program") return;
  state.previewId = null;
  state.latestPreview = null;
  state.previewAngles = null;
  state.viewPreviewSource = null;
  state.view?.clearPreview();
  elements.targetHud.textContent = "none";
  elements.pathHud.textContent = "0 pts";
  syncJointControls();
}

function markProgramEdited(reason = "Program edited", options = {}) {
  state.programRevision += 1;
  state.programLastEditReason = reason;
  state.programExecutionFailed = false;
  state.programExecutionAwaitingStart = false;
  state.programExecutionError = "";
  clearActiveProgramPreview();
  renderProgramBuilder({ inspector: options.inspector !== false });
}

function insertProgramStep(step) {
  if (!step) {
    showLocalError("The selected program source is not available.");
    return;
  }
  const selectedIndex = selectedProgramIndex();
  const position = elements.programInsertPosition.value;
  let insertIndex = state.programWaypoints.length;
  if (selectedIndex >= 0 && position === "before") insertIndex = selectedIndex;
  if (selectedIndex >= 0 && position === "after") insertIndex = selectedIndex + 1;
  state.programWaypoints.splice(insertIndex, 0, step);
  state.programSelectedId = step.id;
  markProgramEdited(`Added ${step.label}`);
}

function addProgramStep(source = elements.programStepSource.value) {
  const options = programSourceOptions(source);
  const sourceItem = ["named_position", "vision_detection", "drop_zone"].includes(source)
    ? (source === elements.programStepSource.value ? elements.programSourceItem.value : options[0]?.value)
    : "";
  insertProgramStep(createProgramStep(source, sourceItem || ""));
}

function duplicateProgramStep(index) {
  const original = state.programWaypoints[index];
  if (!original) return;
  const duplicate = clonePlain(original);
  duplicate.id = nextProgramStepId();
  duplicate.label = uniqueProgramLabel(`${original.label || `Step ${index + 1}`} copy`);
  state.programWaypoints.splice(index + 1, 0, duplicate);
  state.programSelectedId = duplicate.id;
  markProgramEdited(`Duplicated ${original.label || `step ${index + 1}`}`);
}

function deleteProgramStep(index) {
  const [removed] = state.programWaypoints.splice(index, 1);
  if (!removed) return;
  const next = state.programWaypoints[Math.min(index, state.programWaypoints.length - 1)];
  state.programSelectedId = next?.id || null;
  markProgramEdited(`Deleted ${removed.label || `step ${index + 1}`}`);
}

function moveProgramStep(index, direction) {
  const nextIndex = index + direction;
  if (index < 0 || nextIndex < 0 || nextIndex >= state.programWaypoints.length) return;
  const [step] = state.programWaypoints.splice(index, 1);
  state.programWaypoints.splice(nextIndex, 0, step);
  state.programSelectedId = step.id;
  markProgramEdited(`Reordered ${step.label || "step"}`);
}

function programStepClientValidation(step, index) {
  if (step.enabled === false) return { ok: true, status: "disabled", errors: [] };
  const errors = [];
  if (!String(step.label || "").trim()) errors.push("label is required");
  if (step.type === "joint") {
    const expected = state.config?.joints?.length || 4;
    if (!Array.isArray(step.angles_deg) || step.angles_deg.length !== expected) {
      errors.push(`expected ${expected} joint angles`);
    } else {
      step.angles_deg.forEach((value, jointIndex) => {
        if (!Number.isFinite(Number(value))) {
          errors.push(`J${jointIndex + 1} must be a number`);
          return;
        }
        const joint = state.config?.joints?.[jointIndex];
        if (joint && (Number(value) < joint.min_deg || Number(value) > joint.max_deg)) {
          errors.push(`J${jointIndex + 1} is outside ${format(joint.min_deg, 1)}..${format(joint.max_deg, 1)} deg`);
        }
      });
    }
  } else {
    const target = step.target || {};
    ["x_mm", "y_mm", "z_mm"].forEach((key) => {
      if (!Number.isFinite(Number(target[key]))) errors.push(`${key.replace("_mm", "").toUpperCase()} must be a number`);
    });
    if (!target.phi_auto && !Number.isFinite(Number(target.phi_deg))) errors.push("Phi must be a number or Auto");
    if (!["joint", "linear"].includes(step.mode)) errors.push("move mode must be joint or linear");
  }
  return {
    ok: errors.length === 0,
    status: errors.length ? "invalid" : "unvalidated",
    errors: errors.map((error) => `Step ${index + 1}: ${error}`),
  };
}

function latestProgramTrajectory() {
  if (state.programPreviewFailure) return state.programPreviewFailure.trajectory || {};
  return state.programPreview?.trajectory || {};
}

function programStepResult(index) {
  return (latestProgramTrajectory().step_results || []).find((result) => Number(result.index) === index) || null;
}

function programStepValues(step) {
  if (step.type === "joint") {
    return (step.angles_deg || []).map((value, index) => `J${index + 1} ${format(value, 1)}°`).join(" · ");
  }
  return formatCartesianTarget(step.target || {});
}

function conciseProgramError(error) {
  const message = String(error || "").trim();
  const stepPrefix = message.match(/^Step \d+:/i)?.[0] || "";
  const prefix = stepPrefix ? `${stepPrefix} ` : "";
  if (/no valid position\/orientation IK solution/i.test(message)) {
    return `${prefix}No valid IK solution for this path point. Adjust the target position, orientation, or move mode.`;
  }
  if (/no valid IK solution/i.test(message)) {
    return `${prefix}No valid IK solution at the requested target. Adjust the position, orientation, or IK branch.`;
  }
  if (/outside .* reach/i.test(message)) {
    return `${prefix}Target is outside the configured robot reach.`;
  }
  return message.length > 220 ? `${message.slice(0, 217)}…` : message;
}

function renderProgramList() {
  elements.programList.innerHTML = "";
  const stepCount = state.programWaypoints.length;
  elements.programStepCount.textContent = `(${stepCount} step${stepCount === 1 ? "" : "s"})`;
  const trajectory = latestProgramTrajectory();
  const stale = state.programValidationRevision !== null && state.programValidationRevision !== state.programRevision;
  elements.programEstimatedDuration.textContent = Number.isFinite(Number(trajectory.duration_s))
    ? `${stale ? "Stale · " : ""}${format(trajectory.duration_s, 2)} s`
    : "No estimate";
  if (!stepCount) {
    elements.programList.innerHTML = `
      <div class="program-empty-state">
        <strong>Build the first move</strong>
        <p>Add the reported robot pose or the current IK target, then select the step to edit its values and move mode.</p>
        <div class="program-empty-actions">
          <button type="button" data-program-quick-source="current_pose">Add current pose</button>
          <button type="button" data-program-quick-source="ik_target">Add IK target</button>
        </div>
      </div>
    `;
    return;
  }

  state.programWaypoints.forEach((step, index) => {
    const client = programStepClientValidation(step, index);
    const result = programStepResult(index);
    const currentValidation = state.programValidationRevision === state.programRevision;
    const status = step.enabled === false
      ? "disabled"
      : !client.ok
        ? "invalid"
        : currentValidation
          ? result?.status || "unvalidated"
          : stale && result
            ? "stale"
            : "unvalidated";
    const errors = !client.ok ? client.errors : (currentValidation ? result?.errors || [] : []);
    const duration = result && Number.isFinite(Number(result.duration_s)) ? `${format(result.duration_s, 2)} s` : "—";
    const item = document.createElement("div");
    item.className = `program-step-row ${state.programSelectedId === step.id ? "selected" : ""} ${status}`;
    item.dataset.programSelect = String(index);
    item.innerHTML = `
      <div class="program-step-main">
        <label class="program-step-enable" title="${step.enabled === false ? "Enable step" : "Disable step"}">
          <input type="checkbox" data-program-action="toggle" data-program-index="${index}" ${step.enabled === false ? "" : "checked"} ${state.programExecutionActive ? "disabled" : ""} />
          <span>${index + 1}</span>
        </label>
        <div class="program-step-copy">
          <div class="program-step-heading">
            <strong>${escapeHtml(step.label || `Step ${index + 1}`)}</strong>
            <span class="program-validation ${status}">${status === "valid" ? "Valid" : status === "invalid" ? "Invalid" : status === "disabled" ? "Disabled" : status === "stale" ? "Stale" : "Not previewed"}</span>
          </div>
          <div class="program-step-meta">
            <span>${step.type === "joint" ? "Joint target" : "Cartesian target"}</span>
            <span class="program-mode ${step.mode === "linear" ? "linear" : "joint"}">${step.mode === "linear" ? "Linear TCP" : "Joint move"}</span>
            <span>${duration}</span>
          </div>
          <code>${escapeHtml(programStepValues(step))}</code>
          ${errors.length ? `<div class="program-step-error" title="${escapeHtml(errors[0])}">${escapeHtml(conciseProgramError(errors[0]))}</div>` : ""}
        </div>
      </div>
      <div class="program-step-actions">
        <button type="button" class="ghost" data-program-action="up" data-program-index="${index}" ${index === 0 || state.programExecutionActive ? "disabled" : ""} aria-label="Move step up">↑</button>
        <button type="button" class="ghost" data-program-action="down" data-program-index="${index}" ${index === stepCount - 1 || state.programExecutionActive ? "disabled" : ""} aria-label="Move step down">↓</button>
        <button type="button" class="ghost" data-program-action="duplicate" data-program-index="${index}" ${state.programExecutionActive ? "disabled" : ""}>Duplicate</button>
        <button type="button" class="ghost danger-text" data-program-action="delete" data-program-index="${index}" ${state.programExecutionActive ? "disabled" : ""}>Delete</button>
      </div>
    `;
    elements.programList.appendChild(item);
  });
}

function renderProgramInspector() {
  const step = selectedProgramStep();
  const index = selectedProgramIndex();
  elements.programInspectorSection.hidden = !step;
  if (!step) {
    elements.programInspector.innerHTML = "";
    return;
  }
  elements.programInspectorTitle.textContent = `Selected step ${index + 1} of ${state.programWaypoints.length}`;
  const disabled = state.programExecutionActive ? "disabled" : "";
  const valueFields = step.type === "joint"
    ? `<div class="program-value-grid">
        ${(step.angles_deg || []).map((value, jointIndex) => `
          <label>J${jointIndex + 1} <span class="input-with-unit"><input type="number" step="0.1" value="${Number.isFinite(Number(value)) ? value : ""}" data-program-field="angle" data-program-value-index="${jointIndex}" ${disabled} /><span>deg</span></span></label>
        `).join("")}
      </div>`
    : `<div class="program-value-grid">
        ${[
          ["x_mm", "X", "mm"],
          ["y_mm", "Y", "mm"],
          ["z_mm", "Z", "mm"],
          ["phi_deg", "Phi", "deg"],
        ].map(([key, label, unit]) => `
          <label>${label} <span class="input-with-unit"><input type="number" step="0.1" value="${Number.isFinite(Number(step.target?.[key])) ? step.target[key] : ""}" data-program-field="target" data-program-target-key="${key}" ${key === "phi_deg" && step.target?.phi_auto ? "disabled" : disabled} /><span>${unit}</span></span></label>
        `).join("")}
      </div>
      <label class="toggle-label compact-toggle program-auto-phi">
        <input type="checkbox" data-program-field="phi_auto" ${step.target?.phi_auto ? "checked" : ""} ${disabled} />
        <span>Choose phi automatically during IK</span>
      </label>`;

  elements.programInspector.innerHTML = `
    <div class="program-inspector-grid">
      <label>Label <input type="text" value="${escapeHtml(step.label || "")}" data-program-field="label" ${disabled} /></label>
      <label>Move mode
        <select data-program-field="mode" ${step.type === "joint" || state.programExecutionActive ? "disabled" : ""}>
          <option value="joint" ${step.mode === "joint" ? "selected" : ""}>Joint move</option>
          <option value="linear" ${step.mode === "linear" ? "selected" : ""}>Linear Cartesian move</option>
        </select>
      </label>
    </div>
    <div class="program-inspector-type"><span>Type</span><strong>${step.type === "joint" ? "Joint target" : "Cartesian target"}</strong></div>
    ${valueFields}
    <details class="program-advanced">
      <summary>Advanced</summary>
      <div class="program-inspector-grid">
        <label>IK branch
          <select data-program-field="branch" ${step.type === "joint" || state.programExecutionActive ? "disabled" : ""}>
            <option value="auto" ${step.branch === "auto" ? "selected" : ""}>Auto nearest</option>
            <option value="elbow_up" ${step.branch === "elbow_up" ? "selected" : ""}>Elbow up</option>
            <option value="elbow_down" ${step.branch === "elbow_down" ? "selected" : ""}>Elbow down</option>
          </select>
        </label>
        <div class="program-source-readout"><span>Source</span><strong>${escapeHtml(step.source_label || step.source || "manual")}</strong></div>
      </div>
      <p class="field-help">Per-step speed and tool actions are not part of the current motion-only program API. The step shape keeps source and settings fields available for later extension.</p>
    </details>
    <div class="program-inspector-actions">
      <button type="button" class="ghost" data-program-inspector-action="duplicate" ${disabled}>Duplicate step</button>
      <button type="button" class="ghost danger-text" data-program-inspector-action="delete" ${disabled}>Delete step</button>
    </div>
  `;
}

function programWorkflowState() {
  if (state.programExecutionActive) return "running";
  if (!state.programWaypoints.length) return "empty";
  if (
    state.programExecutionFailed ||
    (state.programPreviewFailure && state.programValidationRevision === state.programRevision)
  ) return "failed";
  if (programPreviewIsFresh()) return "preview_valid";
  if (state.programHasPreviewed || state.programValidationRevision !== null) return "needs_preview";
  return "editing";
}

function renderProgramWorkflowStatus() {
  const workflowState = programWorkflowState();
  const labels = {
    empty: "Empty",
    editing: "Editing",
    needs_preview: "Needs preview",
    preview_valid: "Preview valid",
    running: "Running",
    failed: "Failed",
  };
  elements.programStatus.textContent = labels[workflowState];
  elements.programStatus.dataset.state = workflowState;
  const diagnostics = state.robotState?.motion_diagnostics || {};
  const details = {
    empty: "Add the first motion step to begin.",
    editing: "Build the sequence, select each step to edit it, then run Preview.",
    needs_preview: `${state.programLastEditReason || "The sequence changed"}. Preview the current version before execution.`,
    preview_valid: "This exact sequence passed preview and is ready when the robot safety gates are satisfied.",
    running: `Executing ${diagnostics.current_waypoint_total ? `waypoint ${diagnostics.current_waypoint_index || 0} of ${diagnostics.current_waypoint_total}` : "the validated program"}.`,
    failed: state.programExecutionError || state.programPreviewFailure?.error || "Preview or execution failed. Inspect the affected step below.",
  };
  elements.programStatusDetail.textContent = details[workflowState];

  const activeStage = workflowState === "running" || state.programExecutionFailed
    ? "execute"
    : ["preview_valid", "failed"].includes(workflowState)
      ? "preview"
      : "build";
  const completed = new Set();
  if (["preview_valid", "running"].includes(workflowState)) completed.add("build");
  if (workflowState === "running") completed.add("preview");
  elements.programWorkflow.querySelectorAll("[data-program-stage]").forEach((stage) => {
    stage.classList.toggle("active", stage.dataset.programStage === activeStage);
    stage.classList.toggle("done", completed.has(stage.dataset.programStage));
  });
}

function currentProgramErrors() {
  const errors = [];
  state.programWaypoints.forEach((step, index) => {
    errors.push(...programStepClientValidation(step, index).errors);
  });
  if (state.programValidationRevision === state.programRevision && state.programPreviewFailure) {
    const stepErrors = (state.programPreviewFailure.trajectory?.step_results || [])
      .filter((result) => result.status === "invalid")
      .flatMap((result) => (result.errors || []).map((error) => `Step ${Number(result.index) + 1}: ${error}`));
    if (stepErrors.length) errors.push(...stepErrors.map(conciseProgramError));
    else if (state.programPreviewFailure.error) errors.push(conciseProgramError(state.programPreviewFailure.error));
  }
  return [...new Set(errors)];
}

function renderProgramPreviewSummary() {
  const trajectory = latestProgramTrajectory();
  const stepCount = state.programWaypoints.length;
  const moveCount = state.programWaypoints.filter((step) => step.enabled !== false).length;
  const fresh = programPreviewIsFresh();
  const stale = state.programValidationRevision !== null && state.programValidationRevision !== state.programRevision;
  const status = state.programExecutionActive
    ? "Running"
    : fresh
      ? "Valid"
      : state.programPreviewPending
      ? "Checking"
      : state.programPreviewFailure && state.programValidationRevision === state.programRevision
        ? "Failed"
        : stale
          ? "Stale"
          : "Not previewed";
  const errors = currentProgramErrors();
  const calibrationWarnings = Array.isArray(state.programPreview?.calibration)
    ? state.programPreview.calibration.flatMap((item) => item.warnings || [])
    : state.programPreview?.calibration?.warnings || [];
  elements.programPreviewSummary.innerHTML = `
    <div class="program-summary-grid">
      <div><span>Steps</span><strong>${stepCount}</strong></div>
      <div><span>Moves</span><strong>${moveCount}</strong></div>
      <div><span>Duration</span><strong>${Number.isFinite(Number(trajectory.duration_s)) ? `${format(trajectory.duration_s, 2)} s` : "—"}</strong></div>
      <div><span>Preview</span><strong class="program-summary-status ${status.toLowerCase().replace(" ", "-")}">${status}</strong></div>
    </div>
    <div class="program-summary-messages">
      ${stale ? `<div class="program-summary-message warning"><strong>Preview is stale</strong><span>${escapeHtml(state.programLastEditReason || "The sequence changed after preview.")}</span></div>` : ""}
      ${errors.slice(0, 5).map((error) => `<div class="program-summary-message error"><strong>Needs attention</strong><span>${escapeHtml(error)}</span></div>`).join("")}
      ${calibrationWarnings.slice(0, 3).map((warning) => `<div class="program-summary-message warning"><strong>Warning</strong><span>${escapeHtml(warning)}</span></div>`).join("")}
      ${fresh && !errors.length ? `<div class="program-summary-message success"><strong>Preview matches</strong><span>${trajectory.waypoint_count || 0} planned path points are ready for execution.</span></div>` : ""}
      ${!stepCount ? `<div class="program-summary-message neutral"><strong>No sequence yet</strong><span>Add a motion step above. Editing never sends a robot command.</span></div>` : ""}
    </div>
  `;
  const gateReason = programMotionGateReason();
  if (state.programExecutionActive) {
    elements.programExecuteHint.textContent = "Program execution is in progress. Editing is locked until it finishes or is stopped.";
  } else if (fresh && gateReason) {
    elements.programExecuteHint.textContent = `Preview is valid, but execution is blocked: ${gateReason}`;
  } else if (fresh) {
    elements.programExecuteHint.textContent = "Ready to execute the exact previewed sequence.";
  } else {
    elements.programExecuteHint.textContent = "Run a fresh preview with no errors before execution.";
  }
}

function renderProgramBuilder(options = {}) {
  renderProgramSourceOptions();
  renderProgramList();
  if (options.inspector !== false) renderProgramInspector();
  renderProgramWorkflowStatus();
  renderProgramPreviewSummary();
  updateDisabledState();
}

function updateProgramStepFromControl(control) {
  const step = selectedProgramStep();
  if (!step || !control?.dataset?.programField) return;
  const field = control.dataset.programField;
  if (field === "label") step.label = control.value;
  if (field === "mode") step.mode = control.value === "linear" && step.type === "cartesian" ? "linear" : "joint";
  if (field === "branch") step.branch = control.value;
  if (field === "phi_auto") {
    step.target = step.target || {};
    step.target.phi_auto = control.checked;
  }
  if (field === "target") {
    step.target = step.target || {};
    step.target[control.dataset.programTargetKey] = control.value === "" ? Number.NaN : Number(control.value);
  }
  if (field === "angle") {
    const valueIndex = Number(control.dataset.programValueIndex);
    step.angles_deg[valueIndex] = control.value === "" ? Number.NaN : Number(control.value);
  }
  markProgramEdited(`Edited ${step.label || "selected step"}`, { inspector: false });
}

async function previewProgram() {
  const enabledSteps = state.programWaypoints.filter((step) => step.enabled !== false);
  if (!enabledSteps.length || state.programPreviewPending) {
    renderProgramBuilder();
    return;
  }
  const revision = state.programRevision;
  const clientErrors = currentProgramErrors();
  state.programHasPreviewed = true;
  if (clientErrors.length) {
    state.programPreviewRevision = null;
    state.programValidationRevision = revision;
    state.programPreviewFailure = {
      ok: false,
      error: clientErrors[0],
      trajectory: {
        step_count: state.programWaypoints.length,
        move_count: enabledSteps.length,
        step_results: state.programWaypoints.map((step, index) => {
          const validation = programStepClientValidation(step, index);
          return {
            index,
            label: step.label,
            type: step.type,
            mode: step.mode,
            enabled: step.enabled !== false,
            status: validation.status,
            duration_s: 0,
            waypoint_count: 0,
            errors: validation.errors.map((error) => error.replace(/^Step \d+:\s*/, "")),
          };
        }),
      },
    };
    clearActiveProgramPreview();
    renderProgramBuilder();
    return;
  }

  state.programPreviewPending = true;
  state.programPreviewFailure = null;
  renderProgramBuilder({ inspector: false });
  const payload = await postJson("/api/path/preview", {
    mode: "program",
    branch: elements.ikBranchSelect.value,
    settings: pathSettings(),
    waypoints: clonePlain(state.programWaypoints),
    program_revision: revision,
  });
  state.programPreviewPending = false;
  if (revision !== state.programRevision) {
    renderProgramBuilder();
    return;
  }
  state.programValidationRevision = revision;
  if (payload.ok) {
    renderPreview(payload.preview);
    state.programPreview = payload.preview;
    state.programPreviewRevision = revision;
    state.programPreviewFailure = null;
    state.programExecutionFailed = false;
  } else {
    state.programPreviewRevision = null;
    state.programPreviewFailure = payload;
    renderPreviewFailure(payload);
  }
  renderProgramBuilder();
}

async function executeProgram() {
  if (!programPreviewIsFresh()) {
    showLocalError("Preview the current program successfully before execution.");
    return;
  }
  const gateReason = programMotionGateReason();
  if (gateReason) {
    showLocalError(gateReason);
    renderProgramBuilder({ inspector: false });
    return;
  }
  const previewId = state.previewId;
  state.programExecutionActive = true;
  state.programExecutionAwaitingStart = true;
  state.programExecutionFailed = false;
  state.programExecutionError = "";
  renderProgramBuilder({ inspector: false });
  const payload = await postJson("/api/path/execute", {
    preview_id: previewId,
    program_revision: state.programRevision,
  });
  if (payload.ok) {
    releaseJointControlIntent();
    state.previewId = null;
    state.previewAngles = null;
    state.taskPreviewId = null;
    state.ikUserEdited = false;
    state.programPreviewRevision = null;
    state.programLastEditReason = "The previous preview was consumed by execution";
  } else {
    state.programExecutionActive = false;
    state.programExecutionAwaitingStart = false;
    state.programExecutionFailed = true;
    state.programExecutionError = payload.error || "Program execution failed.";
    if (/preview|configuration|model|start pose/i.test(payload.error || "")) {
      clearViewPreview();
      state.programPreviewRevision = null;
      state.programLastEditReason = "The robot pose or model changed after preview";
    }
  }
  if (payload.state) renderState(payload.state);
  renderProgramBuilder({ inspector: false });
}

function syncProgramExecutionState(robotState) {
  if (!state.programExecutionActive) return;
  const diagnostics = robotState?.motion_diagnostics || {};
  const result = String(diagnostics.result || robotState?.motion_execution_state || "").toLowerCase();
  if (state.programExecutionAwaitingStart) {
    if (["queued", "executing", "uploading", "accepted", "command_sent"].includes(result)) {
      state.programExecutionAwaitingStart = false;
    } else {
      return;
    }
  }
  if (result === "failed" || result === "stopped") {
    state.programExecutionActive = false;
    state.programExecutionAwaitingStart = false;
    state.programExecutionFailed = true;
    state.programExecutionError = diagnostics.error || robotState?.last_error || `Program ${result}.`;
  } else if (result === "reached") {
    state.programExecutionActive = false;
    state.programExecutionAwaitingStart = false;
    state.programExecutionFailed = false;
    state.programExecutionError = "";
    state.programLastEditReason = "Execution finished";
  }
}

function readHardwarePatch(index, actuator) {
  const patch = {};
  document.querySelectorAll(`[data-hw-index="${index}"][data-hw-kind="${actuator}"][data-hw-field]`).forEach((input) => {
    const field = input.dataset.hwField;
    if (input.type === "checkbox") {
      patch[field] = input.checked;
    } else if (field === "driver_model") {
      patch[field] = input.value || "TBD";
    } else {
      patch[field] = readNumber(input, 0);
    }
  });
  return { [actuator]: patch };
}

function readCalibrationPayload() {
  const links = readLinkPayload();
  const jointValidation = validateJointCalibrationDraft();
  if (!jointValidation.ok) return null;
  const joints = state.config.joints.map((joint, index) => {
    const minInput = $(`[data-joint-index="${index}"][data-calib-limit="min"]`);
    const maxInput = $(`[data-joint-index="${index}"][data-calib-limit="max"]`);
    const patch = {
      limits_deg: {
        min: readNumber(minInput, joint.min_deg),
        max: readNumber(maxInput, joint.max_deg),
      },
    };
    document.querySelectorAll(`[data-joint-index="${index}"][data-calib-field]`).forEach((input) => {
      const field = input.dataset.calibField;
      patch[field] = field === "direction_sign" ? Number(input.value) : readNumber(input, joint[field]);
    });
    patch.hardware = joint.actuator === "servo" ? readHardwarePatch(index, "servo") : readHardwarePatch(index, "stepper");
    patch.max_speed_deg_s = readNumber($(`.joint-speed-limit[data-index="${index}"]`), joint.max_speed_deg_s);
    patch.max_accel_deg_s2 = readNumber($(`.joint-accel-limit[data-index="${index}"]`), joint.max_accel_deg_s2);
    return patch;
  });
  const dhValidation = validateDhDraft();
  if (!dhValidation.ok) return null;
  return {
    links_mm: links,
    kinematics: {
      ...(state.config.kinematics || {}),
      dh_rows: dhValidation.rows,
    },
    geometry: readGeometryPayload(),
    tools: readToolsPayload(),
    joints,
    color_profiles: taskColorProfiles(),
    drop_zones: taskDropZones(),
    tasks: {
      ...(state.config.tasks || {}),
      color_sorting: {
        ...(state.config.tasks?.color_sorting || {}),
        orientation_policy: elements.orientationPolicySelect?.value || state.config.tasks?.color_sorting?.orientation_policy || "prefer_downward",
      },
    },
    path_defaults: pathSettings(),
    motion: {
      command_rate_limit_hz: readNumber(elements.waypointRateInput, state.config.motion.command_rate_limit_hz),
      acceleration_deg_s2: readNumber(elements.globalAccelInput, state.config.motion.acceleration_deg_s2),
    },
  };
}

async function saveCalibration(options = {}) {
  const showStatus = options.showStatus !== false;
  if (showStatus) updateSettingsSaveBar({ mode: "saving", title: "Saving robot settings", detail: "Validating and writing the current robot configuration…" });
  const calibrationPayload = readCalibrationPayload();
  if (!calibrationPayload) {
    if (showStatus) {
      updateSettingsSaveBar({
        mode: "error",
        title: "Settings could not be saved",
        detail: "Fix the highlighted joint calibration or derived-model values, then try again.",
      });
    }
    return { ok: false, error: "invalid settings draft" };
  }
  const payload = await postJson("/api/config/calibration", calibrationPayload);
  if (payload.ok) {
    state.hardwareDraftDirty = false;
    applyConfig(payload.config);
    if (payload.state) renderState(payload.state);
    if (options.clearPreview !== false) clearViewPreview();
    robotSettingsScopes.forEach((scope) => state.settingsDirtyScopes.delete(scope));
    if (showStatus) updateSettingsSaveBar();
  } else {
    updateSettingsSaveBar({
      mode: "error",
      title: "Settings could not be saved",
      detail: payload.error || "The robot configuration save failed.",
    });
  }
  return payload;
}

async function saveAllSettings() {
  if (!state.settingsDirtyScopes.size) return true;
  let cameraDraft = null;
  if (state.settingsDirtyScopes.has("camera")) {
    try {
      cameraDraft = cameraSettingsDraft();
    } catch (error) {
      updateSettingsSaveBar({
        mode: "error",
        title: "Camera settings are invalid",
        detail: error?.message || String(error),
      });
      return false;
    }
  }
  updateSettingsSaveBar({
    mode: "saving",
    title: "Saving all settings",
    detail: "Writing the changed robot and camera settings…",
  });
  const hasRobotDraft = [...state.settingsDirtyScopes].some((scope) => robotSettingsScopes.has(scope));
  if (hasRobotDraft) {
    const robotPayload = await saveCalibration({ showStatus: false });
    if (!robotPayload.ok) {
      updateSettingsSaveBar({
        mode: "error",
        title: "Settings could not be saved",
        detail: robotPayload.error || "The robot settings save failed.",
      });
      return false;
    }
  }
  if (cameraDraft) {
    const cameraPayload = await postJson("/api/vision/settings", { camera: cameraDraft });
    if (!cameraPayload.ok) {
      updateSettingsSaveBar({
        mode: "error",
        title: "Camera settings could not be saved",
        detail: cameraPayload.error || "The camera settings save failed.",
      });
      return false;
    }
    if (cameraPayload.config) applyConfig(cameraPayload.config);
  }
  clearSettingsDirty();
  return true;
}

async function discardSettingsChanges() {
  updateSettingsSaveBar({
    mode: "saving",
    title: "Discarding draft changes",
    detail: "Reloading the last saved configuration…",
  });
  await loadConfig();
  state.hardwareDraftDirty = false;
  clearSettingsDirty();
}

function applyConfig(config) {
  const previousConfigId = state.config?.app_version?.running_config_id;
  const nextConfigId = config?.app_version?.running_config_id;
  const replacingConfig = Boolean(state.config && previousConfigId !== nextConfigId);
  state.config = config;
  state.linkDraft = null;
  state.dhDraftRows = null;
  state.taskColorProfilesDraft = clonePlain(config.color_profiles || {});
  state.taskDropZonesDraft = clonePlain(config.drop_zones || {});
  state.unsavedColorProfiles.clear();
  state.selectedSerialPort = state.selectedSerialPort || state.config.serial.port;
  elements.baudRate.value = state.config.serial.baud_rate;
  elements.commandRate.textContent = `${format(state.config.motion.command_rate_limit_hz, 0)} Hz command limit`;
  const pathDefaults = state.config.path_defaults || {};
  elements.globalSpeedInput.value = format(
    pathDefaults.global_speed_deg_s ?? Math.min(...state.config.joints.map((joint) => joint.max_speed_deg_s)),
    1
  );
  elements.globalAccelInput.value = format(pathDefaults.global_accel_deg_s2 ?? state.config.motion.acceleration_deg_s2, 1);
  elements.waypointRateInput.value = format(pathDefaults.waypoint_rate_hz ?? state.config.motion.command_rate_limit_hz, 0);
  elements.cartesianStepInput.value = format(pathDefaults.cartesian_step_mm ?? 10, 0);
  elements.plannerTypeSelect.value = pathDefaults.planner_type || "s_curve";
  elements.jerkPercentInput.value = format(pathDefaults.jerk_percent ?? 25, 0);
  elements.blendPercentInput.value = format(pathDefaults.blend_percent ?? 0, 0);
  buildJointControls();
  buildPerJointTuning();
  buildCalibrationEditors();
  buildHardwareIoEditors();
  renderToolSelectOptions();
  if (state.robotState) renderHardwareStatus(state.robotState);
  renderOperatorPanels();
  buildIkTargetControls();
  buildSliderRanges();
  updatePhiControlState();
  renderProgramBuilder();
  state.view.setConfig(state.config);
  syncJointControls();
  renderCameraIntrinsics(state.config.camera);
  if (!state.config.camera?.enabled) {
    if (state.cameraLive) setCameraLive(false);
    window.clearTimeout(state.workspaceProjectionTimer);
    state.workspaceProjectionTimer = null;
    state.view?.setWorkspaceCameraProjection(null, false);
  } else {
    if (state.cameraLive) scheduleCameraFrame(0);
    scheduleWorkspaceProjection(0);
  }
  updateSettingsSaveBar();
  if (replacingConfig) invalidateTaskDetections("Robot configuration changed - refresh detections");
  if (replacingConfig && state.programWaypoints.length && !state.programExecutionActive) {
    markProgramEdited("Robot configuration changed", { inspector: false });
  }
}

function clearViewPreview() {
  state.previewId = null;
  state.latestPreview = null;
  state.previewAngles = null;
  state.previewBasePoseRevision = null;
  state.intentBasePoseRevision = null;
  state.taskPreviewId = null;
  state.viewPreviewSource = null;
  if (state.view) state.view.clearPreview();
  state.view?.setTaskPreview?.(null);
  elements.targetHud.textContent = "none";
  elements.pathHud.textContent = "0 pts";
  syncJointControls();
  updateDisabledState();
}

async function loadConfig() {
  const response = await fetch("/api/config");
  applyConfig(await response.json());
}

function connectWebSocket() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  state.ws = new WebSocket(`${protocol}://${window.location.host}/ws`);
  state.ws.addEventListener("message", (event) => {
    const payload = JSON.parse(event.data);
    if (payload.type === "config") applyConfig(payload.config);
    if (payload.type === "state") {
      if (state.pendingAngles && anglesAlmostEqual(payload.state.target_angles_deg, state.pendingAngles)) {
        state.pendingAngles = null;
      }
      renderState(payload.state);
    }
  });
  state.ws.addEventListener("close", () => {
    window.setTimeout(connectWebSocket, 800);
  });
}

function bindFader(element, onDelta, onStop = null, onStart = null) {
  let dragging = false;
  let offset = 0;
  let pointerId = null;
  let raf = null;
  let lastTime = null;
  const speed = Number(element.dataset.faderSpeed || 1);

  const updateOffset = (clientX) => {
    const rect = element.getBoundingClientRect();
    const maxOffset = Math.max(8, rect.width / 2 - 10);
    offset = clamp((clientX - (rect.left + rect.width / 2)) / maxOffset, -1, 1);
    element.style.setProperty("--fader-offset", `${offset * maxOffset}px`);
  };
  const reset = () => {
    offset = 0;
    element.style.setProperty("--fader-offset", "0px");
  };
  const stop = () => {
    dragging = false;
    element.classList.remove("dragging");
    reset();
    if (raf) cancelAnimationFrame(raf);
    raf = null;
    lastTime = null;
  };
  const loop = (time) => {
    if (!dragging) return;
    if (lastTime === null) lastTime = time;
    const dt = Math.min(0.05, (time - lastTime) / 1000);
    lastTime = time;
    const velocity = Math.abs(offset) > 0.001 ? speed * offset * Math.abs(offset) : 0;
    onDelta(velocity * dt, { velocity, dt, offset });
    raf = requestAnimationFrame(loop);
  };

  element.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    event.preventDefault();
    dragging = true;
    pointerId = event.pointerId;
    element.classList.add("dragging");
    element.setPointerCapture(pointerId);
    updateOffset(event.clientX);
    if (onStart) onStart();
    raf = requestAnimationFrame(loop);
  });
  element.addEventListener("pointermove", (event) => {
    if (dragging) updateOffset(event.clientX);
  });
  ["pointerup", "pointercancel", "pointerleave"].forEach((name) => {
    element.addEventListener(name, () => {
      if (pointerId !== null) {
        try {
          element.releasePointerCapture(pointerId);
        } catch {
          // Pointer capture may already be released by the browser.
        }
      }
      pointerId = null;
      stop();
      if (onStop) onStop();
    });
  });
}

function bindFaders() {
  document.querySelectorAll(".target-fader").forEach((fader) => {
    bindFader(fader, (delta, meta) => {
      const key = fader.dataset.faderKey;
      if (key === "phi" && ikAutoPhiEnabled()) return;
      if (cartesianJogEnabled()) {
        scheduleCartesianJog(key, meta.velocity);
        return;
      }
      if (Math.abs(delta) <= 1e-9) return;
      const input = $(`#ik-${key}-input`);
      if (!input) return;
      setIkTargetValue(key, readNumber(input, 0) + delta);
      scheduleIkPreview();
    }, () => {
      const key = fader.dataset.faderKey;
      if (!cartesianJogEnabled()) return;
      axisVelocityPayload(key, 0);
      state.cartesianJogActiveAxes.delete(key);
      if (state.cartesianJogActiveAxes.size === 0) stopCartesianJog();
    }, () => {
      if (cartesianJogEnabled()) setCartesianJogStatus("starting");
    });
  });
}

function bindPanelChrome() {
  let resizing = false;
  let resizePointerId = null;
  let startX = 0;
  let startWidth = 0;
  const finishResize = () => {
    if (!resizing) return;
    resizing = false;
    resizePointerId = null;
    elements.appLayout.classList.remove("panel-resizing");
    elements.panelResizer.setAttribute("aria-valuenow", String(Math.round(document.querySelector(".left-panel")?.getBoundingClientRect().width || 0)));
    state.view.resize();
  };
  const resizePanel = (event) => {
    if (!resizing || (resizePointerId != null && event.pointerId !== resizePointerId)) return;
    const minimumWidth = window.innerWidth <= 760 ? 320 : 420;
    const viewportReserve = window.innerWidth <= 1100 ? 260 : 340;
    const maximumWidth = Math.max(minimumWidth, window.innerWidth - viewportReserve);
    const width = clamp(startWidth + event.clientX - startX, minimumWidth, maximumWidth);
    if (elements.appLayout.classList.contains("settings-active")) {
      elements.appLayout.style.setProperty("--settings-panel-width", `${width}px`);
    } else {
      document.documentElement.style.setProperty("--left-panel-width", `${width}px`);
    }
    elements.panelResizer.setAttribute("aria-valuenow", String(Math.round(width)));
    state.view.resize();
  };
  elements.panelResizer.setAttribute("aria-label", "Resize control panel");
  elements.panelResizer.setAttribute("aria-valuemin", "320");
  elements.panelResizer.setAttribute("aria-valuenow", "500");
  elements.panelResizer.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    resizing = true;
    resizePointerId = event.pointerId;
    startX = event.clientX;
    startWidth = document.querySelector(".left-panel")?.getBoundingClientRect().width || 500;
    elements.appLayout.classList.add("panel-resizing");
    elements.panelResizer.setPointerCapture(event.pointerId);
  });
  window.addEventListener("pointermove", resizePanel);
  elements.panelResizer.addEventListener("pointerup", (event) => {
    if (elements.panelResizer.hasPointerCapture(event.pointerId)) {
      elements.panelResizer.releasePointerCapture(event.pointerId);
    }
    finishResize();
  });
  elements.panelResizer.addEventListener("pointercancel", finishResize);
  elements.panelResizer.addEventListener("lostpointercapture", finishResize);
  elements.collapsePanelBtn.addEventListener("click", () => {
    elements.appLayout.classList.toggle("collapsed");
    window.setTimeout(() => state.view.resize(), 300);
  });
  document.querySelectorAll("[data-collapse-widget]").forEach((button) => {
    const widget = $(`#${button.dataset.collapseWidget}`);
    const widgetName = button.dataset.collapseWidget === "sceneHud" ? "HUD" : "faders";
    const updateCollapseButton = () => {
      const expanded = !widget.classList.contains("collapsed");
      button.setAttribute("aria-controls", widget.id);
      button.setAttribute("aria-expanded", String(expanded));
      button.setAttribute("aria-label", `${expanded ? "Collapse" : "Expand"} ${widgetName}`);
      button.title = `${expanded ? "Collapse" : "Expand"} ${widgetName}`;
    };
    updateCollapseButton();
    button.addEventListener("click", () => {
      widget.classList.toggle("collapsed");
      updateCollapseButton();
    });
  });
  const updateTabHeaderHeight = () => {
    const height = Math.ceil(elements.tabHeader?.getBoundingClientRect().height || 146);
    document.documentElement.style.setProperty("--tab-header-height", `${height}px`);
  };
  updateTabHeaderHeight();
  if (window.ResizeObserver && elements.tabHeader) {
    const observer = new ResizeObserver(updateTabHeaderHeight);
    observer.observe(elements.tabHeader);
  }
}

function bindCameraPopup() {
  if (!elements.cameraPopup || !elements.cameraPopupHandle || !elements.mainPanel) return;
  let dragging = false;
  let pointerId = null;
  let startX = 0;
  let startY = 0;
  let startLeft = 0;
  let startTop = 0;

  const move = (event) => {
    if (!dragging || event.pointerId !== pointerId) return;
    const panelRect = elements.mainPanel.getBoundingClientRect();
    const popupRect = elements.cameraPopup.getBoundingClientRect();
    const maximumLeft = Math.max(0, panelRect.width - popupRect.width);
    const maximumTop = Math.max(0, panelRect.height - popupRect.height);
    const left = clamp(startLeft + event.clientX - startX, 0, maximumLeft);
    const top = clamp(startTop + event.clientY - startY, 0, maximumTop);
    elements.cameraPopup.style.left = `${left}px`;
    elements.cameraPopup.style.top = `${top}px`;
    elements.cameraPopup.style.right = "auto";
  };
  const finish = (event) => {
    if (!dragging || (event?.pointerId != null && event.pointerId !== pointerId)) return;
    dragging = false;
    if (pointerId != null && elements.cameraPopupHandle.hasPointerCapture(pointerId)) {
      elements.cameraPopupHandle.releasePointerCapture(pointerId);
    }
    pointerId = null;
  };
  elements.cameraPopupHandle.addEventListener("pointerdown", (event) => {
    if (event.target.closest("button, input, label")) return;
    const panelRect = elements.mainPanel.getBoundingClientRect();
    const popupRect = elements.cameraPopup.getBoundingClientRect();
    dragging = true;
    pointerId = event.pointerId;
    startX = event.clientX;
    startY = event.clientY;
    startLeft = popupRect.left - panelRect.left;
    startTop = popupRect.top - panelRect.top;
    elements.cameraPopupHandle.setPointerCapture(pointerId);
  });
  elements.cameraPopupHandle.addEventListener("pointermove", move);
  elements.cameraPopupHandle.addEventListener("pointerup", finish);
  elements.cameraPopupHandle.addEventListener("pointercancel", finish);
  elements.cameraPopupHandle.addEventListener("lostpointercapture", finish);
}

function bindActions() {
  elements.appTabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      const target = tab.dataset.appTab;
      state.activeTab = target;
      elements.appLayout.classList.toggle("settings-active", target === "settings");
      elements.appTabs.forEach((candidate) => candidate.classList.toggle("active", candidate === tab));
      Object.entries(elements.appTabPanels).forEach(([key, panel]) => panel.classList.toggle("active", key === target));
      if (target !== "ik") invalidatePendingIkPreview();
      if (target !== "joint" && (state.viewPreviewSource === "joint" || state.draftAngles || state.pendingAngles || state.commandedAngles)) {
        const hadJointPreview = state.viewPreviewSource === "joint";
        releaseJointControlIntent();
        if (hadJointPreview) clearViewPreview();
        else syncJointControls();
      }
      if (target === "joint") {
        syncJointControls();
        const jointIntentAngles = state.draftAngles || state.commandedAngles || state.pendingAngles;
        if (jointIntentAngles) {
          state.viewPreviewSource = "joint";
          state.view.setPreviewAngles(jointIntentAngles);
        }
      }
      if (target === "ik") {
        scheduleIkPreview(0);
      }
      if (target === "operate") {
        renderDetectionList(state.latestDetections);
        renderWorkflowStepper(state.taskPreviewId ? "preview" : "detect");
      } else {
        setCameraPopupVisible(false);
        setCameraLive(false);
        invalidateTaskDetections("Refresh detections after returning to Tasks");
      }
      if (target === "settings") {
        loadWorkspaceCalibrationStatus();
        updateSettingsSaveBar();
      }
      state.view.resize();
      window.setTimeout(() => state.view.resize(), 300);
    });
  });

  elements.jointControls.addEventListener("input", (event) => {
    const input = event.target;
    if (!input.matches("[data-index]") || !state.robotState) return;
    const index = Number(input.dataset.index);
    const value = Number(input.value);
    const current = jointControlAngles();
    if (!current) return;
    const next = current.slice();
    invalidatePendingIkPreview();
    if (state.viewPreviewSource !== "joint") clearViewPreview();
    next[index] = value;
    if (state.intentBasePoseRevision == null) {
      state.intentBasePoseRevision = Number(state.robotState?.pose_revision ?? 0);
    }
    state.commandedAngles = null;
    state.previewAngles = null;
    state.draftAngles = next;
    state.ikUserEdited = false;
    syncJointControls();
    state.viewPreviewSource = "joint";
    state.view.setPreviewAngles(next);
    if (liveRealEnabled()) scheduleLiveTarget({ angles_deg: next });
    else if (elements.liveJogToggle.checked && isMoveEnabled()) queueTarget(next);
  });

  elements.applyJointPreviewBtn.addEventListener("click", async () => {
    const angles = state.draftAngles || state.robotState?.target_angles_deg;
    if (!angles) return;
    const payload = await postJson("/api/joints", { angles_deg: angles, settings: pathSettings() });
    if (payload.ok) {
      if (state.activeTab === "joint") {
        state.pendingAngles = angles.slice();
        state.commandedAngles = angles.slice();
        state.draftAngles = null;
        state.previewAngles = null;
        state.ikUserEdited = false;
        state.viewPreviewSource = "joint";
        state.view.setPreviewAngles(state.commandedAngles);
      } else {
        releaseJointControlIntent();
      }
    }
    if (payload.state) renderState(payload.state);
  });
  elements.resetJointPreviewBtn.addEventListener("click", () => {
    releaseJointControlIntent();
    if (state.viewPreviewSource === "joint") clearViewPreview();
    if (state.robotState) renderState(state.robotState);
  });
  elements.liveJogToggle.addEventListener("change", () => {
    if (!elements.liveJogToggle.checked) state.pendingAngles = null;
  });

  elements.resetViewBtn.addEventListener("click", () => state.view.resetCamera());
  elements.togglePreviewBtn.addEventListener("click", () => {
    elements.togglePreviewBtn.classList.toggle("active");
    state.view.setPreviewVisible(elements.togglePreviewBtn.classList.contains("active"));
  });
  elements.togglePathBtn.addEventListener("click", () => {
    elements.togglePathBtn.classList.toggle("active");
    state.view.setPathVisible(elements.togglePathBtn.classList.contains("active"));
  });
  elements.toggleFramesBtn.addEventListener("click", () => {
    elements.toggleFramesBtn.classList.toggle("active");
    state.view.setFramesVisible(elements.toggleFramesBtn.classList.contains("active"));
  });

  elements.connectSimBtn.addEventListener("click", () => postJson("/api/connect", { simulation: true }));
  elements.connectSerialBtn.addEventListener("click", openSerialModal);
  elements.refreshPortsBtn.addEventListener("click", refreshSerialPorts);
  elements.closeSerialModalBtn.addEventListener("click", () => setSerialModalVisible(false));
  elements.serialPortList.addEventListener("click", (event) => {
    const option = event.target.closest("[data-port]");
    if (!option) return;
    state.selectedSerialPort = option.dataset.port;
    refreshSerialPorts();
  });
  elements.connectSelectedSerialBtn.addEventListener("click", connectSelectedSerial);
  elements.disconnectBtn.addEventListener("click", () => postJson("/api/disconnect"));
  elements.homeBtn.addEventListener("click", async () => {
    const payload = await postJson("/api/home");
    if (payload.ok) {
      invalidatePendingIkPreview();
      releaseJointControlIntent();
      clearViewPreview();
      state.ikUserEdited = false;
    }
    if (payload.state) renderState(payload.state);
  });
  elements.setPoseBtn.addEventListener("click", setCurrentPoseKnown);
  elements.stopBtn.addEventListener("click", async () => {
    elements.cartesianJogToggle.checked = false;
    zeroCartesianJogVelocity();
    setCartesianJogStatus("idle");
    const payload = await postJson("/api/stop");
    if (payload.ok) {
      invalidatePendingIkPreview();
      releaseJointControlIntent();
      clearViewPreview();
      state.ikUserEdited = false;
    }
    if (payload.state) renderState(payload.state);
  });
  elements.diagnosticsBtn.addEventListener("click", async () => {
    elements.diagnosticsDrawer.hidden = false;
    await refreshDiagnostics();
  });
  elements.closeDiagnosticsBtn.addEventListener("click", () => {
    elements.diagnosticsDrawer.hidden = true;
  });
  elements.hardwareArmToggle.addEventListener("change", () => postJson("/api/hardware-arm", { armed: elements.hardwareArmToggle.checked }));
  elements.hardwareIo.addEventListener("input", markHardwareDraftDirty);
  elements.hardwareIo.addEventListener("change", markHardwareDraftDirty);
  elements.jointCalibration.addEventListener("input", markJointCalibrationDraftDirty);
  elements.jointCalibration.addEventListener("change", markJointCalibrationDraftDirty);
  elements.toolCalibration.addEventListener("input", markToolDraftDirty);
  elements.toolCalibration.addEventListener("change", (event) => {
    markToolDraftDirty();
    if (event.target?.dataset?.toolField === "type") {
      state.config.tools = readToolsPayload();
      renderToolEditor();
    }
  });
  elements.geometryPresetEditor?.addEventListener("input", () => {
    refreshDerivedModelDraft();
    markSettingsDirty("geometry", "Robot geometry changed. Preview it if needed, then save all settings.");
  });
  elements.geometryPresetEditor?.addEventListener("change", (event) => {
    if (event.target?.id === "geometryPresetSelect") {
      state.linkDraft = null;
      state.dhDraftRows = null;
      buildGeometryPresetEditor();
    }
    refreshDerivedModelDraft();
    markSettingsDirty("geometry", "Robot geometry changed. Preview it if needed, then save all settings.");
  });
  elements.geometryPresetEditor?.addEventListener("click", (event) => {
    if (event.target?.id === "applyGeometryPresetBtn") applyGeometryPresetToDhDraft();
  });
  elements.syncHardwareBtn.addEventListener("click", async () => {
    const saved = await saveAllSettings();
    if (!saved) return;
    updateSettingsSaveBar({ mode: "saving", title: "Syncing controller", detail: "Sending the saved hardware configuration to the ESP…" });
    const payload = await postJson("/api/hardware/sync");
    if (payload.state) renderState(payload.state);
    updateSettingsSaveBar(
      payload.ok
        ? { title: "All settings saved", detail: payload.message || "Saved locally and synced to the controller." }
        : { mode: "error", title: "Controller sync failed", detail: payload.message || payload.error || "Hardware settings remain saved locally." }
    );
  });
  [
    elements.globalSpeedInput,
    elements.globalAccelInput,
    elements.waypointRateInput,
    elements.cartesianStepInput,
    elements.plannerTypeSelect,
    elements.jerkPercentInput,
    elements.blendPercentInput,
  ].forEach((input) => {
    const handleMotionSettingsChange = () => {
      markSettingsDirty("motion", "Motion defaults changed. Save all settings to persist them.");
      if (state.programWaypoints.length && !state.programExecutionActive) {
        markProgramEdited("Motion settings changed", { inspector: false });
      }
    };
    input?.addEventListener("input", handleMotionSettingsChange);
  });
  elements.perJointTuning?.addEventListener("input", () => {
    markSettingsDirty("motion", "Per-joint motion limits changed. Save all settings to persist them.");
    if (state.programWaypoints.length && !state.programExecutionActive) {
      markProgramEdited("Per-joint motion limits changed", { inspector: false });
    }
  });
  [
    elements.cameraEnabledInput,
    elements.cameraSourceInput,
    elements.cameraWidthInput,
    elements.cameraHeightInput,
    elements.workspaceProjectionInput,
    elements.workspaceArucoEnabledInput,
    elements.workspaceArucoInvertInput,
    elements.workspaceArucoFallbackInput,
    elements.cameraFxInput,
    elements.cameraFyInput,
    elements.cameraCxInput,
    elements.cameraCyInput,
    elements.cameraDistortionInput,
  ].forEach((input) => {
    input?.addEventListener("input", () => markSettingsDirty("camera", "Camera settings changed. Save all settings to persist them."));
    input?.addEventListener("change", () => markSettingsDirty("camera", "Camera settings changed. Save all settings to persist them."));
  });
  elements.workspaceProjectionInput?.addEventListener("change", () => {
    if (!elements.workspaceProjectionInput.checked) {
      window.clearTimeout(state.workspaceProjectionTimer);
      state.workspaceProjectionTimer = null;
      state.view?.setWorkspaceCameraProjection(null, false);
      return;
    }
    scheduleWorkspaceProjection(0);
  });
  elements.settingsSectionNav?.addEventListener("click", (event) => {
    const button = event.target.closest("[data-settings-target]");
    if (!button) return;
    const section = $(`#${button.dataset.settingsTarget}`);
    if (!section) return;
    elements.settingsSectionNav.querySelectorAll("[data-settings-target]").forEach((item) => {
      item.classList.toggle("active", item === button);
    });
    section.scrollIntoView({ behavior: "smooth", block: "start" });
  });
  elements.liveRealToggle.addEventListener("change", async () => {
    const wasCartesianJogEnabled = cartesianJogEnabled();
    const wasSimulation = Boolean(state.robotState?.simulation);
    const payload = await postJson("/api/live-motion", { enabled: elements.liveRealToggle.checked });
    elements.liveRealToggle.checked = Boolean(payload.ok && payload.state?.live_motion_enabled);
    if (!elements.liveRealToggle.checked && wasCartesianJogEnabled && !wasSimulation) {
      await stopCartesianJog();
      elements.cartesianJogToggle.checked = false;
    }
    if (payload.state) renderState(payload.state);
    updateDisabledState();
  });
  elements.cartesianJogToggle?.addEventListener("change", async () => {
    if (elements.cartesianJogToggle.checked && !cartesianJogCanRun()) {
      elements.cartesianJogToggle.checked = false;
      showLocalError(state.robotState?.simulation ? "Cartesian jog is not available." : "Enable Live Real and Arm before hardware Cartesian jog.");
      updateDisabledState();
      return;
    }
    if (elements.cartesianJogToggle.checked) {
      invalidatePendingIkPreview();
      releaseJointControlIntent();
      clearViewPreview();
      state.ikUserEdited = false;
      setIkTargetFromFk(state.robotState?.fk);
      setCartesianJogStatus("ready");
    } else {
      await stopCartesianJog();
    }
    updateDisabledState();
  });
  elements.namedPositionsList?.addEventListener("click", (event) => {
    const previewButton = event.target.closest("[data-named-preview]");
    const applyButton = event.target.closest("[data-named-apply]");
    if (previewButton) previewNamedPosition(previewButton.dataset.namedPreview);
    if (applyButton) moveNamedPosition(applyButton.dataset.namedApply);
  });
  elements.toolSelect.addEventListener("change", () => saveActiveTool(elements.toolSelect.value));
  elements.toolValueSlider.addEventListener("pointerdown", beginToolSliderEdit);
  elements.toolValueSlider.addEventListener("pointerup", () => {
    endToolSliderEdit();
    queueToolSliderLiveSet({ immediate: true });
  });
  elements.toolValueSlider.addEventListener("pointercancel", () => {
    endToolSliderEdit();
    queueToolSliderLiveSet({ immediate: true });
  });
  elements.toolValueSlider.addEventListener("focus", beginToolSliderEdit);
  elements.toolValueSlider.addEventListener("blur", () => {
    endToolSliderEdit();
    queueToolSliderLiveSet({ immediate: true });
  });
  elements.toolValueSlider.addEventListener("input", () => {
    state.toolSliderEditing = true;
    queueToolSliderLiveSet();
  });
  elements.toolValueSlider.addEventListener("change", () => {
    endToolSliderEdit();
    queueToolSliderLiveSet({ immediate: true });
  });
  elements.toolOpenBtn.addEventListener("click", () => sendTool("open"));
  elements.toolCloseBtn.addEventListener("click", () => sendTool("close"));
  elements.toolOnBtn.addEventListener("click", () => sendTool("on"));
  elements.toolOffBtn.addEventListener("click", () => sendTool("off"));
  elements.taskWorkflowStepper?.addEventListener("click", (event) => {
    const button = event.target.closest("[data-task-step]");
    if (!button) return;
    state.activeTaskStep = button.dataset.taskStep;
    renderWorkflowStepper(state.taskProgressStep || state.activeTaskStep);
  });
  elements.colorPresetMapping?.addEventListener("change", (event) => {
    const enabledInput = event.target.closest("[data-color-enabled]");
    const zoneSelect = event.target.closest("[data-color-drop-zone]");
    if (enabledInput) {
      updateColorProfileDraft(enabledInput.dataset.colorEnabled, { enabled: enabledInput.checked });
    }
    if (zoneSelect) {
      updateColorProfileDraft(zoneSelect.dataset.colorDropZone, { drop_zone: zoneSelect.value });
    }
  });
  elements.dropPresetEditor?.addEventListener("input", (event) => {
    const input = event.target.closest("[data-drop-zone-field]");
    if (!input) return;
    updateDropZoneDraft(input.dataset.dropZone, input.dataset.dropZoneField, input.value);
  });
  elements.dropPresetEditor?.addEventListener("click", (event) => {
    const button = event.target.closest("[data-drop-zone-delete]");
    if (!button) return;
    deleteDropPreset(button.dataset.dropZoneDelete);
  });
  elements.colorProfileEditor?.addEventListener("change", (event) => {
    const enabledInput = event.target.closest("[data-settings-color-enabled]");
    const zoneSelect = event.target.closest("[data-settings-color-drop-zone]");
    if (enabledInput) {
      updateColorProfileDraft(enabledInput.dataset.settingsColorEnabled, { enabled: enabledInput.checked });
    }
    if (zoneSelect) {
      updateColorProfileDraft(zoneSelect.dataset.settingsColorDropZone, { drop_zone: zoneSelect.value });
    }
  });
  elements.addDropPresetBtn?.addEventListener("click", addDropPreset);
  [
    elements.taskModeSelect,
    elements.executionStrategySelect,
    elements.objectSelectionPolicySelect,
    elements.maxObjectsInput,
    elements.minConfidenceInput,
    elements.includeColorsSelect,
    elements.colorPriorityInput,
    elements.missingDropzonePolicySelect,
    elements.unknownColorPolicySelect,
    elements.placementPolicySelect,
    elements.dropZoneSelect,
    elements.sortColorSelect,
    elements.pickupZInput,
    elements.dropoffZInput,
    elements.approachClearanceInput,
    elements.dropApproachClearanceInput,
    elements.orientationPolicySelect,
    elements.pickupPhiInput,
    elements.dropPhiInput,
    elements.transferModeSelect,
    elements.pickupDescentModeSelect,
    elements.liftModeSelect,
    elements.dropDescentModeSelect,
    elements.captureSettleInput,
    elements.toolSettleInput,
    elements.objectProfilesInput,
  ].forEach((input) => input?.addEventListener("input", () => invalidateTaskPreview("Task settings changed")));
  [
    elements.taskModeSelect,
    elements.executionStrategySelect,
    elements.objectSelectionPolicySelect,
    elements.includeColorsSelect,
    elements.missingDropzonePolicySelect,
    elements.unknownColorPolicySelect,
    elements.placementPolicySelect,
    elements.dropZoneSelect,
    elements.sortColorSelect,
    elements.orientationPolicySelect,
    elements.transferModeSelect,
    elements.pickupDescentModeSelect,
    elements.liftModeSelect,
    elements.dropDescentModeSelect,
  ].forEach((input) => input?.addEventListener("change", () => invalidateTaskPreview("Task settings changed")));
  elements.detectionList?.addEventListener("change", (event) => {
    const input = event.target.closest("[data-detection-select]");
    if (!input) return;
    const id = String(input.dataset.detectionSelect);
    if (input.checked) state.selectedDetectionIds.add(id);
    else state.selectedDetectionIds.delete(id);
    invalidateTaskPreview("Detection selection changed");
    renderDetectionList(state.latestDetections);
  });
  elements.detectionList?.addEventListener("click", (event) => {
    const button = event.target.closest("[data-runtime-select]");
    if (!button) return;
    selectRuntimeDetection(button.dataset.runtimeSelect);
  });
  elements.taskRunMonitor?.addEventListener("click", (event) => {
    const button = event.target.closest("[data-runtime-select]");
    if (!button) return;
    selectRuntimeDetection(button.dataset.runtimeSelect);
  });
  elements.previewTaskBtn.addEventListener("click", previewTask);
  elements.executeTaskBtn.addEventListener("click", executeTask);
  elements.taskStopBtn?.addEventListener("click", stopTask);
  elements.viewCameraBtn?.addEventListener("click", () => {
    setCameraPopupVisible(true);
    detectVision();
  });
  elements.detectVisionBtn.addEventListener("click", detectVision);
  elements.cameraPopupRefreshBtn?.addEventListener("click", detectVision);
  elements.closeCameraPopupBtn?.addEventListener("click", () => setCameraPopupVisible(false));
  elements.cameraLiveToggle?.addEventListener("change", () => setCameraLive(elements.cameraLiveToggle.checked));
  elements.calibrateWorkspaceBtn?.addEventListener("click", calibrateWorkspace);
  elements.verifyWorkspaceCalibrationBtn?.addEventListener("click", verifyWorkspaceCalibration);
  elements.tcpCalGenerateBtn?.addEventListener("click", generateTcpCalibrationTargets);
  elements.tcpCalPreviewFitBtn?.addEventListener("click", () => previewTcpCalibrationMove(false, "fit"));
  elements.tcpCalPreviewValidationBtn?.addEventListener("click", () => previewTcpCalibrationMove(true, "validation"));
  elements.tcpCalExecuteBtn?.addEventListener("click", executeTcpCalibrationMove);
  elements.tcpCalCaptureXyBtn?.addEventListener("click", captureTcpCalibrationXy);
  elements.tcpCalUseTouchOffBtn?.addEventListener("click", useTcpCalibrationTouchOff);
  elements.tcpCalSaveSampleBtn?.addEventListener("click", saveTcpCalibrationSample);
  elements.tcpCalFitBtn?.addEventListener("click", fitTcpCalibration);
  elements.tcpCalApplyEnableBtn?.addEventListener("click", applyTcpCalibrationEnableState);
  elements.tcpCalibrationTargetList?.addEventListener("click", (event) => {
    const button = event.target.closest("[data-tcp-cal-target]");
    if (!button) return;
    const point = state.tcpCalibrationTargets[Number(button.dataset.tcpCalTarget)];
    if (point) setTcpCalibrationTarget(point.intended_target);
  });
  elements.tcpCalibrationSamples?.addEventListener("click", (event) => {
    const button = event.target.closest("[data-tcp-cal-delete]");
    if (button) deleteTcpCalibrationSample(button.dataset.tcpCalDelete);
  });

  elements.sliderRangeControls.addEventListener("input", () => {
    state.ikUserEdited = true;
    applySliderRanges();
    scheduleIkPreview();
  });
  elements.ikTargetControls.addEventListener("input", (event) => {
    const input = event.target;
    if (!input.matches("[data-target-key]")) return;
    state.ikUserEdited = true;
    const key = input.dataset.targetKey;
    const pair = input.type === "range" ? $(`#ik-${key}-input`) : $(`#ik-${key}-slider`);
    if (pair) pair.value = input.value;
    scheduleIkPreview();
  });
  elements.ikModeSelect.addEventListener("change", () => scheduleIkPreview());
  elements.ikBranchSelect.addEventListener("change", () => scheduleIkPreview());
  elements.ikAutoPhiToggle.addEventListener("change", () => {
    state.ikUserEdited = true;
    updatePhiControlState();
    scheduleIkPreview();
  });
  elements.previewIkBtn.addEventListener("click", () => {
    state.ikPreviewWantedSeq += 1;
    previewIkPath();
  });
  elements.executeIkBtn.addEventListener("click", () => executePreview());
  elements.ikStopBtn.addEventListener("click", () => postJson("/api/stop"));

  elements.appTabPanels.program.addEventListener("click", (event) => {
    const quickAdd = event.target.closest("[data-program-quick-source]");
    if (quickAdd) {
      addProgramStep(quickAdd.dataset.programQuickSource);
    }
  });
  elements.programStepSource.addEventListener("change", () => {
    renderProgramSourceOptions();
    updateDisabledState();
  });
  elements.programSourceItem.addEventListener("change", updateDisabledState);
  elements.addProgramStepBtn.addEventListener("click", () => addProgramStep());
  elements.clearProgramBtn.addEventListener("click", () => {
    if (!window.confirm(`Clear all ${state.programWaypoints.length} program steps? This cannot be undone.`)) return;
    state.programWaypoints = [];
    state.programSelectedId = null;
    state.programPreview = null;
    state.programPreviewFailure = null;
    state.programPreviewRevision = null;
    state.programValidationRevision = null;
    state.programHasPreviewed = false;
    markProgramEdited("Program cleared");
  });
  elements.previewProgramBtn.addEventListener("click", previewProgram);
  elements.executeProgramBtn.addEventListener("click", executeProgram);
  elements.programList.addEventListener("click", (event) => {
    const actionControl = event.target.closest("[data-program-action]");
    if (actionControl) {
      const index = Number(actionControl.dataset.programIndex);
      const action = actionControl.dataset.programAction;
      if (action === "toggle") {
        state.programWaypoints[index].enabled = actionControl.checked;
        state.programSelectedId = state.programWaypoints[index].id;
        markProgramEdited(`${actionControl.checked ? "Enabled" : "Disabled"} ${state.programWaypoints[index].label || `step ${index + 1}`}`);
      }
      if (action === "delete") deleteProgramStep(index);
      if (action === "duplicate") duplicateProgramStep(index);
      if (action === "up") moveProgramStep(index, -1);
      if (action === "down") moveProgramStep(index, 1);
      return;
    }
    const row = event.target.closest("[data-program-select]");
    if (row) {
      const step = state.programWaypoints[Number(row.dataset.programSelect)];
      state.programSelectedId = step?.id || null;
      renderProgramList();
      renderProgramInspector();
    }
  });
  elements.programInspector.addEventListener("input", (event) => {
    if (event.target.matches('input[type="text"], input[type="number"]')) {
      updateProgramStepFromControl(event.target);
    }
  });
  elements.programInspector.addEventListener("change", (event) => {
    if (event.target.matches("select, input[type='checkbox']")) {
      updateProgramStepFromControl(event.target);
      renderProgramInspector();
    }
  });
  elements.programInspector.addEventListener("click", (event) => {
    const action = event.target.closest("[data-program-inspector-action]")?.dataset.programInspectorAction;
    const index = selectedProgramIndex();
    if (action === "duplicate") duplicateProgramStep(index);
    if (action === "delete") deleteProgramStep(index);
  });

  elements.saveCalibrationBtn.addEventListener("click", saveAllSettings);
  elements.buildStatus?.addEventListener("click", () => {
    if (elements.buildStatus.dataset.action === "reload") {
      window.location.reload();
    } else {
      checkAppVersion();
    }
  });
  elements.discardSettingsBtn?.addEventListener("click", discardSettingsChanges);
  bindFaders();
  bindPanelChrome();
  bindCameraPopup();
}

async function init() {
  state.view = new RobotView($("#robotViewport"));
  await loadConfig();
  bindActions();
  await postJson("/api/live-motion", { enabled: false });
  await checkAppVersion();
  state.versionTimer = window.setInterval(checkAppVersion, 15000);
  connectWebSocket();
  await loadWorkspaceCalibrationStatus();
}

init();
