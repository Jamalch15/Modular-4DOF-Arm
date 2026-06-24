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
  programPlanRestorePending: false,
  programSavedPlanStatus: "",
  programHasPreviewed: false,
  programExecutionActive: false,
  programExecutionAwaitingStart: false,
  programExecutionFailed: false,
  programExecutionError: "",
  programLastEditReason: "",
  programNextId: 1,
  activeProgramStage: "library",
  programLibrary: [],
  programActiveId: null,
  programName: "Untitled program",
  programDescription: "",
  programReadOnly: false,
  programDirty: false,
  programSaving: false,
  programTargetPreview: null,
  programTargetPreviewPending: false,
  programTargetMovePending: false,
  programTargetStatus: "",
  programPlaybackPlaying: false,
  programPlaybackElapsedS: 0,
  programPlaybackStartedAtMs: 0,
  programPlaybackFrame: null,
  programPlaybackRate: 1,
  hardwareDraftDirty: false,
  settingsDirtyScopes: new Set(),
  taskPreviewId: null,
  taskPreviewCreatedAt: null,
  taskPreviewPending: false,
  taskLocalStatusAt: 0,
  lastTaskPreview: null,
  selectedDetectionIds: new Set(),
  selectedSerialPort: null,
  latestDetections: [],
  taskDetectionsCapturedAt: null,
  taskDetectionSnapshotId: null,
  taskDetectionMinAreaPx: null,
  activeTaskStep: "setup",
  taskProgressStep: "setup",
  taskColorProfilesDraft: null,
  taskDestinationsDraft: null,
  unsavedColorProfiles: new Set(),
  taskMappingSaveTimer: null,
  taskMappingSavePromise: null,
  positionLibraryDraft: null,
  positionLibraryErrors: {},
  positionLibrarySaving: false,
  pendingSetPoseAngles: null,
  toolSwitchPending: false,
  diagnosticsRenderTimer: null,
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
  tcpCalibrationPhysicalResult: null,
  encoderCalibrationSession: null,
  encoderCalibrationValidation: null,
  encoderCalibrationMessage: "",
  encoderSweepPollTimer: null,
  encoderLiveSamples: [],
  encoderBacklashResult: null,
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

const ENCODER_RECOMMENDED_PINS = {
  sck_pin: 12,
  miso_pin: 13,
  mosi_pin: 14,
  cs_pin: 15,
};

const ENCODER_STANDARD_LIMITS = {
  clock_hz: 1000000,
  sample_interval_ms: 100,
  freshness_timeout_ms: 500,
  max_noise_deg: 0.5,
  settle_delay_ms: 300,
  required_stable_samples: 3,
  warning_tolerance_deg: 2.0,
  fault_tolerance_deg: 10.0,
  hysteresis_deg: 0.25,
  correction_deadband_deg: 0.75,
  correction_max_delta_deg: 8.0,
  align_max_delta_deg: 60.0,
  pose_tracking_min_update_delta_deg: 0.10,
  pose_tracking_max_jump_deg: 180.0,
  pose_tracking_preview_stale_tolerance_deg: 2.0,
};

const ENCODER_BACKLASH_PRELOAD_DEG = 8.0;

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
  setPoseModal: $("#setPoseModal"),
  setPoseAngles: $("#setPoseAngles"),
  cancelSetPoseBtn: $("#cancelSetPoseBtn"),
  confirmSetPoseBtn: $("#confirmSetPoseBtn"),
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
  viewportSyncHardwareBtn: $("#viewportSyncHardwareBtn"),
  homeBtn: $("#homeBtn"),
  alignShoulderBtn: $("#alignShoulderBtn"),
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
  tcpSpeedInput: $("#tcpSpeedInput"),
  tcpAccelInput: $("#tcpAccelInput"),
  phiSpeedInput: $("#phiSpeedInput"),
  phiAccelInput: $("#phiAccelInput"),
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
  positionLibraryStatus: $("#positionLibraryStatus"),
  positionLibraryList: $("#positionLibraryList"),
  addJointPositionBtn: $("#addJointPositionBtn"),
  addCartesianPositionBtn: $("#addCartesianPositionBtn"),
  saveCurrentPositionBtn: $("#saveCurrentPositionBtn"),
  savePositionLibraryBtn: $("#savePositionLibraryBtn"),
  resetPositionLibraryBtn: $("#resetPositionLibraryBtn"),
  saveTaskMappingsBtn: $("#saveTaskMappingsBtn"),
  discardTaskMappingsBtn: $("#discardTaskMappingsBtn"),
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
  programPanels: document.querySelectorAll("[data-program-panel]"),
  programStatusDetail: $("#programStatusDetail"),
  programLibraryStatus: $("#programLibraryStatus"),
  programTemplateNotice: $("#programTemplateNotice"),
  programNameInput: $("#programNameInput"),
  programDescriptionInput: $("#programDescriptionInput"),
  newProgramBtn: $("#newProgramBtn"),
  saveProgramBtn: $("#saveProgramBtn"),
  copyProgramBtn: $("#copyProgramBtn"),
  builtInProgramList: $("#builtInProgramList"),
  userProgramList: $("#userProgramList"),
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
  programPlayback: $("#programPlayback"),
  programPlaybackStep: $("#programPlaybackStep"),
  programPlaybackTime: $("#programPlaybackTime"),
  programPlaybackProgress: $("#programPlaybackProgress"),
  programPlaybackToggle: $("#programPlaybackToggle"),
  programPlaybackRestart: $("#programPlaybackRestart"),
  programPlaybackRate: $("#programPlaybackRate"),
  stopProgramBtn: $("#stopProgramBtn"),
  programRunMonitor: $("#programRunMonitor"),
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
  cycleConfirmationSelect: $("#cycleConfirmationSelect"),
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
  taskPreviewFeedback: $("#taskPreviewFeedback"),
  executeTaskBtn: $("#executeTaskBtn"),
  taskStopBtn: $("#taskStopBtn"),
  taskStatus: $("#taskStatus"),
  taskSummary: $("#taskSummary"),
  taskPlanPreview: $("#taskPlanPreview"),
  taskRunMonitor: $("#taskRunMonitor"),
  viewCameraBtn: $("#viewCameraBtn"),
  detectVisionBtn: $("#detectVisionBtn"),
  detectionMinAreaInput: $("#detectionMinAreaInput"),
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
  modelTruthSummary: $("#modelTruthSummary"),
  toolCalibration: $("#toolCalibration"),
  toolValidationStatus: $("#toolValidationStatus"),
  toolValidationBtn: $("#toolValidationBtn"),
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
  workspaceMarginInput: $("#workspaceMarginInput"),
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
  tcpCalibrationReferenceStatus: $("#tcpCalibrationReferenceStatus"),
  tcpCalWorkspacePlaneZInput: $("#tcpCalWorkspacePlaneZInput"),
  tcpCalMeasuredPointSelect: $("#tcpCalMeasuredPointSelect"),
  tcpCalApproachSelect: $("#tcpCalApproachSelect"),
  tcpCalReferenceConfirmInput: $("#tcpCalReferenceConfirmInput"),
  tcpCalPhysicalModelSelect: $("#tcpCalPhysicalModelSelect"),
  tcpCalFitPhysicalBtn: $("#tcpCalFitPhysicalBtn"),
  tcpCalApplyPhysicalBtn: $("#tcpCalApplyPhysicalBtn"),
  tcpCalibrationPhysicalMetrics: $("#tcpCalibrationPhysicalMetrics"),
  tcpCalManualReachOffsetInput: $("#tcpCalManualReachOffsetInput"),
  tcpCalManualZOffsetInput: $("#tcpCalManualZOffsetInput"),
  tcpCalModelSelect: $("#tcpCalModelSelect"),
  tcpCalEnableInput: $("#tcpCalEnableInput"),
  tcpCalSaveManualOffsetsBtn: $("#tcpCalSaveManualOffsetsBtn"),
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
          <span>Estimated/planning <strong id="reported-${index}">0.0 deg</strong></span>
          <span>Measured <strong id="measured-${index}">unknown</strong></span>
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

const robotSettingsScopes = new Set(["geometry", "joints", "motion", "tooling", "hardware", "task_destinations", "calibration"]);
const CORE_POSITION_IDS = new Set(["home"]);

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
  if (scope === "motion") {
    invalidateTaskPreview("Motion settings changed - preview again");
  } else if (["geometry", "tooling", "hardware", "camera"].includes(scope)) {
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

function savedPositionLibraryRecords() {
  return clonePlain(state.config?.position_library?.positions || {});
}

function ensurePositionLibraryDraft() {
  if (!state.positionLibraryDraft) {
    state.positionLibraryDraft = savedPositionLibraryRecords();
  }
  return state.positionLibraryDraft;
}

function positionLibraryDirty() {
  if (!state.config) return false;
  return JSON.stringify(state.positionLibraryDraft || savedPositionLibraryRecords()) !== JSON.stringify(savedPositionLibraryRecords());
}

function planningPoseMetadata(robotState = state.robotState) {
  if (!robotState) return {};
  return {
    pose_basis: "planning_estimate",
    pose_source: robotState.pose_source || "unknown",
    planning_known_mask: robotState.pose_known_mask || "0000",
    joint_authority: clonePlain(robotState.joint_authority || []),
    measurement_valid_mask: robotState.measurement_valid_mask || "0000",
    shoulder_measurement: clonePlain(robotState.encoder_evidence?.[1] || {}),
  };
}

function setPositionLibraryStatus(text = null, mode = "") {
  if (!elements.positionLibraryStatus) return;
  const dirty = positionLibraryDirty();
  elements.positionLibraryStatus.textContent = text || (dirty ? "Unsaved edits" : "Saved");
  elements.positionLibraryStatus.classList.toggle("warn", mode === "warn" || dirty);
  elements.positionLibraryStatus.classList.toggle("error", mode === "error");
}

function updatePositionLibraryControls() {
  const dirty = positionLibraryDirty();
  if (elements.savePositionLibraryBtn) {
    elements.savePositionLibraryBtn.disabled = !dirty || state.positionLibrarySaving;
  }
  if (elements.resetPositionLibraryBtn) {
    elements.resetPositionLibraryBtn.disabled = !dirty || state.positionLibrarySaving;
  }
  setPositionLibraryStatus(state.positionLibrarySaving ? "Saving..." : null, dirty ? "warn" : "");
}

function positionDisplayName(positionId, record = {}) {
  return String(record.display_name || record.label || record.name || positionId);
}

function positionRecordKind(record = {}) {
  return String(record.type || record.kind || "joint").toLowerCase() === "cartesian" ? "cartesian" : "joint";
}

function formatPositionRecord(record = {}) {
  if (positionRecordKind(record) === "joint") {
    return (record.angles_deg || []).map((value, index) => `J${index + 1} ${format(value, 1)}`).join(", ");
  }
  return formatCartesianTarget(record.target || record);
}

function positionLibraryFieldsHtml(positionId, record = {}) {
  if (positionRecordKind(record) === "joint") {
    const angles = record.angles_deg || [];
    const count = Math.max(state.config?.joints?.length || 4, angles.length);
    return `
      <div class="position-library-fields">
        ${Array.from({ length: count }, (_, index) => `
          <label>J${index + 1}
            <input type="number" step="0.1" data-position-field="angles_deg.${index}" data-position-id="${escapeHtml(positionId)}" value="${format(angles[index] ?? 0, 1)}" />
          </label>
        `).join("")}
      </div>
    `;
  }
  const target = record.target || record;
  return `
    <div class="position-library-fields">
      <label>X <input type="number" step="1" data-position-field="target.x_mm" data-position-id="${escapeHtml(positionId)}" value="${format(target.x_mm ?? 0, 1)}" /></label>
      <label>Y <input type="number" step="1" data-position-field="target.y_mm" data-position-id="${escapeHtml(positionId)}" value="${format(target.y_mm ?? 0, 1)}" /></label>
      <label>Z <input type="number" step="1" data-position-field="target.z_mm" data-position-id="${escapeHtml(positionId)}" value="${format(target.z_mm ?? 45, 1)}" /></label>
      <label>Phi <input type="number" step="1" data-position-field="target.phi_deg" data-position-id="${escapeHtml(positionId)}" value="${format(target.phi_deg ?? 0, 1)}" /></label>
    </div>
  `;
}

function sortedPositionEntries(records) {
  return Object.entries(records).sort(([leftId], [rightId]) => {
    const leftCore = CORE_POSITION_IDS.has(leftId) ? 0 : 1;
    const rightCore = CORE_POSITION_IDS.has(rightId) ? 0 : 1;
    if (leftCore !== rightCore) return leftCore - rightCore;
    return leftId.localeCompare(rightId);
  });
}

function renderPositionLibrary() {
  if (!elements.positionLibraryList || !state.config) return;
  const records = ensurePositionLibraryDraft();
  const entries = sortedPositionEntries(records);
  elements.positionLibraryList.innerHTML = entries.length
      ? entries.map(([positionId, record]) => {
        const kind = positionRecordKind(record);
        const core = CORE_POSITION_IDS.has(positionId);
        const source = record.source || (core ? "core" : "saved");
        const validationErrors = state.positionLibraryErrors?.[positionId] || [];
        return `
          <div class="position-library-row ${core ? "core-position" : ""} ${validationErrors.length ? "invalid" : ""}" data-position-id="${escapeHtml(positionId)}">
            <div class="position-library-title">
              <strong>${escapeHtml(positionDisplayName(positionId, record))}</strong>
              <small>${escapeHtml(positionId)} | ${kind} | ${escapeHtml(source)}</small>
            </div>
            <label>Name
              <input type="text" data-position-field="display_name" data-position-id="${escapeHtml(positionId)}" value="${escapeHtml(positionDisplayName(positionId, record))}" />
            </label>
            ${positionLibraryFieldsHtml(positionId, record)}
            <code>${escapeHtml(formatPositionRecord(record))}</code>
            ${validationErrors.length ? `<p class="position-library-error">${validationErrors.map(escapeHtml).join("<br />")}</p>` : ""}
            <div class="button-row position-library-actions">
              <button type="button" class="ghost" data-position-preview="${escapeHtml(positionId)}">Preview</button>
              <button type="button" data-position-go="${escapeHtml(positionId)}">Go To</button>
              <button type="button" class="ghost" data-position-duplicate="${escapeHtml(positionId)}">Duplicate</button>
              <button type="button" class="danger ghost" data-position-delete="${escapeHtml(positionId)}" ${core ? "disabled title=\"Home is required by the robot configuration\"" : ""}>Delete</button>
            </div>
          </div>
        `;
      }).join("")
    : `<div class="empty-state">No saved positions yet.</div>`;
  updatePositionLibraryControls();
}

function markPositionLibraryDirty(message = "Unsaved position-library edits") {
  setPositionLibraryStatus(message, "warn");
  updatePositionLibraryControls();
}

function updatePositionLibraryDraft(positionId, field, value) {
  const records = ensurePositionLibraryDraft();
  if (!records[positionId]) return;
  delete state.positionLibraryErrors[positionId];
  if (field === "display_name") {
    records[positionId].display_name = String(value || "").trim() || positionId;
  } else if (field.startsWith("angles_deg.")) {
    const index = Number(field.slice("angles_deg.".length));
    if (!Number.isInteger(index) || index < 0) return;
    const angles = Array.isArray(records[positionId].angles_deg) ? records[positionId].angles_deg : [];
    const numeric = Number(value);
    angles[index] = Number.isFinite(numeric) ? numeric : 0;
    records[positionId].type = "joint";
    records[positionId].angles_deg = angles;
  } else if (field.startsWith("target.")) {
    const key = field.slice("target.".length);
    if (!["x_mm", "y_mm", "z_mm", "phi_deg"].includes(key)) return;
    const numeric = Number(value);
    records[positionId].type = "cartesian";
    records[positionId].target = records[positionId].target || {};
    records[positionId].target[key] = Number.isFinite(numeric) ? numeric : 0;
  }
  markPositionLibraryDirty();
}

function addPositionLibraryRecord(kind) {
  const records = ensurePositionLibraryDraft();
  const displayName = kind === "cartesian" ? "New Cartesian Position" : "New Joint Position";
  const positionId = uniquePositionId(displayName);
  if (kind === "cartesian") {
    const fk = state.robotState?.fk || {};
    records[positionId] = {
      schema_version: 1,
      id: positionId,
      display_name: displayName,
      type: "cartesian",
      units: { length: "mm", angle: "deg" },
      source: "operator",
      metadata: planningPoseMetadata(),
      target: {
        x_mm: Number(fk.x_mm ?? 0),
        y_mm: Number(fk.y_mm ?? 180),
        z_mm: Number(fk.z_mm ?? 45),
        phi_deg: Number(fk.tool_phi_deg ?? fk.phi_deg ?? 0),
      },
    };
  } else {
    const angles = normalizeJointAngles(state.robotState?.reported_angles_deg)
      || normalizeJointAngles(jointControlAngles())
      || normalizeJointAngles(state.config?.joints?.map((joint) => joint.home_deg))
      || [];
    records[positionId] = {
      schema_version: 1,
      id: positionId,
      display_name: displayName,
      type: "joint",
      units: { length: "mm", angle: "deg" },
      source: "operator",
      metadata: planningPoseMetadata(),
      angles_deg: angles,
    };
  }
  state.positionLibraryErrors = {};
  markPositionLibraryDirty(`Added ${displayName}. Save Library to persist.`);
  renderPositionLibrary();
}

function duplicatePositionLibraryRecord(positionId) {
  const records = ensurePositionLibraryDraft();
  const source = records[positionId];
  if (!source) return;
  const baseName = `${positionDisplayName(positionId, source)} Copy`;
  const copyId = uniquePositionId(baseName);
  records[copyId] = {
    ...clonePlain(source),
    id: copyId,
    display_name: baseName,
    source: "operator",
  };
  delete records[copyId].created_at;
  delete records[copyId].updated_at;
  delete state.positionLibraryErrors[copyId];
  markPositionLibraryDirty(`Duplicated ${positionDisplayName(positionId, source)}. Save Library to persist.`);
  renderPositionLibrary();
}

function resetPositionLibraryDraft() {
  state.positionLibraryDraft = savedPositionLibraryRecords();
  state.positionLibraryErrors = {};
  renderPositionLibrary();
}

function positionIdFromName(name) {
  const base = String(name || "")
    .trim()
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "_")
    .replace(/^_+|_+$/g, "");
  return base || "position";
}

function uniquePositionId(displayName) {
  const records = ensurePositionLibraryDraft();
  const base = positionIdFromName(displayName);
  let candidate = base;
  let index = 2;
  while (records[candidate]) {
    candidate = `${base}_${index}`;
    index += 1;
  }
  return candidate;
}

function positionLibraryErrorText(payload) {
  if (payload.error) return payload.error;
  const errors = payload.errors || {};
  const first = Object.entries(errors)[0];
  if (!first) return "Position library could not be saved.";
  const formatErrorValue = (value) => {
    if (Array.isArray(value)) return value.join("; ");
    if (value && typeof value === "object") {
      return Object.entries(value)
        .map(([key, nested]) => `${key}: ${formatErrorValue(nested)}`)
        .join("; ");
    }
    return String(value || "invalid value");
  };
  return `${first[0]}: ${formatErrorValue(first[1])}`;
}

async function savePositionLibrary(options = {}) {
  if (!state.config) return { ok: false, error: "configuration not loaded" };
  state.positionLibrarySaving = true;
  updatePositionLibraryControls();
  const payload = await postJson("/api/position-library", {
    positions: ensurePositionLibraryDraft(),
  });
  state.positionLibrarySaving = false;
  if (payload.ok) {
    state.positionLibraryErrors = {};
    if (payload.config) applyConfig(payload.config);
    if (payload.state) renderState(payload.state);
    setPositionLibraryStatus(options.message || "Saved", "");
  } else {
    state.positionLibraryErrors = payload.errors || {};
    setPositionLibraryStatus(positionLibraryErrorText(payload), "error");
    renderPositionLibrary();
  }
  updatePositionLibraryControls();
  return payload;
}

async function requestJson(url, { method = "GET", body = null } = {}) {
  const options = { method, headers: { "Content-Type": "application/json" } };
  if (body !== null) options.body = JSON.stringify(body);
  const response = await fetch(url, options);
  const payload = await response.json();
  if (!response.ok && !payload.error) {
    payload.ok = false;
    payload.error = formatApiError(payload, response.status);
  }
  if (!payload.ok && payload.error) showLocalError(payload.error);
  return payload;
}

async function saveCurrentReportedPosition() {
  const angles = normalizeJointAngles(state.robotState?.reported_angles_deg);
  if (!angles) {
    showLocalError("No reported pose is available to save.");
    return;
  }
  const displayName = window.prompt("Position name", "Current pose");
  if (displayName === null) return;
  const trimmed = displayName.trim();
  if (!trimmed) {
    showLocalError("Position name is required.");
    return;
  }
  const records = ensurePositionLibraryDraft();
  const positionId = uniquePositionId(trimmed);
  records[positionId] = {
    schema_version: 1,
    id: positionId,
    display_name: trimmed,
    type: "joint",
    units: { length: "mm", angle: "deg" },
    source: "operator",
    metadata: planningPoseMetadata(),
    angles_deg: angles,
  };
  renderPositionLibrary();
  await savePositionLibrary({ message: `Saved ${trimmed}` });
}

function positionLibraryRecord(positionId) {
  return (state.positionLibraryDraft && state.positionLibraryDraft[positionId])
    || state.config?.position_library?.positions?.[positionId]
    || null;
}

function waypointFromPositionRecord(record) {
  if (!record) return null;
  if (positionRecordKind(record) === "joint") {
    return { type: "joint", mode: "joint", angles_deg: (record.angles_deg || []).map(Number) };
  }
  return { type: "cartesian", mode: record.preferred_motion_mode || record.motion_mode || "joint", target: record.target || record };
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

function formatPoint(point = {}, decimals = 1) {
  if (!point) return "-";
  return `x ${format(point.x_mm ?? point.x, decimals)}, y ${format(point.y_mm ?? point.y, decimals)}, z ${format(point.z_mm ?? point.z, decimals)}`;
}

function renderModelTruthSummary() {
  if (!elements.modelTruthSummary) return;
  const truth = state.config?.model_truth || {};
  const tool = truth.active_tool || {};
  const chain = Array.isArray(truth.transform_chain) ? truth.transform_chain : [];
  const conventions = Array.isArray(truth.joint_conventions) ? truth.joint_conventions : [];
  const fk = state.robotState?.fk || truth.current_fk || {};
  const tcpPoint = fk.tcp_frame?.origin || fk;
  const flangePoint = fk.flange_frame?.origin || fk.wrist_frame;
  const axisZ = fk.tcp_frame?.axes?.z;
  const currentFrameHtml = `
    <div class="model-truth-current">
      <div class="log-line"><span>Current TCP</span><code>${formatPoint(tcpPoint)}</code></div>
      <div class="log-line"><span>Flange F4</span><code>${formatPoint(flangePoint)}</code></div>
      <div class="log-line"><span>Tool +Z</span><code>${axisZ ? `x ${format(axisZ.x, 3)}, y ${format(axisZ.y, 3)}, z ${format(axisZ.z, 3)}` : "toggle Frames for viewport axes"}</code></div>
    </div>
  `;
  elements.modelTruthSummary.innerHTML = `
    <div class="log-line"><span>Active tool</span><code>${escapeHtml(tool.label || tool.name || "-")} (${escapeHtml(tool.type || "generic")}) TCP ${formatPoint(tool.tcp_offset_mm || {}, 1)} mm</code></div>
    <div class="log-line"><span>Workspace plane Z</span><code>${format(truth.measurement_reference?.workspace_plane_z_mm, 2)} mm in robot base</code></div>
    <div class="log-line"><span>Tool mapping</span><code>tool +Z -> local DH +X; command correction does not change FK</code></div>
    ${currentFrameHtml}
    <div class="model-truth-chain">
      ${chain.map((step, index) => `
        <div class="model-truth-step">
          <strong>${index + 1}. ${escapeHtml(step.label || step.id)}</strong>
          <span>${escapeHtml(step.notes || "")}</span>
        </div>
      `).join("")}
    </div>
    <div class="dh-table-wrap model-truth-table">
      <table class="dh-grid dh-grid-readonly">
        <thead>
          <tr>
            <th>Joint</th>
            <th>Actuator mapping</th>
            <th>DH theta mapping</th>
            <th>Home</th>
          </tr>
        </thead>
        <tbody>
          ${conventions.map((row) => `
            <tr>
              <td><strong>J${row.joint} ${escapeHtml(row.name)}</strong></td>
              <td><code>zero ${format(row.actuator_mapping?.zero_offset_deg, 2)}, sign ${Number(row.actuator_mapping?.direction_sign) === -1 ? "-1" : "+1"}</code></td>
              <td><code>theta = q*${Number(row.dh_model_mapping?.direction_sign) === -1 ? "-1" : "+1"} + ${format(row.dh_model_mapping?.zero_offset_deg, 2)} + ${format(row.dh_model_mapping?.theta_offset_deg, 2)}</code></td>
              <td><code>${format(row.mechanical_home_deg, 2)} deg</code></td>
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

function activeToolDimensionsValidated() {
  const tools = state.config?.tools || {};
  const active = tools.active || state.robotState?.active_tool || "gripper";
  const preset = tools.presets?.[active] || {};
  if (Object.prototype.hasOwnProperty.call(preset, "dimensions_validated")) {
    return Boolean(preset.dimensions_validated);
  }
  return Boolean(state.config?.calibration?.tool_dimensions_validated);
}

function renderToolValidationStatus() {
  if (!elements.toolValidationStatus || !elements.toolValidationBtn) return;
  const active = state.config?.tools?.active || state.robotState?.active_tool || "active tool";
  const validated = activeToolDimensionsValidated();
  elements.toolValidationStatus.textContent = validated
    ? `${active} TCP dimensions are physically validated.`
    : `${active} TCP dimensions are not physically validated.`;
  elements.toolValidationStatus.classList.toggle("ready", validated);
  elements.toolValidationStatus.classList.toggle("warning", !validated);
  elements.toolValidationBtn.textContent = validated
    ? "Clear validation"
    : "Mark measured and verified";
  elements.toolValidationBtn.dataset.validated = validated ? "true" : "false";
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
  renderToolValidationStatus();
}

async function setActiveToolDimensionsValidation() {
  const currentlyValidated = elements.toolValidationBtn?.dataset.validated === "true";
  elements.toolValidationBtn.disabled = true;
  try {
    if (!currentlyValidated && state.settingsDirtyScopes.has("tooling")) {
      const saved = await saveCalibration();
      if (!saved.ok) return;
    }
    const payload = await postJson("/api/tools/validation", {
      validated: !currentlyValidated,
    });
    if (payload.ok && payload.config) applyConfig(payload.config);
    if (payload.state) renderState(payload.state);
    if (!payload.ok) showLocalError(payload.error || "Tool validation could not be updated.");
  } finally {
    elements.toolValidationBtn.disabled = false;
    renderToolValidationStatus();
  }
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

function rememberEncoderLiveSample(robotState = state.robotState) {
  if (!robotState) return;
  const evidence = robotState.encoder_evidence?.[1] || {};
  const estimated = Number(robotState.estimated_angles_deg?.[1] ?? robotState.reported_angles_deg?.[1]);
  const measured = Number(evidence.measured_angle_deg);
  const raw = Number(evidence.raw_angle_deg);
  if (![estimated, measured, raw].some(Number.isFinite)) return;
  const stamp = Number(robotState.updated_at ?? Date.now() / 1000);
  const last = state.encoderLiveSamples[state.encoderLiveSamples.length - 1];
  if (last && Math.abs(last.stamp - stamp) < 1e-6) return;
  state.encoderLiveSamples.push({
    stamp,
    estimated: Number.isFinite(estimated) ? estimated : null,
    measured: Number.isFinite(measured) ? measured : null,
    raw: Number.isFinite(raw) ? raw : null,
    fresh: Boolean(evidence.fresh),
    error: Number.isFinite(measured) && Number.isFinite(estimated) ? measured - estimated : null,
  });
  if (state.encoderLiveSamples.length > 120) {
    state.encoderLiveSamples.splice(0, state.encoderLiveSamples.length - 120);
  }
}

function encoderEvidenceHealthText(evidence = {}) {
  if (!evidence || Object.keys(evidence).length === 0) return "unknown";
  if (evidence.fresh) return "fresh";
  if (evidence.valid) return evidence.health || "stale";
  if (evidence.sensor_valid) return evidence.calibration_validated ? "sensor valid" : "uncalibrated";
  if (evidence.raw_angle_deg != null) return evidence.health || "raw only";
  return evidence.health || (evidence.sensor_available ? "invalid" : "unavailable");
}

function encoderMeasuredText(evidence = {}, { precision = 1, rawFallback = true } = {}) {
  const health = encoderEvidenceHealthText(evidence);
  if (evidence?.measured_angle_deg != null) {
    return `${format(evidence.measured_angle_deg, precision)} deg (${health})`;
  }
  if (rawFallback && evidence?.raw_angle_deg != null) {
    return `raw ${format(evidence.raw_angle_deg, precision)} deg (${health})`;
  }
  return health;
}

function encoderRawText(evidence = {}, { precision = 2 } = {}) {
  if (evidence?.raw_angle_deg == null) return "unavailable";
  return `${format(evidence.raw_angle_deg, precision)} deg raw${evidence.raw_count == null ? "" : ` / ${evidence.raw_count} count`}`;
}

function encoderSampleQualityText(evidence = {}) {
  const parts = [
    encoderEvidenceHealthText(evidence),
    evidence?.age_ms == null ? null : `${evidence.age_ms} ms old`,
    evidence?.noise_deg == null ? null : `noise ${format(evidence.noise_deg, 3)} deg`,
    evidence?.consecutive_valid_samples == null
      ? null
      : `${evidence.consecutive_valid_samples}/${evidence.required_health_samples || 1} stable`,
  ].filter(Boolean);
  return parts.join(" / ") || "unknown";
}

function svgPoint(value, minValue, maxValue, lowPixel, highPixel) {
  if (!Number.isFinite(value)) return null;
  const span = Math.max(1e-9, maxValue - minValue);
  return highPixel - ((value - minValue) / span) * (highPixel - lowPixel);
}

function svgPolyline(points, xForIndex, yForValue) {
  return points
    .map((point, index) => {
      const y = yForValue(point);
      if (y == null) return "";
      return `${format(xForIndex(index), 1)},${format(y, 1)}`;
    })
    .filter(Boolean)
    .join(" ");
}

function renderEncoderLiveChart(robotState = state.robotState) {
  rememberEncoderLiveSample(robotState);
  const samples = state.encoderLiveSamples.filter((sample) => sample.estimated != null || sample.measured != null);
  if (samples.length < 2) {
    return `<div class="encoder-chart empty-state compact-empty">Live chart will appear after a few encoder status updates.</div>`;
  }
  const values = samples.flatMap((sample) => [sample.estimated, sample.measured]).filter(Number.isFinite);
  const minValue = Math.min(...values) - 1;
  const maxValue = Math.max(...values) + 1;
  const width = 460;
  const height = 170;
  const pad = 24;
  const xForIndex = (index) => pad + (index / Math.max(1, samples.length - 1)) * (width - pad * 2);
  const yFor = (field) => (sample) => svgPoint(sample[field], minValue, maxValue, pad, height - pad);
  const measuredPoints = svgPolyline(samples, xForIndex, yFor("measured"));
  const estimatedPoints = svgPolyline(samples, xForIndex, yFor("estimated"));
  const latest = samples[samples.length - 1];
  return `
    <div class="encoder-chart">
      <div class="encoder-chart-title">
        <strong>Live shoulder readback</strong>
        <span>estimate ${latest.estimated == null ? "-" : `${format(latest.estimated, 2)}°`} · measured ${latest.measured == null ? "-" : `${format(latest.measured, 2)}°`} · error ${latest.error == null ? "-" : `${format(latest.error, 2)}°`}</span>
      </div>
      <svg viewBox="0 0 ${width} ${height}" role="img" aria-label="Live shoulder encoder diagnostic chart">
        <line x1="${pad}" y1="${height - pad}" x2="${width - pad}" y2="${height - pad}" class="chart-axis" />
        <line x1="${pad}" y1="${pad}" x2="${pad}" y2="${height - pad}" class="chart-axis" />
        <text x="${pad}" y="14" class="chart-label">${format(maxValue, 1)}°</text>
        <text x="${pad}" y="${height - 5}" class="chart-label">${format(minValue, 1)}°</text>
        <polyline points="${estimatedPoints}" class="chart-line chart-estimated" />
        <polyline points="${measuredPoints}" class="chart-line chart-measured" />
      </svg>
      <div class="encoder-chart-legend">
        <span><i class="legend-estimated"></i>software estimate</span>
        <span><i class="legend-measured"></i>encoder measured</span>
      </div>
    </div>
  `;
}

function renderEncoderFitChart(validation) {
  const points = (validation?.fit_points || []).filter((point) =>
    Number.isFinite(Number(point.joint_angle_deg)) && Number.isFinite(Number(point.predicted_joint_deg))
  );
  if (points.length < 2) {
    return `<div class="encoder-chart empty-state compact-empty">Calibration fit chart appears after two or more fit samples.</div>`;
  }
  const width = 460;
  const height = 170;
  const pad = 26;
  const values = points.flatMap((point) => [Number(point.joint_angle_deg), Number(point.predicted_joint_deg)]);
  const minValue = Math.min(...values) - 1;
  const maxValue = Math.max(...values) + 1;
  const xFor = (value) => pad + ((value - minValue) / Math.max(1e-9, maxValue - minValue)) * (width - pad * 2);
  const yFor = (value) => svgPoint(value, minValue, maxValue, pad, height - pad);
  const circles = points.map((point) => {
    const x = xFor(Number(point.joint_angle_deg));
    const y = yFor(Number(point.predicted_joint_deg));
    const cls = Math.abs(Number(point.error_deg || 0)) > Number(validation?.residual_limit_deg || 1) ? "chart-point warn" : "chart-point";
    return `<circle cx="${format(x, 1)}" cy="${format(y, 1)}" r="3.5" class="${cls}"><title>actual ${format(point.joint_angle_deg, 2)}°, calibrated ${format(point.predicted_joint_deg, 2)}°, error ${format(point.error_deg, 3)}°</title></circle>`;
  }).join("");
  return `
    <div class="encoder-chart">
      <div class="encoder-chart-title">
        <strong>Calibration fit</strong>
        <span>actual known angle vs calibrated readback</span>
      </div>
      <svg viewBox="0 0 ${width} ${height}" role="img" aria-label="Encoder calibration fit chart">
        <line x1="${pad}" y1="${height - pad}" x2="${width - pad}" y2="${height - pad}" class="chart-axis" />
        <line x1="${pad}" y1="${pad}" x2="${pad}" y2="${height - pad}" class="chart-axis" />
        <line x1="${xFor(minValue)}" y1="${yFor(minValue)}" x2="${xFor(maxValue)}" y2="${yFor(maxValue)}" class="chart-ideal" />
        ${circles}
      </svg>
      <div class="encoder-chart-legend">
        <span><i class="legend-ideal"></i>perfect calibration</span>
        <span><i class="legend-measured"></i>fit samples</span>
      </div>
    </div>
  `;
}

function renderBacklashChart(result = state.encoderBacklashResult) {
  const samples = (result?.samples || []).filter((sample) => Number.isFinite(Number(sample.measured_angle_deg)));
  if (!result || samples.length < 2) {
    return `<div class="encoder-chart empty-state compact-empty">Run backlash check to compare the same target approached from both directions.</div>`;
  }
  const center = Number(result.center_joint_angle_deg);
  const width = 460;
  const height = 150;
  const pad = 26;
  const values = samples.map((sample) => Number(sample.measured_angle_deg)).concat([center]);
  const minValue = Math.min(...values) - 1;
  const maxValue = Math.max(...values) + 1;
  const yFor = (value) => svgPoint(value, minValue, maxValue, pad, height - pad);
  const bars = samples.map((sample, index) => {
    const x = pad + 34 + index * 42;
    const y = yFor(Number(sample.measured_angle_deg));
    const cls = Number(sample.approach_direction) === 1 ? "chart-bar below" : "chart-bar above";
    return `<line x1="${x}" y1="${height - pad}" x2="${x}" y2="${format(y, 1)}" class="${cls}"><title>${escapeHtml(sample.label || "")}: ${format(sample.measured_angle_deg, 3)}°</title></line>`;
  }).join("");
  return `
    <div class="encoder-chart">
      <div class="encoder-chart-title">
        <strong>Backlash check</strong>
        <span>${format(result.backlash_estimate_deg, 2)}° branch separation at ${format(center, 1)}°</span>
      </div>
      <svg viewBox="0 0 ${width} ${height}" role="img" aria-label="Shoulder backlash check chart">
        <line x1="${pad}" y1="${height - pad}" x2="${width - pad}" y2="${height - pad}" class="chart-axis" />
        <line x1="${pad}" y1="${format(yFor(center), 1)}" x2="${width - pad}" y2="${format(yFor(center), 1)}" class="chart-ideal" />
        ${bars}
      </svg>
      <div class="encoder-chart-legend">
        <span><i class="legend-below"></i>from below</span>
        <span><i class="legend-above"></i>from above</span>
      </div>
    </div>
  `;
}

function renderEncoderStatus(robotState = state.robotState) {
  const status = $("#encoderReadbackStatus");
  const actionStatus = $("#encoderActionStatus");
  const sessionStatus = $("#encoderCalibrationSessionStatus");
  if (actionStatus) {
    const dirty = state.settingsDirtyScopes.size > 0;
    const message = state.encoderCalibrationMessage
      || (dirty
        ? "Unsaved settings are present. Save and sync before calibration, backlash check, or correction."
        : "Ready. Fast path: set one known shoulder angle, quick-calibrate, then run backlash check.");
    const lowered = message.toLowerCase();
    const tone = lowered.includes("failed")
      || lowered.includes("error")
      || lowered.includes("could not")
      || lowered.includes("requires")
      || lowered.includes("must")
      || lowered.includes("first")
      || lowered.includes("unknown")
      || lowered.includes("invalid")
      || lowered.includes("not ")
      ? "warn-callout"
      : lowered.includes("saved")
        || lowered.includes("running")
        || lowered.includes("queued")
        || lowered.includes("captured")
        || lowered.includes("enabled")
        ? "ok-callout"
        : "muted-callout";
    actionStatus.innerHTML = `
      <div class="encoder-callout ${tone}">
        <strong>Encoder workflow status</strong>
        <span>${escapeHtml(message)}</span>
      </div>
    `;
  }
  if (status && robotState) {
    const evidence = robotState.encoder_evidence?.[1] || {};
    const mismatch = robotState.encoder_mismatch || {};
    const correction = robotState.correction_state || {};
    const encoders = state.config?.encoders || {};
    const axis = (encoders.axes || []).find((item) => Number(item.joint) === 2) || {};
    const readbackEnabled = Boolean(encoders.enabled && axis.enabled);
    const measured = evidence.measured_angle_deg == null
      ? "not calibrated"
      : `${format(evidence.measured_angle_deg, 3)} deg shoulder (${evidence.fresh ? "fresh" : evidence.health || "not authoritative"})`;
    const raw = evidence.raw_angle_deg == null
      ? "unavailable"
      : `${format(evidence.raw_angle_deg, 3)} deg raw magnet angle${evidence.raw_count == null ? "" : ` (${evidence.raw_count}/16383 count)`}`;
    const rawHealth = evidence.sensor_valid
      ? `valid sample${evidence.consecutive_valid_samples == null ? "" : `, ${evidence.consecutive_valid_samples}/${evidence.required_health_samples || 1} stable`}`
      : evidence.sensor_available
        ? "invalid sample"
        : readbackEnabled
          ? "no sensor response"
          : "readback disabled";
    const planning = robotState.estimated_angles_deg?.[1] == null
      ? "unknown"
      : `${format(robotState.estimated_angles_deg[1], 3)} deg (${robotState.joint_authority?.[1] || "planning estimate"})`;
    const calibratedState = axis.calibration_validated
      ? `calibrated ${axis.calibration_model || "linear"}${axis.backlash_estimate_deg == null ? "" : `, backlash ${format(axis.backlash_estimate_deg, 2)} deg`}${axis.calibration_id ? ` (${axis.calibration_id})` : ""}`
      : "not calibrated";
    const correctionGate = mismatch.correction_status
      ? `${mismatch.correction_status}${mismatch.correction_skip_reason ? `: ${mismatch.correction_skip_reason}` : ""}`
      : "not evaluated";
    const correctionLimits = mismatch.correction_deadband_deg == null && mismatch.correction_max_delta_deg == null
      ? "not configured"
      : `deadband ${format(mismatch.correction_deadband_deg ?? 0, 2)} deg / max ${format(mismatch.correction_max_delta_deg ?? 0, 2)} deg`;
    status.innerHTML = `
      <div class="encoder-callout ${readbackEnabled ? "" : "muted-callout"}">
        <strong>${readbackEnabled ? "Shoulder encoder readback" : "Shoulder encoder disabled"}</strong>
        <span>Raw AS5048A angle is only sensor evidence. It does not move the 3D arm or make the full robot pose known.</span>
      </div>
      <div class="log-line"><span>Planning shoulder</span><code>${planning}</code></div>
      <div class="log-line"><span>Raw sensor</span><code>${raw}</code></div>
      <div class="log-line"><span>Raw health</span><code>${rawHealth}</code></div>
      <div class="log-line"><span>Calibrated shoulder</span><code>${measured}</code></div>
      <div class="log-line"><span>Calibration</span><code>${calibratedState}</code></div>
      <div class="log-line"><span>Freshness / noise</span><code>${evidence.age_ms == null ? "-" : `${evidence.age_ms} ms`} / ${evidence.noise_deg == null ? "-" : `${format(evidence.noise_deg, 3)} deg`}</code></div>
      <div class="log-line"><span>Diagnostic flags</span><code>${(evidence.flags || []).join(", ") || "none"}</code></div>
      <div class="log-line"><span>Post-move mismatch</span><code>${mismatch.status || "not checked"}${mismatch.error_deg == null ? "" : ` / ${format(mismatch.error_deg, 3)} deg`}</code></div>
      <div class="log-line"><span>Correction gate</span><code>${escapeHtml(correctionGate)}</code></div>
      <div class="log-line"><span>Correction limits</span><code>${escapeHtml(correctionLimits)}</code></div>
      <div class="log-line"><span>Correction transaction</span><code>${correction.state || "idle"}${correction.transaction_id && correction.transaction_id !== "none" ? ` / ${correction.transaction_id}` : ""}</code></div>
      ${renderEncoderLiveChart(robotState)}
    `;
  }
  if (sessionStatus) {
    const session = state.encoderCalibrationSession;
    const validation = state.encoderCalibrationValidation;
    const samples = session?.samples || [];
    const fitSampleCount = validation?.fit_sample_count ?? samples.filter((sample) => sample.use_for_fit !== false).length;
    const rawSpan = validation?.raw_span_deg;
    const jointSpan = validation?.joint_span_deg;
    const rawNeed = validation?.minimum_raw_span_deg ?? 1;
    const jointNeed = validation?.minimum_joint_span_deg ?? 2;
    const messages = [
      state.encoderCalibrationMessage,
      ...(validation?.errors || []),
      ...(validation?.warnings || []),
    ].filter(Boolean);
    const sampleRows = samples.length
      ? samples.map((sample, index) => `
          <div class="encoder-sample-row">
            <span>${index + 1}</span>
            <code>${format(sample.joint_angle_deg, 3)} deg shoulder</code>
            <code>${format(sample.raw_angle_deg, 3)} deg raw${sample.raw_count == null ? "" : ` / ${sample.raw_count}`}${sample.use_for_fit === false ? " / check only" : ""}</code>
          </div>
        `).join("")
      : `<div class="empty-state compact-empty">No samples yet. Start captures reference 1 at the known angle above.</div>`;
    const spanText = fitSampleCount >= 2
      ? `raw ${format(rawSpan, 3)} / ${format(rawNeed, 3)} deg, shoulder ${format(jointSpan, 3)} / ${format(jointNeed, 3)} deg`
      : `need two references; move shoulder at least ${format(jointNeed, 1)} deg and raw sensor at least ${format(rawNeed, 1)} deg`;
    const localizedBacklash = validation?.localized_backlash_estimate_deg ?? validation?.localized_backlash_estimate_raw_deg;
    const backlashText = validation?.backlash_estimate_deg != null
      ? `${format(validation.backlash_estimate_deg, 3)} deg constant branch separation`
      : localizedBacklash != null
        ? `${format(localizedBacklash, 3)} ${validation?.localized_backlash_estimate_deg == null ? "raw " : ""}deg localized near ${format(validation.localized_backlash_at_joint_deg, 2)} deg`
        : "no paired approach samples";
    const fitText = validation?.ok
      ? `valid ${validation.fit_model || "linear"}${validation.calibration_map_point_count ? `, map ${validation.calibration_map_point_count} pts` : ""}, sign ${validation.direction_sign}, turns ${format(validation.sensor_turns_per_joint_turn, 6)}, residual ${format(validation.max_residual_deg, 3)} deg`
      : "not yet valid";
    const sweep = session?.sweep || null;
    const approachText = sweep?.final_approach_direction === -1
      ? "from above / decreasing"
      : sweep?.final_approach_direction === 1
        ? "from below / increasing"
        : "mixed/manual";
    const sweepStatus = sweep
      ? `
        <div class="log-line"><span>Assisted sweep</span><code>${escapeHtml(sweep.status || "queued")} ${sweep.completed ?? 0}/${sweep.total ?? 0}${sweep.current_target_deg == null ? "" : ` @ ${format(sweep.current_target_deg, 2)} deg`}</code></div>
        <div class="log-line"><span>Sweep approach</span><code>${escapeHtml(approachText)}${sweep.preload_deg == null ? "" : `, preload ${format(sweep.preload_deg, 2)} deg`}</code></div>
        <div class="log-line"><span>Sweep note</span><code>${escapeHtml(sweep.error || sweep.note || "captures stopped samples only")}</code></div>
      `
      : "";
    const nextStep = !session
      ? "Enter the real shoulder angle, then Start + capture reference 1."
      : encoderSweepIsActive(session)
        ? "Sweep is running. Keep clear of the arm; samples are captured only after each settled move."
        : samples.length < 2
        ? "Move to a second known shoulder angle, disarm, enter that angle, then Capture reference; or run the assisted sweep while armed."
        : validation?.ok
          ? "Fit is valid. Commit calibration if the sign/scale and physical setup are correct."
          : "Fit is not valid yet. Move farther or fix the known shoulder angle values.";
    sessionStatus.innerHTML = session
      ? `
        <div class="encoder-callout ${validation?.ok ? "ok-callout" : "warn-callout"}">
          <strong>${validation?.ok ? "Calibration fit valid" : "Calibration in progress"}</strong>
          <span>${escapeHtml(nextStep)}</span>
        </div>
        <div class="log-line"><span>Session</span><code>${session.id}</code></div>
        <div class="log-line"><span>Samples</span><code>${samples.length} total / ${fitSampleCount} fit</code></div>
        ${sweepStatus}
        <div class="log-line"><span>Motion span</span><code>${spanText}</code></div>
        <div class="log-line"><span>Fit</span><code>${fitText}</code></div>
        <div class="log-line"><span>Backlash</span><code>${backlashText}</code></div>
        ${renderEncoderFitChart(validation)}
        <div class="encoder-sample-list">${sampleRows}</div>
        <div class="log-line"><span>Notes</span><code>${escapeHtml(messages.join("; ") || "capture at least two references")}</code></div>
      `
      : `
        <div class="encoder-callout muted-callout">
          <strong>No calibration session</strong>
          <span>${escapeHtml(nextStep)}</span>
        </div>
        <div class="log-line"><span>Session</span><code>${state.encoderCalibrationMessage || "not started"}</code></div>
      `;
  }
}

function markEncoderDraftDirty(event = {}) {
  if (
    event.target?.matches?.(
      "[data-encoder-field], [data-encoder-bus-field], [data-encoder-axis-field], [data-encoder-verification-field], [data-encoder-correction-field], [data-encoder-pose-field], #encoderReferenceDescription"
    )
  ) {
    markSettingsDirty(
      "hardware",
      "Encoder hardware, calibration, or policy settings changed. Save while disarmed, then sync the controller."
    );
  }
}

function readEncoderPayload() {
  const settings = clonePlain(state.config?.encoders || {});
  settings.schema_version = 2;
  settings.bus ||= {};
  settings.verification ||= {};
  settings.correction ||= { enabled: false };
  settings.pose_tracking ||= {};
  const savedBus = clonePlain(settings.bus || {});
  const savedVerification = clonePlain(settings.verification || {});
  const savedCorrection = clonePlain(settings.correction || {});
  const savedAxis = (settings.axes || []).find((axis) => Number(axis.joint) === 2) || {};
  const axis = { ...savedAxis, joint: 2, name: "shoulder", sensor: "as5048a" };

  elements.encoderCalibration?.querySelectorAll("[data-encoder-field]").forEach((input) => {
    settings[input.dataset.encoderField] = input.type === "checkbox" ? input.checked : input.value;
  });
  elements.encoderCalibration?.querySelectorAll("[data-encoder-bus-field]").forEach((input) => {
    settings.bus[input.dataset.encoderBusField] = readNumber(input, settings.bus[input.dataset.encoderBusField] ?? 0);
  });
  elements.encoderCalibration?.querySelectorAll("[data-encoder-verification-field]").forEach((input) => {
    const field = input.dataset.encoderVerificationField;
    settings.verification[field] = input.type === "checkbox"
      ? input.checked
      : input.tagName === "SELECT"
        ? input.value
        : readNumber(input, settings.verification[field] ?? 0);
  });
  elements.encoderCalibration?.querySelectorAll("[data-encoder-correction-field]").forEach((input) => {
    const field = input.dataset.encoderCorrectionField;
    settings.correction[field] = input.type === "checkbox"
      ? input.checked
      : input.tagName === "SELECT"
        ? input.value
        : readNumber(input, settings.correction[field] ?? 0);
  });
  elements.encoderCalibration?.querySelectorAll("[data-encoder-pose-field]").forEach((input) => {
    const field = input.dataset.encoderPoseField;
    settings.pose_tracking[field] = input.type === "checkbox"
      ? input.checked
      : input.tagName === "SELECT"
        ? input.value
        : readNumber(input, settings.pose_tracking[field] ?? 0);
  });
  elements.encoderCalibration?.querySelectorAll("[data-encoder-axis-field]").forEach((input) => {
    const field = input.dataset.encoderAxisField;
    axis[field] = input.type === "checkbox"
      ? input.checked
      : input.tagName === "SELECT"
        ? (field === "direction_sign" ? Number(input.value) : input.value)
        : readNumber(input, axis[field] ?? 0);
  });
  axis.reference_description = $("#encoderReferenceDescription")?.value?.trim() || axis.reference_description || "";

  const calibrationFields = [
    "reference_raw_deg",
    "reference_joint_deg",
    "direction_sign",
    "mounting_location",
    "sensor_turns_per_joint_turn",
  ];
  const calibrationChanged = calibrationFields.some((field) => String(axis[field]) !== String(savedAxis[field]));
  const fieldChanged = (current, saved, field) => String(current?.[field] ?? "") !== String(saved?.[field] ?? "");
  const runtimeChanged = [
    "sck_pin",
    "miso_pin",
    "mosi_pin",
    "clock_hz",
    "sample_interval_ms",
  ].some((field) => fieldChanged(settings.bus, savedBus, field));
  const axisRuntimeChanged = [
    "enabled",
    "cs_pin",
    "freshness_timeout_ms",
    "max_noise_deg",
  ].some((field) => fieldChanged(axis, savedAxis, field));
  const correctionRelevantVerificationChanged = [
    "settle_delay_ms",
    "required_stable_samples",
    "require_encoder",
  ].some((field) => fieldChanged(settings.verification, savedVerification, field));
  const verificationChanged = [
    "policy",
    "settle_delay_ms",
    "required_stable_samples",
    "warning_tolerance_deg",
    "fault_tolerance_deg",
    "hysteresis_deg",
    "require_encoder",
  ].some((field) => fieldChanged(settings.verification, savedVerification, field));
  const correctionLimitsChanged = [
    "deadband_deg",
    "max_delta_deg",
    "align_max_delta_deg",
    "joint_limit_margin_deg",
    "speed_deg_s",
    "accel_deg_s2",
    "max_attempts",
  ].some((field) => fieldChanged(settings.correction, savedCorrection, field));
  if (calibrationChanged) {
    axis.calibration_validated = false;
    axis.calibration_id = "";
    axis.calibration_validated_at = null;
  }
  if (calibrationChanged || runtimeChanged || axisRuntimeChanged || correctionRelevantVerificationChanged || correctionLimitsChanged) {
    settings.correction.enabled = false;
    settings.correction.validation_id = "";
  }
  if (!settings.enabled || !axis.enabled) {
    settings.correction.enabled = false;
    settings.pose_tracking.enabled = false;
  }
  settings.axes = [axis];
  settings.mode = settings.correction.enabled
    ? "bounded_correction"
    : settings.verification.policy === "diagnostic"
      ? "diagnostic"
      : "verification";
  return settings;
}

function encoderKnownShoulderAngleInput() {
  return $("#encoderKnownJointAngle");
}

function currentPlanningShoulderAngle() {
  const candidates = [
    state.robotState?.estimated_angles_deg?.[1],
    state.robotState?.reported_angles_deg?.[1],
    state.robotState?.target_angles_deg?.[1],
    state.config?.joints?.[1]?.home_deg,
  ];
  const value = candidates.map(Number).find(Number.isFinite);
  return value == null ? 0 : value;
}

function setEncoderKnownShoulderAngle(value) {
  const input = encoderKnownShoulderAngleInput();
  if (!input) return;
  input.value = format(value, 3);
}

async function setPlanningShoulderToKnownAngle() {
  let knownAngle;
  try {
    knownAngle = readRequiredNumber(encoderKnownShoulderAngleInput(), "Known shoulder angle");
  } catch (error) {
    state.encoderCalibrationMessage = error?.message || String(error);
    renderEncoderStatus();
    return;
  }
  if (state.robotState?.hardware_armed && !state.robotState?.simulation) {
    state.encoderCalibrationMessage = "disarm hardware before Set Pose";
    renderEncoderStatus();
    return;
  }
  const current = normalizeJointAngles(state.robotState?.reported_angles_deg)
    || normalizeJointAngles(state.robotState?.target_angles_deg)
    || normalizeJointAngles(state.config?.joints?.map((joint) => joint.home_deg));
  if (!current) {
    state.encoderCalibrationMessage = "cannot build Set Pose payload from the current displayed pose";
    renderEncoderStatus();
    return;
  }
  const shoulder = state.config?.joints?.[1];
  if (shoulder && (knownAngle < Number(shoulder.min_deg) || knownAngle > Number(shoulder.max_deg))) {
    state.encoderCalibrationMessage = `known shoulder angle must be within ${format(shoulder.min_deg, 1)}..${format(shoulder.max_deg, 1)} deg`;
    renderEncoderStatus();
    return;
  }
  const next = current.slice();
  next[1] = knownAngle;
  const confirmed = window.confirm(
    `Set the software planning pose shoulder to ${format(knownAngle, 3)} deg now?\n\n` +
    "This is an operator Set Pose assertion. It does not come from the encoder, and it keeps the other displayed joints unchanged."
  );
  if (!confirmed) return;
  const payload = await postJson("/api/hardware/setpose", { angles_deg: next });
  state.encoderCalibrationMessage = payload.ok
    ? "planning shoulder set to the known start angle; arm before running the assisted sweep"
    : payload.error || "Set Pose failed";
  if (payload.state) renderState(payload.state);
  renderEncoderStatus();
}

function conservativeEncoderCalibrationMoveSettings() {
  const settings = pathSettings();
  const shoulderIndex = 1;
  const maxSpeed = 8.0;
  const maxAccel = 30.0;
  settings.global_speed_deg_s = Math.min(Number(settings.global_speed_deg_s || maxSpeed), maxSpeed);
  settings.global_accel_deg_s2 = Math.min(Number(settings.global_accel_deg_s2 || maxAccel), maxAccel);
  settings.per_joint_speed_deg_s = Array.isArray(settings.per_joint_speed_deg_s)
    ? settings.per_joint_speed_deg_s.slice()
    : [];
  settings.per_joint_accel_deg_s2 = Array.isArray(settings.per_joint_accel_deg_s2)
    ? settings.per_joint_accel_deg_s2.slice()
    : [];
  settings.per_joint_speed_deg_s[shoulderIndex] = Math.min(
    Number(settings.per_joint_speed_deg_s[shoulderIndex] || maxSpeed),
    maxSpeed
  );
  settings.per_joint_accel_deg_s2[shoulderIndex] = Math.min(
    Number(settings.per_joint_accel_deg_s2[shoulderIndex] || maxAccel),
    maxAccel
  );
  return settings;
}

async function moveShoulderForEncoderCalibration() {
  let target;
  try {
    target = readRequiredNumber(encoderKnownShoulderAngleInput(), "Known shoulder angle");
  } catch (error) {
    state.encoderCalibrationMessage = error?.message || String(error);
    renderEncoderStatus();
    return;
  }
  if (!state.robotState?.hardware_armed && !state.robotState?.simulation) {
    state.encoderCalibrationMessage = "arm hardware before the helper move; disarm again before capture";
    renderEncoderStatus();
    return;
  }
  const current = normalizeJointAngles(state.robotState?.target_angles_deg)
    || normalizeJointAngles(state.robotState?.reported_angles_deg)
    || normalizeJointAngles(state.config?.joints?.map((joint) => joint.home_deg));
  if (!current) {
    state.encoderCalibrationMessage = "cannot move shoulder: current planning pose is unknown";
    renderEncoderStatus();
    return;
  }
  const shoulder = state.config?.joints?.[1];
  if (shoulder && (target < Number(shoulder.min_deg) || target > Number(shoulder.max_deg))) {
    state.encoderCalibrationMessage = `known shoulder angle must be within ${format(shoulder.min_deg, 1)}..${format(shoulder.max_deg, 1)} deg`;
    renderEncoderStatus();
    return;
  }
  const next = current.slice();
  next[1] = target;
  state.encoderCalibrationMessage = `moving shoulder to ${format(target, 2)} deg using the normal planner`;
  renderEncoderStatus();
  const payload = await postJson("/api/joints", {
    angles_deg: next,
    settings: conservativeEncoderCalibrationMoveSettings(),
  });
  if (payload.ok) {
    state.encoderCalibrationMessage = "move accepted; wait until idle, disarm, then capture the reference";
    invalidatePendingIkPreview();
    releaseJointControlIntent();
    clearViewPreview();
  } else {
    state.encoderCalibrationMessage = payload.error || "shoulder calibration move failed";
  }
  if (payload.state) renderState(payload.state);
  renderEncoderStatus();
}

function encoderSweepIsActive(session = state.encoderCalibrationSession) {
  const status = session?.sweep?.status;
  return ["queued", "running", "preloading", "moving", "settling", "sampling", "cancel_requested"].includes(status);
}

function scheduleEncoderSweepPoll(sessionId) {
  if (state.encoderSweepPollTimer) window.clearTimeout(state.encoderSweepPollTimer);
  state.encoderSweepPollTimer = window.setTimeout(() => {
    void pollEncoderCalibrationSession(sessionId);
  }, 700);
}

function showEncoderWorkflowMessage(message) {
  state.encoderCalibrationMessage = message;
  renderEncoderStatus();
  $("#encoderActionStatus")?.scrollIntoView({ block: "nearest", behavior: "smooth" });
}

async function pollEncoderCalibrationSession(sessionId = state.encoderCalibrationSession?.id) {
  if (!sessionId) return;
  try {
    const response = await fetch(`/api/encoder/calibration/session/${encodeURIComponent(sessionId)}?t=${Date.now()}`, {
      cache: "no-store",
    });
    const payload = await response.json();
    if (payload.session) state.encoderCalibrationSession = payload.session;
    if (payload.validation) state.encoderCalibrationValidation = payload.validation;
    if (payload.state) renderState(payload.state);
    if (!payload.ok) state.encoderCalibrationMessage = payload.error || "could not refresh assisted sweep status";
    renderEncoderStatus();
    if (payload.ok && encoderSweepIsActive(payload.session)) {
      scheduleEncoderSweepPoll(sessionId);
    }
  } catch (error) {
    state.encoderCalibrationMessage = error?.message || "could not poll assisted sweep";
    renderEncoderStatus();
  }
}

function readEncoderSweepPayload() {
  const start = readRequiredNumber(encoderKnownShoulderAngleInput(), "Known shoulder angle");
  return {
    start_joint_angle_deg: start,
    sweep_min_deg: readRequiredNumber($("#encoderSweepMin"), "Sweep minimum"),
    sweep_max_deg: readRequiredNumber($("#encoderSweepMax"), "Sweep maximum"),
    step_deg: readRequiredNumber($("#encoderSweepStep"), "Sweep step", { min: 1, max: 45 }),
    final_approach_direction: Number($("#encoderSweepApproach")?.value || 1),
    preload_deg: readRequiredNumber($("#encoderSweepPreload"), "Backlash preload", { min: 0, max: 45 }),
    speed_deg_s: readRequiredNumber($("#encoderSweepSpeed"), "Sweep speed", { min: 0.1, max: 8 }),
    accel_deg_s2: 24.0,
    settle_ms: readRequiredNumber($("#encoderSweepSettleMs"), "Settle time", { min: 100, max: 5000 }),
    mounting_location: $("[data-encoder-axis-field='mounting_location']")?.value || "joint_output",
    reference_description: $("#encoderReferenceDescription")?.value?.trim() || "",
    confirm_open_loop_sweep: true,
  };
}

function readQuickEncoderCalibrationPayload() {
  return {
    joint_angle_deg: readRequiredNumber(encoderKnownShoulderAngleInput(), "Known shoulder angle"),
    direction_sign: Number($("#encoderQuickDirection")?.value || $("[data-encoder-axis-field='direction_sign']")?.value || 1),
    sensor_turns_per_joint_turn: 1.0,
    mounting_location: $("[data-encoder-axis-field='mounting_location']")?.value || "joint_output",
    reference_description: $("#encoderReferenceDescription")?.value?.trim() || "",
    confirm_one_to_one_output_mount: true,
  };
}

async function quickCalibrateEncoder() {
  if (state.settingsDirtyScopes.size) {
    const saved = await saveAllSettings();
    if (!saved) return;
  }
  let request;
  try {
    request = readQuickEncoderCalibrationPayload();
  } catch (error) {
    showEncoderWorkflowMessage(error?.message || String(error));
    return;
  }
  state.encoderCalibrationMessage = "quick calibration reading current raw sensor sample";
  renderEncoderStatus();
  const payload = await postJson("/api/encoder/calibration/quick", request);
  if (payload.validation) state.encoderCalibrationValidation = payload.validation;
  if (payload.ok) {
    state.encoderCalibrationSession = null;
    state.encoderCalibrationMessage = "quick encoder calibration saved; save/sync is handled, now run backlash check while armed";
  } else {
    state.encoderCalibrationMessage = payload.error || "quick calibration failed";
  }
  if (payload.config) applyConfig(payload.config);
  if (payload.state) renderState(payload.state);
  renderEncoderStatus();
}

async function runEncoderBacklashCheck() {
  let request;
  try {
    request = {
      center_joint_angle_deg: readRequiredNumber($("#encoderBacklashCenter"), "Backlash center"),
      travel_deg: readRequiredNumber($("#encoderBacklashTravel"), "Backlash travel", { min: 2, max: 30 }),
      repeats: Math.round(readRequiredNumber($("#encoderBacklashRepeats"), "Backlash repeats", { min: 1, max: 5 })),
      speed_deg_s: readRequiredNumber($("#encoderBacklashSpeed"), "Backlash check speed", { min: 0.1, max: 8 }),
      settle_ms: readRequiredNumber($("#encoderBacklashSettleMs"), "Backlash settle time", { min: 100, max: 5000 }),
    };
  } catch (error) {
    showEncoderWorkflowMessage(error?.message || String(error));
    return;
  }
  state.encoderCalibrationMessage = "backlash check running; the shoulder will approach the same angle from both directions";
  renderEncoderStatus();
  const payload = await postJson("/api/encoder/backlash/check", request);
  if (payload.backlash) state.encoderBacklashResult = payload.backlash;
  if (payload.ok) {
    state.encoderCalibrationMessage = `backlash check complete: ${format(payload.backlash?.backlash_estimate_deg, 2)} deg branch separation`;
  } else {
    state.encoderCalibrationMessage = payload.error || "backlash check failed";
  }
  if (payload.state) renderState(payload.state);
  renderEncoderStatus();
}

async function runAssistedEncoderSweep() {
  if (state.settingsDirtyScopes.size) {
    updateSettingsSaveBar({
      mode: "error",
      title: "Save + sync required",
      detail: "The assisted sweep uses live controller encoder pins and limits. Save all settings, then sync the controller while disarmed before running it.",
    });
    showEncoderWorkflowMessage("Save and sync settings first. The assisted sweep must use the active controller encoder pins and limits.");
    return;
  }
  let request;
  try {
    request = readEncoderSweepPayload();
  } catch (error) {
    showEncoderWorkflowMessage(error?.message || String(error));
    return;
  }
  state.encoderCalibrationMessage = "assisted sweep queued";
  renderEncoderStatus();
  const payload = await postJson("/api/encoder/calibration/sweep/start", request);
  if (payload.session) state.encoderCalibrationSession = payload.session;
  if (payload.validation) state.encoderCalibrationValidation = payload.validation;
  if (payload.ok) {
    state.encoderCalibrationMessage = "assisted sweep running; samples are captured after each settled move";
    scheduleEncoderSweepPoll(payload.session?.id);
  } else {
    state.encoderCalibrationMessage = payload.error || "assisted sweep could not start";
  }
  if (payload.state) renderState(payload.state);
  renderEncoderStatus();
}

async function cancelAssistedEncoderSweep() {
  const sessionId = state.encoderCalibrationSession?.id;
  if (!sessionId) {
    state.encoderCalibrationMessage = "no assisted sweep session is active";
    renderEncoderStatus();
    return;
  }
  const payload = await postJson("/api/encoder/calibration/sweep/cancel", { session_id: sessionId });
  if (payload.session) state.encoderCalibrationSession = payload.session;
  if (payload.validation) state.encoderCalibrationValidation = payload.validation;
  state.encoderCalibrationMessage = payload.ok ? "assisted sweep cancellation requested" : payload.error || "could not cancel assisted sweep";
  if (payload.state) renderState(payload.state);
  renderEncoderStatus();
}

async function setEncoderCorrectionPolicy(enabled) {
  if (state.settingsDirtyScopes.size) {
    state.encoderCalibrationMessage = "save and sync settings before changing correction; correction validation must use the active saved encoder calibration";
    renderEncoderStatus();
    return;
  }
  const confirmed = window.confirm(
    enabled
      ? "Enable bounded post-move shoulder correction?\n\nThis is not continuous control. It only runs after eligible moves, uses strict limits, and faults instead of rebasing the software pose if it cannot converge."
      : "Disable bounded shoulder correction?"
  );
  if (!confirmed) return;
  const payload = await postJson("/api/encoder/correction/policy", { enabled, confirm: true });
  if (payload.config) applyConfig(payload.config);
  if (payload.state) renderState(payload.state);
  if (payload.ok) {
    state.encoderCalibrationMessage = enabled
      ? "bounded correction enabled locally; sync the controller while disarmed before relying on it"
      : "bounded correction disabled";
  } else {
    state.encoderCalibrationMessage = payload.error || "could not change correction policy";
  }
  renderEncoderStatus();
}

async function handleEncoderUiAction(action) {
  if (!action) return;
  if (action === "recommended-pins") {
    const values = {
      "[data-encoder-bus-field='sck_pin']": ENCODER_RECOMMENDED_PINS.sck_pin,
      "[data-encoder-bus-field='miso_pin']": ENCODER_RECOMMENDED_PINS.miso_pin,
      "[data-encoder-bus-field='mosi_pin']": ENCODER_RECOMMENDED_PINS.mosi_pin,
      "[data-encoder-axis-field='cs_pin']": ENCODER_RECOMMENDED_PINS.cs_pin,
    };
    Object.entries(values).forEach(([selector, value]) => {
      const input = elements.encoderCalibration?.querySelector(selector);
      if (input) input.value = String(value);
    });
    state.encoderCalibrationMessage = "recommended ESP32-S3 pins applied; save and sync while disarmed";
    markEncoderDraftDirty({ target: elements.encoderCalibration?.querySelector("[data-encoder-bus-field='sck_pin']") });
    renderEncoderStatus();
  } else if (action === "disable-readback") {
    const bus = elements.encoderCalibration?.querySelector("[data-encoder-field='enabled']");
    const axis = elements.encoderCalibration?.querySelector("[data-encoder-axis-field='enabled']");
    if (bus) bus.checked = false;
    if (axis) axis.checked = false;
    state.encoderCalibrationMessage = "encoder readback disabled in the draft; save and sync while disarmed";
    markEncoderDraftDirty({ target: bus || axis });
    renderEncoderStatus();
  } else if (action === "standard-limits") {
    const values = {
      "[data-encoder-bus-field='clock_hz']": ENCODER_STANDARD_LIMITS.clock_hz,
      "[data-encoder-bus-field='sample_interval_ms']": ENCODER_STANDARD_LIMITS.sample_interval_ms,
      "[data-encoder-axis-field='freshness_timeout_ms']": ENCODER_STANDARD_LIMITS.freshness_timeout_ms,
      "[data-encoder-axis-field='max_noise_deg']": ENCODER_STANDARD_LIMITS.max_noise_deg,
      "[data-encoder-verification-field='settle_delay_ms']": ENCODER_STANDARD_LIMITS.settle_delay_ms,
      "[data-encoder-verification-field='required_stable_samples']": ENCODER_STANDARD_LIMITS.required_stable_samples,
      "[data-encoder-verification-field='warning_tolerance_deg']": ENCODER_STANDARD_LIMITS.warning_tolerance_deg,
      "[data-encoder-verification-field='fault_tolerance_deg']": ENCODER_STANDARD_LIMITS.fault_tolerance_deg,
      "[data-encoder-verification-field='hysteresis_deg']": ENCODER_STANDARD_LIMITS.hysteresis_deg,
      "[data-encoder-correction-field='deadband_deg']": ENCODER_STANDARD_LIMITS.correction_deadband_deg,
      "[data-encoder-correction-field='max_delta_deg']": ENCODER_STANDARD_LIMITS.correction_max_delta_deg,
      "[data-encoder-correction-field='align_max_delta_deg']": ENCODER_STANDARD_LIMITS.align_max_delta_deg,
    };
    Object.entries(values).forEach(([selector, value]) => {
      const input = elements.encoderCalibration?.querySelector(selector);
      if (input) input.value = String(value);
    });
    state.encoderCalibrationMessage = "standard diagnostic limits applied; save and sync while disarmed";
    markEncoderDraftDirty({ target: elements.encoderCalibration?.querySelector("[data-encoder-bus-field='clock_hz']") });
    renderEncoderStatus();
  } else if (action === "use-planning-angle") {
    setEncoderKnownShoulderAngle(currentPlanningShoulderAngle());
    state.encoderCalibrationMessage = "known angle field filled from the current planning shoulder estimate; verify it physically before capture";
    renderEncoderStatus();
  } else if (action === "set-known-pose") {
    await setPlanningShoulderToKnownAngle();
  } else if (action === "arm") {
    const payload = await postJson("/api/hardware-arm", { armed: true });
    state.encoderCalibrationMessage = payload.ok
      ? "hardware armed; use Move shoulder, then disarm before capture"
      : payload.error || "could not arm hardware";
    if (payload.state) renderState(payload.state);
    renderEncoderStatus();
  } else if (action === "disarm") {
    const payload = await postJson("/api/hardware-arm", { armed: false });
    state.encoderCalibrationMessage = payload.ok
      ? "hardware disarmed; capture is allowed if the encoder sample is fresh"
      : payload.error || "could not disarm hardware";
    if (payload.state) renderState(payload.state);
    renderEncoderStatus();
  } else if (action === "move-shoulder") {
    await moveShoulderForEncoderCalibration();
  } else if (action === "quick-calibrate") {
    await quickCalibrateEncoder();
  } else if (action === "run-backlash-check") {
    await runEncoderBacklashCheck();
  } else if (action === "run-sweep") {
    await runAssistedEncoderSweep();
  } else if (action === "cancel-sweep") {
    await cancelAssistedEncoderSweep();
  } else if (action === "enable-correction") {
    await setEncoderCorrectionPolicy(true);
  } else if (action === "disable-correction") {
    await setEncoderCorrectionPolicy(false);
  }
}

async function handleEncoderCalibrationAction(action) {
  if (!action) return;
  if (action === "start" && state.settingsDirtyScopes.has("hardware")) {
    state.encoderCalibrationMessage = "save and sync controller before calibration; unsynced encoder pins cannot be used for capture";
    renderEncoderStatus();
    return;
  }
  if (action === "start" && state.settingsDirtyScopes.size) {
    const saved = await saveAllSettings();
    if (!saved) return;
  }
  const sessionId = state.encoderCalibrationSession?.id;
  let payload;
  if (action === "start") {
    let knownJointAngle;
    try {
      knownJointAngle = readRequiredNumber(
        encoderKnownShoulderAngleInput(),
        "Known shoulder angle"
      );
    } catch (error) {
      state.encoderCalibrationMessage = error?.message || String(error);
      renderEncoderStatus();
      return;
    }
    payload = await postJson("/api/encoder/calibration/start", {
      mounting_location: $("[data-encoder-axis-field='mounting_location']")?.value || "joint_output",
      reference_description: $("#encoderReferenceDescription")?.value?.trim() || "",
      joint_angle_deg: knownJointAngle,
      capture_initial: true,
    });
  } else if (action === "capture") {
    if (!sessionId) {
      state.encoderCalibrationMessage = "start + capture reference 1 first";
      renderEncoderStatus();
      return;
    }
    let knownJointAngle;
    try {
      knownJointAngle = readRequiredNumber(
        $("#encoderKnownJointAngle"),
        "Known shoulder angle"
      );
    } catch (error) {
      state.encoderCalibrationMessage = error?.message || String(error);
      renderEncoderStatus();
      return;
    }
    payload = await postJson("/api/encoder/calibration/sample", {
      session_id: sessionId,
      joint_angle_deg: knownJointAngle,
      label: `reference ${(state.encoderCalibrationSession?.samples?.length || 0) + 1}`,
    });
  } else if (action === "validate") {
    if (!sessionId) {
      state.encoderCalibrationMessage = "start a calibration session first";
      renderEncoderStatus();
      return;
    }
    payload = await postJson("/api/encoder/calibration/validate", { session_id: sessionId });
  } else if (action === "commit") {
    if (!sessionId || !window.confirm("Commit this reversible shoulder encoder calibration to robot.local.yaml?")) return;
    payload = await postJson("/api/encoder/calibration/commit", { session_id: sessionId, confirm: true });
  }
  if (!payload) return;
  if (payload.session) state.encoderCalibrationSession = payload.session;
  if (payload.validation) state.encoderCalibrationValidation = payload.validation;
  if (payload.ok) {
    if (action === "start") {
      state.encoderCalibrationMessage = "reference 1 captured; move to a second known shoulder angle";
    } else if (action === "capture") {
      state.encoderCalibrationMessage = "reference captured";
    } else if (action === "validate") {
      state.encoderCalibrationMessage = payload.validation?.ok ? "calibration fit is valid" : "calibration fit is not valid yet";
    } else if (action === "commit") {
      state.encoderCalibrationMessage = "shoulder encoder calibration saved";
    } else {
      state.encoderCalibrationMessage = "";
    }
  } else {
    state.encoderCalibrationMessage = payload.error || "encoder calibration action failed";
  }
  if (payload.config) {
    state.encoderCalibrationSession = null;
    applyConfig(payload.config);
  }
  if (payload.state) renderState(payload.state);
  renderEncoderStatus();
}

function buildCalibrationEditors() {
  buildGeometryPresetEditor();
  refreshDerivedModelDraft();
  renderModelTruthSummary();

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
    const bus = encoders.bus || {};
    const verification = encoders.verification || {};
    const correction = encoders.correction || {};
    const poseTracking = encoders.pose_tracking || {};
    const axis = (encoders.axes || []).find((item) => Number(item.joint) === 2) || {};
    const shoulderJoint = state.config.joints?.[1] || {};
    const shoulderMin = Number.isFinite(Number(shoulderJoint.min_deg)) ? Number(shoulderJoint.min_deg) : 0;
    const shoulderMax = Number.isFinite(Number(shoulderJoint.max_deg)) ? Number(shoulderJoint.max_deg) : 180;
    const currentShoulder = clamp(currentPlanningShoulderAngle(), shoulderMin, shoulderMax);
    const shoulderRange = Math.max(0, shoulderMax - shoulderMin);
    const defaultSweepPreload = Math.min(ENCODER_BACKLASH_PRELOAD_DEG, Math.max(0, shoulderRange / 4));
    const sweepMaxDefault = shoulderMax;
    let sweepMinDefault = clamp(shoulderMin + defaultSweepPreload, shoulderMin, shoulderMax);
    if (sweepMaxDefault - sweepMinDefault > 180) sweepMinDefault = sweepMaxDefault - 180;
    if (sweepMinDefault >= sweepMaxDefault) sweepMinDefault = shoulderMin;
    const backlashTravelDefault = Math.min(10, Math.max(2, shoulderRange / 8 || 2));
    const backlashCenterDefault = clamp(
      currentShoulder,
      shoulderMin + backlashTravelDefault,
      shoulderMax - backlashTravelDefault
    );
    const correctionStatus = correction.enabled
      ? `enabled (${correction.validation_id || "validation record missing"})`
      : "disabled";
    elements.encoderCalibration.innerHTML = `
      <div class="encoder-dashboard">
      <div class="encoder-top-row">
        <div id="encoderReadbackStatus" class="encoder-status-grid"></div>
        <div id="encoderActionStatus" class="encoder-action-status"></div>
      </div>
      <p class="field-help encoder-field-help">Normal setup needs readback enablement, the four SPI pins, one known start pose, and the assisted sweep. Timing and correction limits are advanced bench settings.</p>
      <div class="encoder-simple-card">
        <div class="encoder-simple-header">
          <strong>Basic setup</strong>
          <span>Recommended ESP32-S3 pins: SCK 12, MISO 13, MOSI 14, CS 15. Save and sync while disarmed after changing these.</span>
        </div>
        <div class="hardware-grid encoder-config-grid">
          <label class="toggle-label compact-toggle">
            <input data-encoder-field="enabled" type="checkbox" ${encoders.enabled ? "checked" : ""} />
            <span>Enable encoder readback</span>
          </label>
          <label class="toggle-label compact-toggle">
            <input data-encoder-axis-field="enabled" type="checkbox" ${axis.enabled ? "checked" : ""} />
            <span>Use shoulder AS5048A</span>
          </label>
          <label class="toggle-label compact-toggle">
            <input data-encoder-pose-field="enabled" type="checkbox" ${poseTracking.enabled !== false ? "checked" : ""} />
            <span>Use encoder as shoulder pose while idle</span>
          </label>
          <label>SPI SCK GPIO
            <input data-encoder-bus-field="sck_pin" type="number" step="1" value="${bus.sck_pin ?? ENCODER_RECOMMENDED_PINS.sck_pin}" />
          </label>
          <label>SPI MISO GPIO
            <input data-encoder-bus-field="miso_pin" type="number" step="1" value="${bus.miso_pin ?? ENCODER_RECOMMENDED_PINS.miso_pin}" />
          </label>
          <label>SPI MOSI GPIO
            <input data-encoder-bus-field="mosi_pin" type="number" step="1" value="${bus.mosi_pin ?? ENCODER_RECOMMENDED_PINS.mosi_pin}" />
          </label>
          <label>Shoulder CS GPIO
            <input data-encoder-axis-field="cs_pin" type="number" step="1" value="${axis.cs_pin ?? ENCODER_RECOMMENDED_PINS.cs_pin}" />
          </label>
          <label>Post-move check
            <select data-encoder-verification-field="policy">
              <option value="diagnostic" ${verification.policy === "diagnostic" ? "selected" : ""}>Diagnostic only</option>
              <option value="warning" ${verification.policy === "warning" ? "selected" : ""}>Warn on mismatch</option>
              <option value="fault" ${verification.policy === "fault" ? "selected" : ""}>Fault on mismatch</option>
            </select>
          </label>
          <label>Mounting
            <select data-encoder-axis-field="mounting_location">
              <option value="joint_output" ${axis.mounting_location === "joint_output" ? "selected" : ""}>Joint output</option>
              <option value="gearbox_input" ${axis.mounting_location === "gearbox_input" ? "selected" : ""}>Gearbox input (diagnostic only)</option>
              <option value="motor_shaft" ${axis.mounting_location === "motor_shaft" ? "selected" : ""}>Motor shaft (diagnostic only)</option>
            </select>
          </label>
        </div>
        <div class="button-row">
          <button type="button" class="ghost" data-encoder-ui-action="recommended-pins">Use recommended pins</button>
          <button type="button" class="ghost" data-encoder-ui-action="standard-limits">Restore standard limits</button>
          <button type="button" class="danger ghost" data-encoder-ui-action="disable-readback">Disable encoder readback</button>
        </div>
      </div>
      <div class="path-summary encoder-calibration-guide encoder-primary-workflow">
        <strong>Quick shoulder encoder setup</strong>
        <p>Fast path: put the shoulder on a known mark, enter that angle, disarm, then quick-calibrate. This sets the raw encoder offset. Then arm and run backlash check to measure how much the output moves differently when approached from each direction.</p>
        <div class="encoder-workflow-grid">
          <div class="encoder-form-panel">
            <strong>1. Quick calibrate offset</strong>
            <label>Known shoulder angle deg
              <input id="encoderKnownJointAngle" type="number" step="0.001" value="${format(currentPlanningShoulderAngle(), 3)}" />
            </label>
            <label>Encoder direction
              <select id="encoderQuickDirection">
                <option value="1" ${Number(axis.direction_sign) !== -1 ? "selected" : ""}>Normal: raw increases with shoulder</option>
                <option value="-1" ${Number(axis.direction_sign) === -1 ? "selected" : ""}>Inverted: raw decreases with shoulder</option>
              </select>
            </label>
            <label>Mounting reference
              <input id="encoderReferenceDescription" type="text" value="${escapeHtml(axis.reference_description || "")}" placeholder="Fixture, mark, or mechanical datum" />
            </label>
            <div class="button-row encoder-button-row">
              <button type="button" class="ghost" data-encoder-ui-action="use-planning-angle">Use planning angle</button>
              <button type="button" class="ghost" data-encoder-ui-action="set-known-pose">Set Pose to known angle</button>
              <button type="button" class="primary" data-encoder-ui-action="quick-calibrate">Quick calibrate</button>
            </div>
          </div>
          <div class="encoder-form-panel">
            <strong>2. Measure backlash</strong>
            <p class="field-help">The check approaches the same center angle from below and above, then compares the calibrated output encoder readings.</p>
            <div class="encoder-range-grid">
              <label>Center deg
                <input id="encoderBacklashCenter" type="number" step="0.1" value="${format(backlashCenterDefault, 1)}" />
              </label>
              <label>Travel each side deg
                <input id="encoderBacklashTravel" type="number" min="2" max="30" step="0.5" value="${format(backlashTravelDefault, 1)}" />
              </label>
              <label>Repeats
                <input id="encoderBacklashRepeats" type="number" min="1" max="5" step="1" value="1" />
              </label>
              <label>Speed deg/s
                <input id="encoderBacklashSpeed" type="number" min="0.1" max="8" step="0.1" value="6" />
              </label>
              <label>Settle ms
                <input id="encoderBacklashSettleMs" type="number" min="100" max="5000" step="50" value="350" />
              </label>
            </div>
            <div class="button-row encoder-button-row">
              <button type="button" class="ghost" data-encoder-ui-action="arm">Arm</button>
              <button type="button" class="primary" data-encoder-ui-action="run-backlash-check">Run backlash check</button>
              <button type="button" class="ghost" data-encoder-ui-action="disarm">Disarm</button>
            </div>
            ${renderBacklashChart()}
          </div>
        </div>
        <div id="encoderCalibrationSessionStatus" class="path-summary encoder-session-status"></div>
      </div>
      <details class="advanced-block encoder-sweep-block">
        <summary>
          <span>Optional calibration sweep</span>
          <small>Use only when quick calibration needs range verification or nonlinear mapping</small>
        </summary>
        <div class="advanced-block-body">
          <p class="field-help">The sweep preloads backlash once, then captures fit samples from one final approach direction. It is for validation/refinement, not the normal required setup.</p>
          <div class="encoder-range-grid">
            <label>Min deg
              <input id="encoderSweepMin" type="number" step="0.1" value="${format(sweepMinDefault, 1)}" />
            </label>
            <label>Max deg
              <input id="encoderSweepMax" type="number" step="0.1" value="${format(sweepMaxDefault, 1)}" />
            </label>
            <label>Step deg
              <input id="encoderSweepStep" type="number" min="1" max="45" step="1" value="15" />
            </label>
            <label>Final approach
              <select id="encoderSweepApproach">
                <option value="1" selected>From below / increasing</option>
                <option value="-1">From above / decreasing</option>
              </select>
            </label>
            <label>Preload deg
              <input id="encoderSweepPreload" type="number" min="0" max="45" step="0.5" value="${format(defaultSweepPreload, 1)}" />
            </label>
              <label>Speed deg/s
                <input id="encoderSweepSpeed" type="number" min="0.1" max="8" step="0.1" value="6" />
              </label>
              <label>Settle ms
                <input id="encoderSweepSettleMs" type="number" min="100" max="5000" step="50" value="350" />
              </label>
            </div>
            <p class="field-help">Default is increasing-angle calibration. Keep the first sampled point at least the preload amount away from the lower shoulder limit, or choose decreasing approach and keep the top end away from the upper limit.</p>
          <div class="button-row encoder-action-strip">
            <button type="button" class="ghost" data-encoder-ui-action="arm">Arm</button>
            <button type="button" class="primary" data-encoder-ui-action="run-sweep">Run assisted sweep</button>
            <button type="button" class="danger ghost" data-encoder-ui-action="cancel-sweep">Cancel sweep</button>
            <button type="button" class="ghost" data-encoder-ui-action="disarm">Disarm</button>
          </div>
        </div>
      </details>
      <div class="path-summary encoder-correction-guide">
        <strong>Use encoder after moves</strong>
        <p>Best-value mode for this robot: while idle, use the calibrated shoulder encoder as the shoulder planning/estimated angle. During actual moves, motion stays open-loop; after motion settles, optional bounded correction can still clean up residual shoulder error.</p>
        <div class="button-row encoder-action-strip">
          <button type="button" class="primary" data-encoder-ui-action="enable-correction">Validate + enable post-move correction</button>
          <button type="button" class="danger ghost" data-encoder-ui-action="disable-correction">Disable correction</button>
        </div>
        <div class="log-line"><span>Pose tracking</span><code>${poseTracking.enabled !== false ? "idle shoulder estimate follows fresh calibrated encoder" : "disabled"}</code></div>
        <div class="log-line"><span>Correction</span><code>${escapeHtml(correctionStatus)}</code></div>
        <div class="log-line"><span>Applies to</span><code>manual endpoint joint moves and Go Home after motion settles</code></div>
        <div class="log-line"><span>Auto correction trigger</span><code>above ${format(correction.deadband_deg ?? ENCODER_STANDARD_LIMITS.correction_deadband_deg, 2)} deg, up to ${format(correction.max_delta_deg ?? ENCODER_STANDARD_LIMITS.correction_max_delta_deg, 2)} deg</code></div>
        <div class="log-line"><span>Align cap</span><code>first move up to ${format(correction.align_max_delta_deg ?? ENCODER_STANDARD_LIMITS.align_max_delta_deg, 2)} deg, then cleanup up to ${format(correction.max_delta_deg ?? ENCODER_STANDARD_LIMITS.correction_max_delta_deg, 2)} deg</code></div>
      </div>
      <details class="advanced-block encoder-advanced-block">
        <summary>
          <span>Advanced encoder settings</span>
          <small>Timing, health thresholds, fitted calibration values, and correction gate</small>
        </summary>
        <div class="advanced-block-body">
          <div class="hardware-grid encoder-config-grid">
            <label>SPI clock Hz
              <input data-encoder-bus-field="clock_hz" type="number" min="1" step="1000" value="${bus.clock_hz ?? ENCODER_STANDARD_LIMITS.clock_hz}" />
            </label>
            <label>Sample interval ms
              <input data-encoder-bus-field="sample_interval_ms" type="number" min="10" step="10" value="${bus.sample_interval_ms ?? ENCODER_STANDARD_LIMITS.sample_interval_ms}" />
            </label>
            <label>Freshness timeout ms
              <input data-encoder-axis-field="freshness_timeout_ms" type="number" min="1" step="10" value="${axis.freshness_timeout_ms ?? ENCODER_STANDARD_LIMITS.freshness_timeout_ms}" />
            </label>
            <label>Maximum noise deg
              <input data-encoder-axis-field="max_noise_deg" type="number" min="0" step="0.01" value="${axis.max_noise_deg ?? ENCODER_STANDARD_LIMITS.max_noise_deg}" />
            </label>
            <label>Settle delay ms
              <input data-encoder-verification-field="settle_delay_ms" type="number" min="0" step="10" value="${verification.settle_delay_ms ?? ENCODER_STANDARD_LIMITS.settle_delay_ms}" />
            </label>
            <label>Stable samples
              <input data-encoder-verification-field="required_stable_samples" type="number" min="1" step="1" value="${verification.required_stable_samples ?? ENCODER_STANDARD_LIMITS.required_stable_samples}" />
            </label>
            <label>Warning threshold deg
              <input data-encoder-verification-field="warning_tolerance_deg" type="number" min="0.01" step="0.1" value="${verification.warning_tolerance_deg ?? ENCODER_STANDARD_LIMITS.warning_tolerance_deg}" />
            </label>
            <label>Fault threshold deg (fault only)
              <input data-encoder-verification-field="fault_tolerance_deg" type="number" min="0.01" step="0.1" value="${verification.fault_tolerance_deg ?? ENCODER_STANDARD_LIMITS.fault_tolerance_deg}" />
            </label>
            <label>Hysteresis deg
              <input data-encoder-verification-field="hysteresis_deg" type="number" min="0" step="0.05" value="${verification.hysteresis_deg ?? ENCODER_STANDARD_LIMITS.hysteresis_deg}" />
            </label>
            <label>Pose tracking mode
              <select data-encoder-pose-field="mode">
                <option value="idle" ${(poseTracking.mode || "idle") === "idle" ? "selected" : ""}>Idle or stopped</option>
                <option value="disarmed_idle" ${poseTracking.mode === "disarmed_idle" ? "selected" : ""}>Disarmed idle only</option>
              </select>
            </label>
            <label>Pose tracking min update deg
              <input data-encoder-pose-field="min_update_delta_deg" type="number" min="0" step="0.01" value="${poseTracking.min_update_delta_deg ?? ENCODER_STANDARD_LIMITS.pose_tracking_min_update_delta_deg}" />
            </label>
            <label>Pose tracking max jump deg
              <input data-encoder-pose-field="max_jump_deg" type="number" min="0.01" step="1" value="${poseTracking.max_jump_deg ?? ENCODER_STANDARD_LIMITS.pose_tracking_max_jump_deg}" />
            </label>
            <label class="toggle-label compact-toggle">
              <input data-encoder-pose-field="set_shoulder_known" type="checkbox" ${poseTracking.set_shoulder_known !== false ? "checked" : ""} />
              <span>Measured shoulder counts as known shoulder</span>
            </label>
            <label>Reference raw deg
              <input data-encoder-axis-field="reference_raw_deg" type="number" step="0.001" value="${axis.reference_raw_deg ?? 0}" />
            </label>
            <label>Reference shoulder deg
              <input data-encoder-axis-field="reference_joint_deg" type="number" step="0.001" value="${axis.reference_joint_deg ?? 0}" />
            </label>
            <label>Direction
              <select data-encoder-axis-field="direction_sign">
                <option value="1" ${Number(axis.direction_sign) !== -1 ? "selected" : ""}>Normal (+)</option>
                <option value="-1" ${Number(axis.direction_sign) === -1 ? "selected" : ""}>Inverted (-)</option>
              </select>
            </label>
            <label>Sensor turns / joint turn
              <input data-encoder-axis-field="sensor_turns_per_joint_turn" type="number" min="0.000001" step="0.0001" value="${axis.sensor_turns_per_joint_turn ?? 1}" />
            </label>
            <label class="toggle-label compact-toggle">
              <input data-encoder-verification-field="require_encoder" type="checkbox" ${verification.require_encoder ? "checked" : ""} />
              <span>Fault if readback is unavailable</span>
            </label>
            <label>Correction deadband deg
              <input data-encoder-correction-field="deadband_deg" type="number" min="0" step="0.05" value="${correction.deadband_deg ?? ENCODER_STANDARD_LIMITS.correction_deadband_deg}" />
            </label>
            <label>Correction max delta deg (movement cap)
              <input data-encoder-correction-field="max_delta_deg" type="number" min="0.01" step="0.05" value="${correction.max_delta_deg ?? ENCODER_STANDARD_LIMITS.correction_max_delta_deg}" />
            </label>
            <label>Align max total deg
              <input data-encoder-correction-field="align_max_delta_deg" type="number" min="0.01" step="0.5" value="${correction.align_max_delta_deg ?? ENCODER_STANDARD_LIMITS.align_max_delta_deg}" />
            </label>
            <label>Correction speed deg/s
              <input data-encoder-correction-field="speed_deg_s" type="number" min="0.1" step="0.1" value="${correction.speed_deg_s ?? 2}" />
            </label>
            <label>Correction attempts
              <input data-encoder-correction-field="max_attempts" type="number" min="1" step="1" value="${correction.max_attempts ?? 2}" />
            </label>
            <label>Correction validation ID
              <input value="${correction.validation_id || "none"}" readonly />
            </label>
          </div>
          <div class="encoder-callout muted-callout">
            <strong>Correction is enabled by validation, not by editing raw config.</strong>
            <span>Use the buttons below after calibration. Correction remains bounded post-move only; it is never continuous closed-loop control.</span>
          </div>
        </div>
      </details>
      <details class="advanced-block encoder-manual-block">
        <summary>
          <span>Manual two-reference calibration</span>
          <small>Fallback workflow when you do not want the app to move the shoulder automatically</small>
        </summary>
        <div class="advanced-block-body">
          <p class="field-help">Manual capture is disarmed-only. Move the shoulder yourself, enter the real shoulder angle above, then capture each reference.</p>
          <div class="button-row encoder-action-strip">
            <button type="button" data-encoder-ui-action="move-shoulder">Move shoulder to angle</button>
          </div>
        <div class="button-row">
          <button type="button" class="primary" data-encoder-action="start">Start + capture reference 1</button>
          <button type="button" data-encoder-action="capture">Capture reference</button>
          <button type="button" data-encoder-action="validate">Validate</button>
          <button type="button" data-encoder-action="commit">Commit calibration</button>
        </div>
        </div>
      </details>
      </div>
    `;
    renderEncoderStatus();
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
  const capabilities = robotState.controller_capabilities || {};
  const controllerText = robotState.simulation
    ? "simulation"
    : capabilities.raw
      ? `protocol ${capabilities.protocol || "?"}, encoder config ${capabilities.encoder_config ? "supported" : "unsupported"}`
      : "not advertised";
  elements.hardwareStatus.innerHTML = `
    <div class="log-line"><span>Coverage</span><code>${robotState.hardware_mode || "simulated"} (${robotState.hardware_enabled_axes || "0000"})</code></div>
    <div class="log-line"><span>Sync</span><code>${robotState.config_sync_status || "unknown"}</code></div>
    <div class="log-line"><span>Controller</span><code>${controllerText}</code></div>
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
  if (elements.taskSummary) elements.taskSummary.innerHTML = "";
  if (elements.taskPlanPreview) elements.taskPlanPreview.innerHTML = "";
  if (elements.taskPreviewFeedback && reason && !state.taskPreviewPending) {
    elements.taskPreviewFeedback.className = "task-preview-feedback";
    elements.taskPreviewFeedback.textContent = reason;
  }
  state.view?.setTaskPreview?.(null);
}

function setTaskPreviewFeedback(mode, message) {
  if (!elements.taskPreviewFeedback) return;
  elements.taskPreviewFeedback.className = `task-preview-feedback ${mode || ""}`.trim();
  elements.taskPreviewFeedback.textContent = message;
}

function invalidateTaskDetections(reason = "Refresh detections before planning") {
  state.latestDetections = [];
  state.taskDetectionsCapturedAt = null;
  state.taskDetectionSnapshotId = null;
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

function savedTaskDestinations() {
  const wrapped = state.config?.task_destinations;
  if (wrapped?.destinations && typeof wrapped.destinations === "object") return clonePlain(wrapped.destinations);
  if (wrapped && typeof wrapped === "object") {
    return clonePlain(Object.fromEntries(
      Object.entries(wrapped).filter(([key, value]) => !["schema_version", "updated_at"].includes(key) && value && typeof value === "object")
    ));
  }
  return clonePlain(state.config?.drop_zones || {});
}

function ensureTaskDrafts() {
  if (!state.config) return;
  if (!state.taskColorProfilesDraft) {
    state.taskColorProfilesDraft = clonePlain(state.config.color_profiles || {});
  }
  if (!state.taskDestinationsDraft) {
    state.taskDestinationsDraft = savedTaskDestinations();
  }
}

function taskColorProfiles() {
  ensureTaskDrafts();
  return state.taskColorProfilesDraft || {};
}

function taskDestinations() {
  ensureTaskDrafts();
  return state.taskDestinationsDraft || {};
}

function positionLibraryTaskDestinations() {
  const records = state.config?.position_library?.positions || {};
  return Object.fromEntries(
    Object.entries(records).map(([positionId, record]) => [
      positionId,
      {
        id: positionId,
        label: positionDisplayName(positionId, record),
        position_id: positionId,
        position_type: positionRecordKind(record),
        source: "position_library",
      },
    ])
  );
}

function availableTaskDestinations() {
  return {
    ...taskDestinations(),
    ...positionLibraryTaskDestinations(),
  };
}

function taskDestinationsForSave() {
  const explicit = taskDestinations();
  const positions = positionLibraryTaskDestinations();
  const referenced = new Set(
    Object.values(taskColorProfiles())
      .map((profile) => String(profile?.drop_zone || ""))
      .filter(Boolean)
  );
  const destinations = {};
  referenced.forEach((destinationId) => {
    if (positions[destinationId]) {
      destinations[destinationId] = {
        id: destinationId,
        label: positions[destinationId].label,
        position_id: destinationId,
      };
    } else if (explicit[destinationId]) {
      destinations[destinationId] = clonePlain(explicit[destinationId]);
    }
  });
  return destinations;
}

function taskDestinationDraftChanged() {
  if (!state.config) return false;
  return (
    JSON.stringify(state.taskColorProfilesDraft || state.config.color_profiles || {}) !== JSON.stringify(state.config.color_profiles || {})
    || state.unsavedColorProfiles.size > 0
  );
}

function ensureDetectedColorDrafts(detections = state.latestDetections, options = {}) {
  ensureTaskDrafts();
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
    renderColorPresetMapping();
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
      },
    ])
  );
}

function taskDraftBlocksRun() {
  const zones = availableTaskDestinations();
  const relevantColors = new Set(
    state.latestDetections.map(detectionColor).filter(Boolean)
  );
  return Object.entries(taskColorProfiles()).some(([color, profile]) => {
    if (!relevantColors.has(color)) return false;
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
    cycle_confirmation: elements.cycleConfirmationSelect?.value || "automatic",
    max_objects: readRequiredNumber(elements.maxObjectsInput, "Max objects", { min: 1, integer: true }),
    filters: {
      min_confidence: readRequiredNumber(elements.minConfidenceInput, "Min confidence", { min: 0, max: 1 }),
      min_area_px: 0,
      include_colors: selectedColorFilters(),
      require_robot_coordinates: true,
    },
    ordering: {
      policy: "nearest_to_home",
      color_priority: [],
    },
    missing_drop_zone_policy: elements.missingDropzonePolicySelect?.value || "error",
    unknown_color_policy: elements.unknownColorPolicySelect?.value || "ignore",
    placement_policy: "fixed",
    pickup_z_mm: readRequiredNumber(elements.pickupZInput, "Pickup Z", { min: 0 }),
    dropoff_z_mm: readRequiredNumber(elements.dropoffZInput, "Dropoff Z", { min: 0 }),
    approach_clearance_mm: readRequiredNumber(elements.approachClearanceInput, "Pickup clearance", { min: 0 }),
    drop_approach_clearance_mm: readRequiredNumber(elements.dropApproachClearanceInput, "Drop clearance", { min: 0 }),
    orientation_policy: elements.orientationPolicySelect?.value || "prefer_downward",
    downward_phi_deg: -100,
    pickup_preferred_phi_deg: -100,
    drop_preferred_phi_deg: -100,
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
    object_profiles: {},
    color_profile_overrides: colorProfileOverridesPayload(),
    task_destination_overrides: {
      schema_version: 1,
      destinations: taskDestinationsForSave(),
    },
    _has_unsaved_color_profiles: false,
  };
}

function syncTaskOrientationControls() {
  const fixed = elements.orientationPolicySelect?.value === "fixed";
  document.querySelectorAll("[data-fixed-phi-field]").forEach((label) => {
    label.hidden = !fixed;
    const input = label.querySelector("input");
    if (input) input.disabled = !fixed;
  });
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
  setValue(elements.cycleConfirmationSelect, defaults.cycle_confirmation || "automatic");
  setValue(elements.objectSelectionPolicySelect, "nearest_to_home");
  setValue(elements.maxObjectsInput, defaults.max_objects ?? 10);
  setValue(elements.minConfidenceInput, filters.min_confidence ?? 0);
  setValue(elements.colorPriorityInput, "");
  setValue(elements.missingDropzonePolicySelect, defaults.missing_drop_zone_policy || "error");
  setValue(elements.unknownColorPolicySelect, defaults.unknown_color_policy || "ignore");
  setValue(elements.placementPolicySelect, "fixed");
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
    elements.objectProfilesInput.value = "";
  }
  syncTaskOrientationControls();
}

function renderSetupChecklist() {
  if (!elements.taskSetupChecklist) return;
  const camera = state.config?.camera || {};
  const calibration = state.config?.calibration || {};
  const robot = state.robotState || {};
  const profiles = state.config?.color_profiles || {};
  const zones = availableTaskDestinations();
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
    [
      "Tool/TCP dimensions",
      activeToolDimensionsValidated() || Boolean(robot.simulation),
      activeToolDimensionsValidated()
        ? "physically validated"
        : robot.simulation
          ? "not physically validated; simulation can continue"
          : "not physically validated",
    ],
    ["Safe pose", !state.config?.validation?.named_position_errors?.safe, state.config?.validation?.named_position_errors?.safe?.join("; ") || "valid"],
    ["Task destinations", missingZones.length === 0, missingZones.length ? `missing ${missingZones.join(", ")}` : `${Object.keys(zones).length} configured`],
  ];
  elements.taskSetupChecklist.innerHTML = checks.map(([label, ok, detail]) => `
    <div class="setup-check ${ok ? "ok" : "warn"}">
      <span>${ok ? "✓" : "!"}</span>
      <div><strong>${label}</strong><small>${detail}</small></div>
    </div>
  `).join("");
}

function colorStatusBadge(color, profile, zones) {
  if (profile?.drop_zone && !zones[profile.drop_zone]) {
    return `<span class="missing-badge">Missing/deleted</span>`;
  }
  if (!profile?.drop_zone) {
    return `<span class="missing-badge">Needs destination</span>`;
  }
  return `<span class="saved-badge">Ready</span>`;
}

function zoneOptionsHtml(selected = "") {
  const zones = availableTaskDestinations();
  const options = [`<option value="">Choose Position Library record</option>`];
  if (selected && !zones[selected]) {
    options.push(`<option value="${escapeHtml(selected)}" selected>Missing/deleted: ${escapeHtml(selected)}</option>`);
  }
  Object.keys(zones).sort().forEach((name) => {
    const label = zones[name]?.label || name;
    const sourceSuffix = zones[name]?.source === "position_library" ? "" : " (legacy destination)";
    options.push(`<option value="${escapeHtml(name)}" ${name === selected ? "selected" : ""}>${escapeHtml(label)}${sourceSuffix}</option>`);
  });
  return options.join("");
}

function taskDestinationSummary(profile, zones) {
  const destinationId = profile?.drop_zone || "";
  if (!destinationId) return "no destination assigned";
  const destination = zones[destinationId];
  if (!destination) return `missing/deleted: ${destinationId}`;
  const label = destination.label || destinationId;
  return label === destinationId ? destinationId : `${label} (${destinationId})`;
}

function renderColorPresetMapping() {
  if (!elements.colorPresetMapping || !state.config) return;
  ensureDetectedColorDrafts(state.latestDetections);
  const profiles = taskColorProfiles();
  const zones = availableTaskDestinations();
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
    const destinationSummary = taskDestinationSummary(profile, zones);
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
          <small>${escapeHtml(destinationSummary)}</small>
        </div>
        ${colorStatusBadge(color, profile, zones)}
      </div>
    `;
  }).join("");
}

function updateTaskMappingControls() {
  const dirty = taskDestinationDraftChanged();
  if (elements.saveTaskMappingsBtn) elements.saveTaskMappingsBtn.disabled = !dirty;
  if (elements.discardTaskMappingsBtn) elements.discardTaskMappingsBtn.disabled = !dirty;
}

function applyTaskMappingConfig(config) {
  state.config = config;
  state.taskColorProfilesDraft = clonePlain(config.color_profiles || {});
  state.taskDestinationsDraft = savedTaskDestinations();
  state.unsavedColorProfiles.clear();
  renderColorPresetMapping();
  renderSetupChecklist();
}

async function saveTaskMappingEdits(options = {}) {
  if (!state.config) return { ok: false, error: "configuration not loaded" };
  let payload;
  try {
    payload = await postJson("/api/task-mappings", {
      color_profiles: taskColorProfiles(),
      task_destinations: {
        schema_version: 1,
        destinations: taskDestinationsForSave(),
      },
    });
  } catch (error) {
    payload = {
      ok: false,
      error: `Automatic mapping save failed: ${error.message || error}`,
    };
  }
  if (payload.ok) {
    if (payload.config) applyTaskMappingConfig(payload.config);
    if (payload.state) renderState(payload.state);
    clearSettingsDirty("task_destinations");
    state.unsavedColorProfiles.clear();
  } else {
    if (!options.silent) {
      updateSettingsSaveBar({
        mode: "error",
        title: "Task mappings could not be saved",
        detail: payload.error || "Review the task destinations and color assignments.",
      });
    }
  }
  updateTaskMappingControls();
  return payload;
}

function discardTaskMappingEdits() {
  if (!state.config) return;
  state.taskColorProfilesDraft = clonePlain(state.config.color_profiles || {});
  state.taskDestinationsDraft = savedTaskDestinations();
  state.unsavedColorProfiles.clear();
  clearSettingsDirty("task_destinations");
  invalidateTaskDetections("Task mappings reverted - refresh detections");
  renderColorPresetMapping();
  updateTaskMappingControls();
}

function markTaskDestinationDraftDirty(detail = "Task mappings changed. Save task mappings before running.") {
  invalidateTaskPreview("Destination mapping changed - preview again");
  renderColorPresetMapping();
  updateTaskMappingControls();
  window.clearTimeout(state.taskMappingSaveTimer);
  state.taskMappingSaveTimer = window.setTimeout(() => {
    state.taskMappingSavePromise = saveTaskMappingEdits({ silent: true }).finally(() => {
      state.taskMappingSavePromise = null;
    });
  }, 500);
}

function updateColorProfileDraft(color, patch, detail = "Color destination mapping changed. Save all settings before running.") {
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
  markTaskDestinationDraftDirty(detail);
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
  ensureTaskDrafts();
  renderPositionLibrary();
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
    Object.keys(taskDestinations()).forEach((name) => {
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
  });
  renderColorPresetMapping();
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
  const previousActive = state.config.tools.active || state.robotState?.active_tool || "gripper";
  state.toolSwitchPending = true;
  elements.toolSelect.disabled = true;
  elements.toolStatus.textContent = `Changing to ${active}...`;
  elements.toolStatus.classList.add("warn");
  if (state.config?.tools) state.config.tools.active = active;
  if (state.robotState) state.robotState.active_tool = active;
  renderToolControls(active);
  const tools = state.config?.tools || { active, presets: {} };
  tools.active = active;
  const payload = await postJson("/api/tools", { active, presets: tools.presets || {} });
  state.toolSwitchPending = false;
  elements.toolSelect.disabled = false;
  elements.toolStatus.classList.remove("warn");
  if (payload.ok && payload.config) {
    applyConfig(payload.config);
    elements.toolStatus.textContent = `Active: ${active}`;
  } else {
    state.config.tools.active = previousActive;
    if (state.robotState) state.robotState.active_tool = previousActive;
    renderToolControls(previousActive);
    elements.toolStatus.textContent = payload.error || "Tool change failed";
    elements.toolStatus.classList.add("error");
  }
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

async function refreshDiagnostics(options = {}) {
  if (!elements.diagnosticsDrawer || elements.diagnosticsDrawer.hidden) return;
  const robotState = state.robotState || {};
  let events = null;
  if (options.events !== false) {
    const response = await fetch("/api/events?limit=80");
    const payload = await response.json();
    events = payload.events || [];
  }
  const enc = robotState.encoder_angles_deg || [];
  const err = robotState.encoder_errors_deg || [];
  const shoulderEvidence = robotState.encoder_evidence?.[1] || {};
  const mismatch = robotState.encoder_mismatch || {};
  const correction = robotState.correction_state || {};
  const correctionGate = mismatch.correction_status
    ? `${mismatch.correction_status}${mismatch.correction_skip_reason ? `: ${mismatch.correction_skip_reason}` : ""}`
    : "not evaluated";
  const pending = robotState.pending_motion || {};
  const diagnostics = robotState.motion_diagnostics || {};
  const configChange = robotState.config_change || {};
  const truth = state.config?.model_truth || {};
  const tool = truth.active_tool || {};
  const currentFk = truth.current_fk || robotState.fk || {};
  const currentTcp = currentFk.tcp_frame?.origin || currentFk.tcp || currentFk;
  const currentFlange = currentFk.flange_frame?.origin || robotState.fk?.flange_frame?.origin;
  const angleText = (values) => Array.isArray(values) ? values.map((value) => format(value, 2)).join(", ") : "-";
  const previewRevision = state.latestPreview?.start_pose_revision;
  const activeRunId = diagnostics.run_id || pending.run_id || "";
  const diagnosticMotionSummary = diagnostics.motion_contract
    ? motionContractHtml(diagnostics, null, { includeNotes: true })
    : "";
  elements.diagnosticsSummary.innerHTML = `
    <div class="log-line"><span>Pose source</span><code>${robotState.pose_source || "unknown"}</code></div>
    <div class="log-line"><span>Pose revision</span><code>${robotState.pose_revision ?? 0} (${robotState.pose_known_mask || "0000"})</code></div>
    <div class="log-line"><span>Reported</span><code>${angleText(robotState.reported_angles_deg)}</code></div>
    <div class="log-line"><span>Commanded</span><code>${angleText(robotState.commanded_target_deg || robotState.target_angles_deg)}</code></div>
    <div class="log-line"><span>Active run</span><code>${activeRunId ? `${activeRunId} ${diagnostics.result || pending.status || ""}` : "none"}</code></div>
    <div class="log-line"><span>Run source/mode</span><code>${activeRunId ? `${diagnostics.source || pending.source || "-"} / ${diagnostics.mode || pending.mode || "-"}` : "-"}</code></div>
    <div class="log-line"><span>Run revisions</span><code>${activeRunId ? `start ${pending.start_pose_revision ?? "-"} -> current ${robotState.pose_revision ?? 0}` : "-"}</code></div>
    ${diagnosticMotionSummary}
    <div class="log-line"><span>Draft</span><code>${angleText(state.draftAngles)}</code></div>
    <div class="log-line"><span>Preview start</span><code>${previewRevision == null ? "none" : `revision ${previewRevision}: ${angleText(state.latestPreview?.start_reported_angles_deg)}`}</code></div>
    <div class="log-line"><span>Last rejection/error</span><code>${robotState.last_error || diagnostics.error || "-"}</code></div>
    <div class="log-line"><span>Config impact</span><code>${(configChange.categories || []).join(", ") || "none"}; pose invalidated=${Boolean(configChange.pose_invalidated)}</code></div>
    <div class="log-line"><span>Model chain</span><code>${(truth.transform_chain || []).map((step) => step.id).join(" -> ") || "-"}</code></div>
    <div class="log-line"><span>Active TCP</span><code>${tool.name || "-"} ${formatPoint(tool.tcp_offset_mm || {})} mm</code></div>
    <div class="log-line"><span>FK frames</span><code>flange ${formatPoint(currentFlange)}; TCP ${formatPoint(currentTcp)}</code></div>
    <div class="log-line"><span>Encoders</span><code>${robotState.encoder_available || "0000"}</code></div>
    <div class="log-line"><span>Encoder angles</span><code>${enc.map((value) => value == null ? "-" : format(value, 2)).join(", ")}</code></div>
    <div class="log-line"><span>Encoder errors</span><code>${err.map((value) => value == null ? "-" : format(value, 2)).join(", ")}</code></div>
    <div class="log-line"><span>Measured shoulder live</span><code>${escapeHtml(encoderMeasuredText(shoulderEvidence, { precision: 3 }))}</code></div>
    <div class="log-line"><span>Raw shoulder sensor</span><code>${escapeHtml(encoderRawText(shoulderEvidence, { precision: 3 }))}</code></div>
    <div class="log-line"><span>Shoulder sample quality</span><code>${escapeHtml(encoderSampleQualityText(shoulderEvidence))}</code></div>
    <div class="log-line"><span>Shoulder mismatch</span><code>${mismatch.error_deg == null ? mismatch.status || "not checked" : `${mismatch.status || "checked"} / ${format(mismatch.error_deg, 3)} deg`}</code></div>
    <div class="log-line"><span>Correction gate</span><code>${escapeHtml(correctionGate)}</code></div>
    <div class="log-line"><span>Correction transaction</span><code>${escapeHtml(correction.state || "idle")}${correction.transaction_id && correction.transaction_id !== "none" ? ` / ${escapeHtml(correction.transaction_id)}` : ""}</code></div>
    ${renderEncoderLiveChart(robotState)}
    <div class="log-line"><span>Sync</span><code>${robotState.config_sync_status || "unknown"}</code></div>
  `;
  if (events) {
    elements.eventLog.innerHTML = "";
    events.reverse().forEach((event) => {
      const line = document.createElement("div");
      line.className = "log-line";
      const ts = new Date((event.ts || 0) * 1000).toLocaleTimeString();
      line.innerHTML = `<span>${ts} ${event.source}</span><code>${event.message}</code>`;
      elements.eventLog.appendChild(line);
    });
  }
}

function scheduleDiagnosticsRender() {
  if (!elements.diagnosticsDrawer || elements.diagnosticsDrawer.hidden || state.diagnosticsRenderTimer) return;
  state.diagnosticsRenderTimer = window.setTimeout(() => {
    state.diagnosticsRenderTimer = null;
    refreshDiagnostics({ events: false });
  }, 250);
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
  const record = positionLibraryRecord(name);
  const waypoint = waypointFromPositionRecord(record);
  if (waypoint) return waypoint;
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
  const waypoint = namedPositionWaypoint(name);
  if (!waypoint) return;
  const payload = await postJson("/api/path/go", {
    branch: elements.ikBranchSelect.value,
    settings: pathSettings(),
    waypoints: [waypoint],
  });
  if (payload.ok) {
    invalidatePendingIkPreview();
    releaseJointControlIntent();
    clearViewPreview();
    state.ikUserEdited = false;
    elements.previewStatus.textContent = `Moving to ${positionDisplayName(name, positionLibraryRecord(name) || {})}`;
  } else {
    showLocalError(payload.error || `Could not move to ${name}`);
  }
  if (payload.state) renderState(payload.state);
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
  const bindings = taskPreview.bindings || preview?.task_bindings || {};
  const cameraClearCheck = taskPreview.camera_clear_check || {};
  const cameraClearPosition = taskPreview.camera_clear_position || "-";
  const cameraClearStatus = cameraClearCheck.ok === false
    ? `blocked: ${cameraClearCheck.error || "not plannable"}`
    : cameraClearCheck.ok
      ? `checked${cameraClearCheck.trajectory?.duration_s != null ? `, ${format(cameraClearCheck.trajectory.duration_s)} s` : ""}`
      : "-";
  const calibrationApplied = Array.isArray(preview?.calibration)
    && preview.calibration.some((item) => item.applied);
  const orientation = normalized.orientation_policy === "fixed"
    ? `${format(normalized.pickup_phi_deg)}° / ${format(normalized.drop_phi_deg)}°`
    : normalized.orientation_policy || "-";
  elements.taskSummary.innerHTML = `
    <div class="log-line"><span>Steps</span><code>${steps.length}</code></div>
    <div class="log-line"><span>Moves</span><code>${sequence?.waypoints?.length || 0}</code></div>
    ${motionContractHtml(preview, trajectory)}
    <div class="log-line"><span>TCP correction</span><code>${calibrationApplied ? "enabled for Cartesian task targets" : "not applied"}</code></div>
    <div class="log-line"><span>Strategy</span><code>${taskPreview.strategy || sequence?.strategy || "-"}</code></div>
    <div class="log-line"><span>Between objects</span><code>${normalized.cycle_confirmation === "confirm_each_object" ? "pause for operator" : "automatic"}</code></div>
    <div class="log-line"><span>Objects</span><code>${taskPreview.selected_objects?.length || sequence?.object_count || 0}</code></div>
    <div class="log-line"><span>Camera clear</span><code>${escapeHtml(cameraClearPosition)} / ${escapeHtml(cameraClearStatus)}</code></div>
    <div class="log-line"><span>TCP Z</span><code>pick ${format(normalized.pickup_z_mm)} / drop ${format(normalized.dropoff_z_mm)} mm</code></div>
    <div class="log-line"><span>Orientation</span><code>${orientation}</code></div>
    <div class="log-line"><span>Detection snapshot</span><code>${bindings.detection_snapshot_id || "-"}</code></div>
    <div class="log-line"><span>Active tool</span><code>${taskPreview.active_tool || "-"} / ${taskPreview.tool_type || "-"}</code></div>
    <div class="log-line"><span>Warnings</span><code>${warnings.length ? warnings.join("; ") : "-"}</code></div>
  `;
}

function renderTaskPlanPreview(taskPreview = {}, sequence = {}) {
  if (!elements.taskPlanPreview) return;
  const objects = taskPreview.selected_objects || sequence.objects || [];
  const ignored = taskPreview.ignored_detections || sequence.ignored_detections || [];
  const assigned = taskPreview.assigned_targets || [];
  const modes = taskPreview.motion_modes || {};
  const steps = sequence.steps || [];
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
  elements.taskPlanPreview.insertAdjacentHTML("beforeend", `
    <div class="task-generated-steps">
      <h3>Generated sequence</h3>
      ${steps.map((step, index) => {
        const moveDetail = step.kind === "move"
          ? `${step.target_frame || "-"} · ${step.movement_mode || step.waypoint?.mode || "-"} · z ${step.height_mm == null ? "-" : format(step.height_mm)} mm`
          : `tool action · ${step.action || "-"}`;
        return `<div class="task-generated-step"><span>${index + 1}</span><strong>${escapeHtml(step.label || step.kind || "step")}</strong><code>${escapeHtml(moveDetail)}</code><small>${escapeHtml(step.phase || "-")}</small></div>`;
      }).join("") || `<div class="empty-state">No generated steps.</div>`}
    </div>
  `);
}

async function previewTask() {
  if (state.taskPreviewPending) return;
  state.taskPreviewPending = true;
  state.taskLocalStatusAt = Date.now() / 1000;
  elements.taskStatus.textContent = "Building preview...";
  elements.previewTaskBtn.disabled = true;
  elements.previewTaskBtn.textContent = "Building Preview...";
  elements.previewTaskBtn.setAttribute("aria-busy", "true");
  setTaskPreviewFeedback("working", "Validating detections, generating robot steps, and checking every motion target...");
  window.clearTimeout(state.taskMappingSaveTimer);
  state.taskMappingSaveTimer = null;
  if (taskDestinationDraftChanged()) {
    state.taskMappingSavePromise = saveTaskMappingEdits({ silent: true }).finally(() => {
      state.taskMappingSavePromise = null;
    });
    await state.taskMappingSavePromise;
  } else if (state.taskMappingSavePromise) {
    await state.taskMappingSavePromise;
  }
  const task = "color_sorting";
  if (elements.taskModeSelect) elements.taskModeSelect.value = "color_sorting";
  const selectedIds = [...state.selectedDetectionIds];
  let taskSettings;
  try {
    taskSettings = taskSettingsPayload();
  } catch (error) {
    elements.taskStatus.textContent = error.message;
    invalidateTaskPreview(error.message);
    setTaskPreviewFeedback("error", error.message);
    state.taskPreviewPending = false;
    elements.previewTaskBtn.textContent = "Preview Task";
    elements.previewTaskBtn.removeAttribute("aria-busy");
    updateDisabledState();
    return;
  }
  if (!state.taskDetectionsCapturedAt || !state.latestDetections.length) {
    invalidateTaskPreview("Refresh detections before previewing a task");
    setTaskPreviewFeedback("error", "No current detection snapshot. Return to Detect and refresh detections.");
    state.taskPreviewPending = false;
    elements.previewTaskBtn.textContent = "Preview Task";
    elements.previewTaskBtn.removeAttribute("aria-busy");
    updateDisabledState();
    return;
  }
  let motionSettings;
  try {
    motionSettings = taskPathSettingsPayload();
  } catch (error) {
    invalidateTaskPreview(error.message);
    setTaskPreviewFeedback("error", error.message);
    state.taskPreviewPending = false;
    elements.previewTaskBtn.textContent = "Preview Task";
    elements.previewTaskBtn.removeAttribute("aria-busy");
    updateDisabledState();
    return;
  }
  const request =
    task === "color_sorting"
      ? {
          task,
          detections: state.latestDetections,
          detection_snapshot_id: state.taskDetectionSnapshotId,
          detection_captured_at: state.taskDetectionsCapturedAt,
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
  let payload;
  try {
    payload = await postJson("/api/task/preview", request);
  } catch (error) {
    const message = `Preview request failed: ${error.message || error}`;
    invalidateTaskPreview(message);
    setTaskPreviewFeedback("error", message);
    state.taskPreviewPending = false;
    elements.previewTaskBtn.textContent = "Preview Task";
    elements.previewTaskBtn.removeAttribute("aria-busy");
    updateDisabledState();
    return;
  }
  if (payload.ok) {
    state.taskPreviewId = payload.preview_id;
    state.taskPreviewCreatedAt = Date.now() / 1000;
    state.taskLocalStatusAt = state.taskPreviewCreatedAt;
    renderPreview(payload.preview, { preserveTaskPreview: true });
    state.lastTaskPreview = payload.task_preview || payload.sequence?.task_preview || null;
    renderTaskSummary(payload.sequence, payload.preview);
    renderTaskPlanPreview(state.lastTaskPreview, payload.sequence);
    state.view?.setTaskPreview?.(state.lastTaskPreview);
    elements.executeTaskBtn.disabled = false;
    elements.taskStatus.textContent = "Preview ready";
    setTaskPreviewFeedback(
      "success",
      `Preview ready: ${payload.sequence?.object_count || state.lastTaskPreview?.selected_objects?.length || 0} object(s), ${payload.sequence?.steps?.length || 0} generated steps.`
    );
    state.activeTaskStep = "run";
    renderWorkflowStepper("run");
    renderTaskExecution();
  } else {
    renderPreviewFailure(payload);
    state.lastTaskPreview = payload.task_preview || payload.sequence?.task_preview || null;
    renderTaskPlanPreview(state.lastTaskPreview, payload.sequence);
    state.view?.setTaskPreview?.(state.lastTaskPreview);
    elements.taskStatus.textContent = payload.error || "Task preview failed";
    setTaskPreviewFeedback("error", payload.error || "Task preview failed. Review the generated diagnostics.");
    state.taskLocalStatusAt = Date.now() / 1000;
  }
  state.taskPreviewPending = false;
  elements.previewTaskBtn.textContent = "Preview Task";
  elements.previewTaskBtn.removeAttribute("aria-busy");
  updateDisabledState();
  await refreshDiagnostics();
}

async function executeTask() {
  if (!state.taskPreviewId) return;
  elements.executeTaskBtn.disabled = true;
  elements.taskStatus.textContent = "Starting task...";
  let payload;
  try {
    payload = await postJson("/api/task/execute", { preview_id: state.taskPreviewId });
  } catch (error) {
    elements.taskStatus.textContent = `Could not start task: ${error.message || error}`;
    updateDisabledState();
    return;
  }
  if (payload.ok) {
    releaseJointControlIntent();
    state.previewId = null;
    state.previewAngles = null;
    state.taskPreviewId = null;
    state.taskPreviewCreatedAt = null;
    state.taskDetectionsCapturedAt = null;
    state.taskDetectionSnapshotId = null;
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
  let payload;
  try {
    payload = await postJson("/api/task/stop", {});
  } catch (error) {
    showLocalError(`Could not stop task: ${error.message || error}`);
    return;
  }
  if (payload.state) renderState(payload.state);
  elements.taskStatus.textContent = payload.ok ? "Task stopped" : payload.error || "Stop failed";
  state.taskLocalStatusAt = Date.now() / 1000;
  await refreshDiagnostics();
}

async function continueTask() {
  const runId = state.robotState?.task_execution?.run_id;
  if (!runId) return;
  let payload;
  try {
    payload = await postJson("/api/task/continue", { run_id: runId });
  } catch (error) {
    showLocalError(`Could not continue task: ${error.message || error}`);
    return;
  }
  if (payload.state) renderState(payload.state);
  if (!payload.ok) showLocalError(payload.error || "Task could not continue.");
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

function configuredDetectionMinAreaPx(camera = state.config?.camera || {}) {
  const value = Number(camera?.detection?.min_object_area_px);
  return Number.isFinite(value) ? Math.max(1, value) : 400;
}

function currentDetectionMinAreaPx() {
  const value = Number(state.taskDetectionMinAreaPx);
  return Number.isFinite(value) ? Math.max(1, value) : configuredDetectionMinAreaPx();
}

function syncDetectionTuningControls() {
  if (!elements.detectionMinAreaInput) return;
  const value = currentDetectionMinAreaPx();
  elements.detectionMinAreaInput.value = format(value, 0);
  elements.detectionMinAreaInput.title = `Saved default: ${format(configuredDetectionMinAreaPx(), 0)} px`;
}

function readDetectionMinAreaInput() {
  const raw = String(elements.detectionMinAreaInput?.value ?? "").trim();
  if (!raw) return configuredDetectionMinAreaPx();
  const value = Number(raw);
  return Number.isFinite(value) ? clamp(value, 1, 1000000) : configuredDetectionMinAreaPx();
}

function updateDetectionMinAreaFromInput() {
  const next = readDetectionMinAreaInput();
  const previous = currentDetectionMinAreaPx();
  state.taskDetectionMinAreaPx = next;
  syncDetectionTuningControls();
  if (Math.round(next) !== Math.round(previous)) {
    invalidateTaskDetections("Detection size filter changed - refresh detections");
    if (state.cameraLive) scheduleCameraFrame(0);
  }
}

function visionDetectionTuningRequest() {
  return { min_object_area_px: currentDetectionMinAreaPx() };
}

function applyDetectionTuningPayload(tuning = {}) {
  const value = Number(tuning.min_object_area_px);
  if (!Number.isFinite(value)) return;
  state.taskDetectionMinAreaPx = Math.max(1, value);
  syncDetectionTuningControls();
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
    const payload = await postJson("/api/vision/frame", visionDetectionTuningRequest());
    elements.visionSummary.innerHTML = "";
    if (!payload.ok) {
      elements.visionSummary.innerHTML = `<div class="log-line"><span>Error</span><code>${payload.error || "-"}</code></div>`;
      if (elements.cameraPlaceholder) elements.cameraPlaceholder.hidden = false;
      if (elements.cameraPopupStatus) elements.cameraPopupStatus.textContent = payload.error || "Detection failed";
      return;
    }
    applyDetectionTuningPayload(payload.detection_tuning);
    state.latestDetections = payload.detections || [];
    state.taskDetectionsCapturedAt = payload.captured_at || Date.now() / 1000;
    state.taskDetectionSnapshotId = payload.detection_snapshot_id || null;
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
    const tuningLine = document.createElement("div");
    tuningLine.className = "log-line";
    tuningLine.innerHTML = `<span>Object area</span><code>minimum ${format(currentDetectionMinAreaPx(), 0)} px</code>`;
    elements.visionSummary.appendChild(tuningLine);
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
  if (elements.workspaceMarginInput) {
    elements.workspaceMarginInput.value = format(workspaceAruco.workspace_margin_mm ?? 0, 1);
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
    workspace_margin_mm: Math.max(0, readNumber(elements.workspaceMarginInput, 0)),
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
  const workspaceMargin = Math.max(0, Number(settings.workspace_margin_mm) || 0);
  const bounds = xValues.length && yValues.length
    ? `X ${format(Math.min(...xValues) - workspaceMargin, 1)} to ${format(Math.max(...xValues) + workspaceMargin, 1)} mm | Y ${format(Math.min(...yValues) - workspaceMargin, 1)} to ${format(Math.max(...yValues) + workspaceMargin, 1)} mm | margin ${format(workspaceMargin, 1)} mm`
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
      max_frames: 120,
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

function tcpCorrectionText(result) {
  const coefficients = result?.coefficients || {};
  const zOffset = format(coefficients.z_offset_mm ?? 0, 2);
  if (result?.model_type === "radial_reach_z_offset") {
    return `reach ${format(coefficients.reach_offset_mm ?? 0, 2)} mm; Z ${zOffset} mm`;
  }
  if (result?.model_type === "constant_xyz") {
    const xy = coefficients.xy_offset_mm || [0, 0];
    return `X ${format(xy[0], 2)} mm; Y ${format(xy[1], 2)} mm; Z ${zOffset} mm`;
  }
  if (result?.model_type === "affine_xy_z_offset") {
    const xy = coefficients.xy_offset_mm || [0, 0];
    return `affine XY, offset ${format(xy[0], 2)} / ${format(xy[1], 2)} mm; Z ${zOffset} mm`;
  }
  return "not fitted";
}

function renderTcpCalibrationTargets() {
  if (!elements.tcpCalibrationTargetList) return;
  elements.tcpCalibrationTargetList.innerHTML = "";
  if (!state.tcpCalibrationTargets.length) {
    elements.tcpCalibrationTargetList.innerHTML = `<div class="empty-state">Generate an automatic model-aware target set.</div>`;
    return;
  }
  state.tcpCalibrationTargets.forEach((point, index) => {
    const target = point.intended_target || {};
    const item = document.createElement("div");
    item.className = `program-item ${point.reachable ? "" : "invalid"}`;
    item.innerHTML = `
      <div class="program-title">
        <span>Point ${index + 1} · ${point.recommended_role || "fit"}</span>
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
  const manualResult = result?.source === "manual_offsets";
  const physicalResult = profile.physical_model_result || state.tcpCalibrationPhysicalResult || null;
  const workspace = summary.workspace || {};
  const freshness = summary.freshness || {};
  const activation = summary.activation || {};
  const activationWarnings = activation.warnings || [];
  const coverage = summary.coverage || result?.coverage || {};
  const reference = summary.context?.measurement_reference || state.config?.calibration?.measurement_reference || {};
  const samples = Array.isArray(profile.samples) ? profile.samples : [];
  const enabled = Boolean(summary.enabled);
  elements.tcpCalibrationStatus.textContent = enabled
    ? "Correction enabled"
    : freshness.fresh === false && result
      ? "Stale, disabled"
      : result
        ? manualResult ? "Manual, disabled" : "Fitted, disabled"
        : "Audit required";
  elements.tcpCalibrationStatus.classList.toggle("ready", enabled);
  elements.tcpCalibrationStatus.classList.toggle("warning", !enabled && Boolean(result || freshness.fresh === false));
  elements.tcpCalEnableInput.checked = Boolean(settings.enabled && profile.enabled);
  elements.tcpCalEnableInput.disabled = !activation.eligible;
  elements.tcpCalModelSelect.value = profile.model_type || settings.default_model || "radial_reach_z_offset";
  const coefficients = result?.coefficients || {};
  if (elements.tcpCalManualReachOffsetInput && document.activeElement !== elements.tcpCalManualReachOffsetInput) {
    elements.tcpCalManualReachOffsetInput.value = format(coefficients.reach_offset_mm ?? 0, 2);
  }
  if (elements.tcpCalManualZOffsetInput && document.activeElement !== elements.tcpCalManualZOffsetInput) {
    elements.tcpCalManualZOffsetInput.value = format(coefficients.z_offset_mm ?? 0, 2);
  }
  if (elements.tcpCalWorkspacePlaneZInput && document.activeElement !== elements.tcpCalWorkspacePlaneZInput) {
    elements.tcpCalWorkspacePlaneZInput.value = format(reference.workspace_plane_z_mm ?? 0, 2);
  }
  if (elements.tcpCalMeasuredPointSelect) elements.tcpCalMeasuredPointSelect.value = reference.measured_point || "active_tcp";

  elements.tcpCalibrationWorkspaceStatus.innerHTML = `
    <div class="log-line"><span>Workspace map</span><code>${workspace.calibrated ? "saved" : "not calibrated"}</code></div>
    <div class="log-line"><span>Source</span><code>${workspace.source || "-"}</code></div>
    <div class="log-line"><span>Tool profile</span><code>${summary.active_profile_key || state.config?.tools?.active || "-"}</code></div>
    <div class="log-line"><span>Frame</span><code>robot base XYZ, mm; +Z upward</code></div>
    <div class="log-line"><span>Workspace plane Z</span><code>${format(reference.workspace_plane_z_mm, 2)} mm in robot base</code></div>
  `;
  elements.tcpCalibrationModelStatus.innerHTML = `
    <div class="log-line"><span>Model</span><code>${profile.model_type || "not fitted"}</code></div>
    <div class="log-line"><span>Fit samples</span><code>${samples.filter((sample) => sample.role !== "validation").length}</code></div>
    <div class="log-line"><span>Validation samples</span><code>${samples.filter((sample) => sample.role === "validation").length}</code></div>
    <div class="log-line"><span>Result</span><code>${result?.fit?.status || "not run"}</code></div>
    <div class="log-line"><span>Profile freshness</span><code>${freshness.fresh ? "current" : (freshness.messages || ["not fitted"]).join("; ")}</code></div>
  `;
  elements.tcpCalibrationReferenceStatus.innerHTML = `
    <div class="log-line"><span>Active tool/TCP</span><code>${summary.context?.tool?.tool || "-"} · x ${format(summary.context?.tool?.tcp_offset_mm?.x, 1)}, y ${format(summary.context?.tool?.tcp_offset_mm?.y, 1)}, z ${format(summary.context?.tool?.tcp_offset_mm?.z, 1)} mm</code></div>
    <div class="log-line"><span>Joint authority</span><code>${state.robotState?.simulation ? "simulation" : state.robotState?.encoder_available === "1111" ? "measured encoders" : "estimated/open-loop"}</code></div>
    <div class="log-line"><span>Measured point</span><code>${reference.measured_point || "active_tcp"}</code></div>
    <div class="log-line"><span>Reference check</span><code>${elements.tcpCalReferenceConfirmInput?.checked ? "operator confirmed" : "confirmation required before preview"}</code></div>
  `;

  const fit = result?.fit || summary.fit_quality;
  const validation = result?.validation || summary.validation_quality;
  const diagnostics = result?.diagnostics || [];
  elements.tcpCalibrationMetrics.innerHTML = `
    <div class="log-line"><span>Before correction</span><code>${tcpMetricText(fit?.before)}</code></div>
    <div class="log-line"><span>Correction</span><code>${tcpCorrectionText(result)}</code></div>
    <div class="log-line"><span>Fit residual</span><code>${tcpMetricText(fit?.after_model)} (${fit?.status || "not run"})</code></div>
    <div class="log-line"><span>Validation landing</span><code>${tcpMetricText(validation?.landing)} (${validation?.status || "not run"})</code></div>
    <div class="log-line"><span>Worst fit</span><code>${(fit?.worst_samples || []).slice(0, 3).map((item) => `${item.id}: ${format(item.error_xy_mm, 1)} XY`).join(" | ") || "-"}</code></div>
    <div class="log-line"><span>Diagnostics</span><code>${diagnostics.join(" | ") || "No fitted diagnostics yet."}</code></div>
    <div class="log-line"><span>Coverage</span><code>XY ${format(coverage.xy_span_mm, 1)} mm · Z ${format(coverage.z_span_mm, 1)} mm · Phi ${format(coverage.phi_span_deg, 1)}°</code></div>
    <div class="log-line"><span>Activation gate</span><code>${activation.eligible ? "eligible" : (activation.reasons || ["fit and validate first"]).join("; ")}</code></div>
    <div class="log-line"><span>Manual warnings</span><code>${activationWarnings.join("; ") || "-"}</code></div>
  `;

  if (elements.tcpCalibrationPhysicalMetrics) {
    const parameters = physicalResult?.parameters || [];
    elements.tcpCalibrationPhysicalMetrics.innerHTML = physicalResult
      ? `
        <div class="log-line"><span>Candidate</span><code>${physicalResult.parameter_group || "-"}</code></div>
        <div class="log-line"><span>Parameter deltas</span><code>${parameters.map((item) => `${item.name} ${format(item.delta, 3)} ${item.unit}`).join(" | ") || "-"}</code></div>
        <div class="log-line"><span>Fit before/after</span><code>${tcpMetricText(physicalResult.fit?.before)} → ${tcpMetricText(physicalResult.fit?.after)}</code></div>
        <div class="log-line"><span>Validation before/after</span><code>${tcpMetricText(physicalResult.validation?.before)} → ${tcpMetricText(physicalResult.validation?.after)}</code></div>
        <div class="log-line"><span>Apply gate</span><code>${physicalResult.safe_to_apply ? "accepted" : (physicalResult.apply_blockers || []).join("; ")}</code></div>
      `
      : `<div class="log-line"><span>Physical model</span><code>No candidate fitted.</code></div>`;
  }
  state.tcpCalibrationPhysicalResult = physicalResult;
  if (elements.tcpCalApplyPhysicalBtn) {
    elements.tcpCalApplyPhysicalBtn.disabled = !physicalResult?.safe_to_apply;
  }

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
  elements.tcpCalGenerateBtn.disabled = true;
  elements.tcpCalGenerateBtn.textContent = "Generating...";
  elements.tcpCalibrationMoveStatus.textContent =
    "Selecting safe, informative targets from the active model and workspace...";
  try {
    const payload = await postJson("/api/kinematics-calibration/targets", {
      count: 12,
      validation_stride: 4,
      apply_calibration: false,
    });
    if (!payload.ok) {
      elements.tcpCalibrationMoveStatus.textContent = payload.error || "Automatic target generation failed.";
      return;
    }
    state.tcpCalibrationTargets = payload.points || [];
    renderTcpCalibrationTargets();
    const firstReachable = state.tcpCalibrationTargets.find((point) => point.reachable);
    if (firstReachable) {
      setTcpCalibrationTarget(firstReachable.intended_target);
      elements.tcpCalRoleSelect.value = firstReachable.recommended_role || "fit";
    }
    const coverage = payload.strategy?.coverage || {};
    elements.tcpCalibrationMoveStatus.textContent =
      `${payload.strategy?.message || "Automatic target set generated"} ` +
      `Coverage: X ${format(coverage.x_span_mm, 0)}, Y ${format(coverage.y_span_mm, 0)}, ` +
      `Z ${format(coverage.z_span_mm, 0)} mm, Phi ${format(coverage.phi_span_deg, 0)} deg.`;
  } finally {
    elements.tcpCalGenerateBtn.disabled = false;
    elements.tcpCalGenerateBtn.textContent = "Generate automatic target set";
  }
}

async function previewTcpCalibrationMove(applyCalibration, role) {
  if (!elements.tcpCalReferenceConfirmInput?.checked) {
    showLocalError("Verify and confirm the active TCP point and robot-base Z reference before previewing calibration motion.");
    return;
  }
  if (state.settingsDirtyScopes.has("calibration")) {
    const saved = await saveAllSettings();
    if (!saved) return;
  }
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
    purpose: role === "validation" ? "kinematics_calibration_validation" : "kinematics_calibration_fit",
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
    const workspacePlaneZ = readRequiredNumber(elements.tcpCalWorkspacePlaneZInput, "Workspace plane Z");
    const measuredZ = workspacePlaneZ + surface + offset;
    elements.tcpCalMeasuredZInput.value = format(measuredZ, 3);
    state.tcpCalibrationMeasurementSource.z = {
      type: "touch_off",
      workspace_plane_z_mm: workspacePlaneZ,
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
    preview_id: state.tcpCalibrationMove?.preview_id || null,
    measured_point: elements.tcpCalMeasuredPointSelect?.value || "active_tcp",
    reference_frame: "robot_base",
    approach: {
      direction: elements.tcpCalApproachSelect?.value || "unknown",
      operator_confirmed_reference: Boolean(elements.tcpCalReferenceConfirmInput?.checked),
    },
    notes: "captured after bound calibration workflow move",
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
    enable_after_fit: false,
  });
  if (!payload.ok) {
    elements.tcpCalibrationMetrics.innerHTML = `<div class="log-line"><span>Fit error</span><code>${payload.error || "failed"}</code></div>`;
    return;
  }
  if (payload.config) applyConfig(payload.config);
}

async function saveManualTcpCalibrationOffsets() {
  let reachOffset;
  let zOffset;
  try {
    reachOffset = readRequiredNumber(elements.tcpCalManualReachOffsetInput, "Manual reach offset");
    zOffset = readRequiredNumber(elements.tcpCalManualZOffsetInput, "Manual Z offset");
  } catch (error) {
    showLocalError(error?.message || String(error));
    return;
  }
  elements.tcpCalibrationMetrics.innerHTML = `<div class="log-line"><span>Status</span><code>Saving manual offsets...</code></div>`;
  const payload = await postJson("/api/kinematics-calibration/manual-offsets", {
    reach_offset_mm: reachOffset,
    z_offset_mm: zOffset,
    enabled: true,
  });
  if (!payload.ok) {
    elements.tcpCalibrationMetrics.innerHTML = `<div class="log-line"><span>Manual offset error</span><code>${escapeHtml(payload.error || "failed")}</code></div>`;
    return;
  }
  if (elements.tcpCalModelSelect) elements.tcpCalModelSelect.value = "radial_reach_z_offset";
  if (payload.config) applyConfig(payload.config);
}

async function fitTcpPhysicalModel() {
  elements.tcpCalibrationPhysicalMetrics.innerHTML = `<div class="log-line"><span>Status</span><code>Fitting constrained physical model...</code></div>`;
  const payload = await postJson("/api/kinematics-calibration/physical-model/fit", {
    parameter_group: elements.tcpCalPhysicalModelSelect?.value || "joint_zeros",
  });
  if (!payload.ok) {
    elements.tcpCalibrationPhysicalMetrics.innerHTML = `<div class="log-line"><span>Fit error</span><code>${escapeHtml(payload.error || "failed")}</code></div>`;
    return;
  }
  state.tcpCalibrationPhysicalResult = payload.result;
  if (payload.config) applyConfig(payload.config);
}

async function applyTcpPhysicalModel() {
  const result = state.tcpCalibrationPhysicalResult;
  if (!result?.safe_to_apply) return;
  if (!window.confirm("Apply this accepted physical-model update? This changes DH/geometry, disables residual correction, invalidates previews, and never moves the robot.")) {
    return;
  }
  const payload = await postJson("/api/kinematics-calibration/physical-model/apply", {
    result_id: result.id,
    confirm: true,
  });
  if (!payload.ok) {
    showLocalError(payload.error || "Physical-model update was not applied.");
    return;
  }
  state.tcpCalibrationPhysicalResult = null;
  if (payload.config) applyConfig(payload.config);
  if (payload.state) renderState(payload.state);
  clearViewPreview();
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
          <th>Color</th>
          <th>Conf</th>
          <th>Area</th>
          <th>Robot X/Y</th>
          <th>Eligibility</th>
          <th>Reason</th>
          <th>Action</th>
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
    row.className = eligible ? "" : "invalid";
    row.innerHTML = `
      <td><strong>${label}</strong><small>${detectionId}</small></td>
      <td>${detection.confidence == null ? "-" : format(detection.confidence, 2)}</td>
      <td>${area == null ? "-" : format(area, 0)}</td>
      <td>${robot}</td>
      <td>${eligible ? "eligible" : "ignored"}</td>
      <td>${detection.projection_error || detection.reason || detection.coordinate_source || "-"}</td>
      <td data-detection-action>${waitingRun && candidateIds.has(detectionId) ? `<button type="button" class="ghost compact-action" data-runtime-select="${detectionId}">Pick</button>` : "-"}</td>
    `;
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

function setIkTargetFromRequestedTarget(target = {}) {
  const values = {
    x: target.x_mm,
    y: target.y_mm,
    z: target.z_mm,
    phi: target.phi_deg,
  };
  Object.entries(values).forEach(([key, value]) => {
    if (Number.isFinite(Number(value))) setIkTargetValue(key, Number(value), false);
  });
  state.ikUserEdited = true;
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

function motionContractFrom(source = {}, fallbackTrajectory = null) {
  return source.motion_contract || fallbackTrajectory?.motion_contract || source.trajectory?.motion_contract || null;
}

function motionLimitsFrom(source = {}, fallbackTrajectory = null) {
  return source.limit_summary
    || motionContractFrom(source, fallbackTrajectory)?.limits
    || fallbackTrajectory?.limit_summary
    || source.trajectory?.limit_summary
    || {};
}

function motionModeLabel(contract = {}, trajectory = {}) {
  const mode = contract.path_mode || trajectory.mode || "-";
  if (mode === "linear") return "sampled Cartesian linear";
  if (mode === "joint") return "joint";
  if (mode === "program") return "program";
  if (mode === "cartesian_jog") return "Cartesian jog";
  return mode;
}

function formatLimitList(values = [], unit = "") {
  if (!Array.isArray(values) || !values.length) return "-";
  return values.map((value, index) => `J${index + 1} ${format(value, 1)}${unit}`).join(", ");
}

function formatLimitingConstraint(limits = {}) {
  const constraint = limits.limiting_constraint || {};
  const type = constraint.type || "none";
  if (type === "none") return "none";
  if (constraint.segment_index !== undefined && constraint.segment_index !== null) {
    const label = constraint.segment_label ? ` ${constraint.segment_label}` : "";
    return `step ${Number(constraint.segment_index) + 1}${label}: ${type}`;
  }
  if (constraint.joint_index !== undefined && constraint.joint_index !== null) {
    const name = constraint.joint_name || `J${Number(constraint.joint_index) + 1}`;
    return `${name}: ${type}`;
  }
  return type;
}

function motionContractLines(source = {}, fallbackTrajectory = null, options = {}) {
  const trajectory = fallbackTrajectory || source.trajectory || {};
  const contract = motionContractFrom(source, trajectory) || {};
  const limits = motionLimitsFrom(source, trajectory);
  const command = contract.controller_command || source.controller_command_contract || {};
  const notes = Array.isArray(limits.notes) ? limits.notes : [];
  const waypointCount = contract.waypoint_count ?? trajectory.waypoint_count ?? 0;
  const duration = contract.duration_s ?? trajectory.duration_s;
  const lines = [
    ["Mode", motionModeLabel(contract, trajectory)],
    ["Profile", contract.profile || trajectory.profile || limits.profile || "-"],
    ["Controller", command.command ? `${command.command} (${command.timing_authority || "unknown"})` : "-"],
    ["Duration", Number.isFinite(Number(duration)) ? `${format(duration, 2)} s` : "-"],
    ["Waypoints", waypointCount],
    ["Limiting", formatLimitingConstraint(limits)],
    ["Joint speed", formatLimitList(limits.effective_joint_speed_deg_s || trajectory.speed_limits_deg_s, " deg/s")],
    ["Joint accel", formatLimitList(limits.effective_joint_accel_deg_s2 || trajectory.accel_limits_deg_s2, " deg/s²")],
  ];
  if (limits.tcp_speed_mm_s !== undefined || limits.tcp_accel_mm_s2 !== undefined) {
    lines.push(["TCP jog", `${format(limits.tcp_speed_mm_s, 1)} mm/s, ${format(limits.tcp_accel_mm_s2, 1)} mm/s²`]);
  }
  if (limits.phi_speed_deg_s !== undefined || limits.phi_accel_deg_s2 !== undefined) {
    lines.push(["Phi jog", `${format(limits.phi_speed_deg_s, 1)} deg/s, ${format(limits.phi_accel_deg_s2, 1)} deg/s²`]);
  }
  if (options.includeNotes !== false && notes.length) {
    lines.push(["Notes", notes.join("; ")]);
  }
  if (options.includeNotes !== false && Array.isArray(command.notes) && command.notes.length) {
    lines.push(["Command notes", command.notes.join("; ")]);
  }
  return lines;
}

function motionContractHtml(source = {}, fallbackTrajectory = null, options = {}) {
  return motionContractLines(source, fallbackTrajectory, options)
    .map(([label, value]) => `<div class="log-line"><span>${label}</span><code>${escapeHtml(value)}</code></div>`)
    .join("");
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
    tcp_speed_mm_s: readNumber(elements.tcpSpeedInput, 60),
    phi_speed_deg_s: readNumber(elements.phiSpeedInput, 45),
    tcp_accel_mm_s2: readNumber(elements.tcpAccelInput, 360),
    phi_accel_deg_s2: readNumber(elements.phiAccelInput, 240),
    waypoint_rate_hz: readNumber(elements.waypointRateInput, 12),
    cartesian_step_mm: readNumber(elements.cartesianStepInput, 10),
    planner_type: elements.plannerTypeSelect.value,
    jerk_percent: readNumber(elements.jerkPercentInput, 25),
    blend_percent: readNumber(elements.blendPercentInput, 0),
    per_joint_speed_deg_s: [...document.querySelectorAll(".joint-speed-limit")].map((input) => readNumber(input, 1)),
    per_joint_accel_deg_s2: [...document.querySelectorAll(".joint-accel-limit")].map((input) => readNumber(input, 1)),
  };
}

function syncPlannerControls() {
  const trapezoidSelected = elements.plannerTypeSelect?.value === "trapezoid";
  if (elements.blendPercentInput) {
    elements.blendPercentInput.disabled = !trapezoidSelected;
    elements.blendPercentInput.title = trapezoidSelected
      ? "Ramp fraction used by the trapezoid profile"
      : "Only used by the trapezoid profile";
  }
}

function taskPathSettingsPayload() {
  const settings = pathSettings();
  settings.global_speed_deg_s = readRequiredNumber(elements.globalSpeedInput, "Global speed", { min: 0.001 });
  settings.global_accel_deg_s2 = readRequiredNumber(elements.globalAccelInput, "Global acceleration", { min: 0.001 });
  settings.tcp_speed_mm_s = readRequiredNumber(elements.tcpSpeedInput, "TCP speed", { min: 0.001 });
  settings.phi_speed_deg_s = readRequiredNumber(elements.phiSpeedInput, "Phi speed", { min: 0.001 });
  settings.tcp_accel_mm_s2 = readRequiredNumber(elements.tcpAccelInput, "TCP acceleration", { min: 0.001 });
  settings.phi_accel_deg_s2 = readRequiredNumber(elements.phiAccelInput, "Phi acceleration", { min: 0.001 });
  settings.waypoint_rate_hz = readRequiredNumber(elements.waypointRateInput, "Waypoint rate", { min: 0.001 });
  settings.cartesian_step_mm = readRequiredNumber(elements.cartesianStepInput, "Cartesian step", { min: 0.001 });
  settings.jerk_percent = readRequiredNumber(elements.jerkPercentInput, "Jerk percent", { min: 0, max: 100 });
  settings.blend_percent = readRequiredNumber(elements.blendPercentInput, "Trapezoid ramp", { min: 0, max: 100 });
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

function taskPreviewFocusStep(preview = state.latestPreview) {
  if (!preview?.task_bindings) return null;
  const executionSteps = preview?.trajectory?.execution_steps;
  if (!Array.isArray(executionSteps)) return null;
  return (
    executionSteps.find((step) => String(step?.label || "").trim().toLowerCase() === "pickup") ||
    executionSteps.find((step) => String(step?.label || "").trim().toLowerCase() === "above pickup") ||
    null
  );
}

function previewEndpointAngles(preview = state.latestPreview) {
  const taskFocusWaypoints = taskPreviewFocusStep(preview)?.trajectory?.waypoints;
  const taskFocusEndpoint =
    Array.isArray(taskFocusWaypoints) && taskFocusWaypoints.length
      ? taskFocusWaypoints[taskFocusWaypoints.length - 1]
      : null;
  const taskFocusAngles = normalizeJointAngles(taskFocusEndpoint);
  if (taskFocusAngles) return taskFocusAngles;
  const waypoints = preview?.trajectory?.waypoints;
  const lastWaypoint = Array.isArray(waypoints) && waypoints.length ? waypoints[waypoints.length - 1] : null;
  return normalizeJointAngles(lastWaypoint) || normalizeJointAngles(preview?.ik?.selected?.angles_deg);
}

function robotStateUsesEncoderTrackedShoulder(robotState = state.robotState) {
  if (!robotState) return false;
  const tracking = state.config?.encoders?.pose_tracking || {};
  const measuredShoulder = robotState.measured_angles_deg?.[1] ?? robotState.encoder_evidence?.[1]?.measured_angle_deg;
  return Boolean(
    tracking.enabled !== false &&
    (
      robotState.pose_source === "encoder_shoulder_tracking" ||
      robotState.joint_authority?.[1] === "measured" ||
      robotState.encoder_mismatch?.pose_tracking_status === "applied"
    ) &&
    Number.isFinite(Number(measuredShoulder))
  );
}

function activeShoulderJointInput() {
  return Boolean(
    document.activeElement?.matches?.(
      '.joint-slider[data-index="1"], .angle-input[data-index="1"]'
    )
  );
}

function shoulderDeltaFromReported(angles, robotState = state.robotState) {
  const normalized = normalizeJointAngles(angles);
  const reported = normalizeJointAngles(robotState?.reported_angles_deg);
  if (!normalized || !reported) return 0;
  return Math.abs(Number(normalized[1]) - Number(reported[1]));
}

function encoderTrackedShoulderToleranceDeg() {
  return Math.max(
    0.1,
    Number(state.config?.encoders?.pose_tracking?.min_update_delta_deg ?? ENCODER_STANDARD_LIMITS.pose_tracking_min_update_delta_deg),
    0.25
  );
}

function jointControlAngles(robotState = state.robotState) {
  const reported = normalizeJointAngles(robotState?.reported_angles_deg);
  if (
    robotStateUsesEncoderTrackedShoulder(robotState) &&
    !state.draftAngles &&
    !state.pendingAngles &&
    !state.commandedAngles &&
    reported
  ) {
    return reported;
  }
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
  syncJointInputs(targets, reported, robotState);
}

function syncEncoderTrackedShoulderUi(robotState = state.robotState) {
  if (!robotStateUsesEncoderTrackedShoulder(robotState) || activeShoulderJointInput()) return;
  const tolerance = encoderTrackedShoulderToleranceDeg();
  const staleDraft = shoulderDeltaFromReported(state.draftAngles, robotState) > tolerance;
  const stalePending = shoulderDeltaFromReported(state.pendingAngles, robotState) > tolerance;
  const staleCommanded = shoulderDeltaFromReported(state.commandedAngles, robotState) > tolerance;
  const previewStart = normalizeJointAngles(state.latestPreview?.start_reported_angles_deg);
  const stalePreviewStart = shoulderDeltaFromReported(previewStart, robotState) > tolerance;
  const shouldResetJointIntent = staleDraft || stalePending || staleCommanded;
  if (shouldResetJointIntent) {
    releaseJointControlIntent();
  }
  // Preview endpoints are expected to differ from the current pose. Only the
  // recorded preview start pose can make the preview stale.
  if (stalePreviewStart) {
    clearViewPreview();
    if (state.activeTab === "ik" && elements.previewStatus) {
      elements.previewStatus.textContent = "Shoulder encoder updated the start pose. Preview from the current shoulder before executing.";
    }
  }
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

function syncJointInputs(targets, reported, robotState = state.robotState) {
  targets.forEach((angle, index) => {
    const slider = elements.jointControls.querySelector(`.joint-slider[data-index="${index}"]`);
    const input = elements.jointControls.querySelector(`.angle-input[data-index="${index}"]`);
    const targetLabel = $(`#target-${index}`);
    const reportedLabel = $(`#reported-${index}`);
    const measuredLabel = $(`#measured-${index}`);
    if (!slider || !input || !targetLabel || !reportedLabel) return;
    if (document.activeElement !== slider) slider.value = angle;
    if (document.activeElement !== input) input.value = format(angle, 1);
    targetLabel.textContent = `${format(angle, 1)} deg`;
    reportedLabel.textContent = `${format(reported[index], 1)} deg`;
    if (measuredLabel) {
      const evidence = robotState?.encoder_evidence?.[index];
      measuredLabel.textContent = encoderMeasuredText(evidence, { precision: 1, rawFallback: index === 1 });
      measuredLabel.title = index === 1
        ? `Latest shoulder encoder readback: ${encoderSampleQualityText(evidence)}`
        : `Measurement authority: ${encoderEvidenceHealthText(evidence)}`;
    }
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
    elements.taskRunMonitor.innerHTML = state.taskPreviewId
      ? `<div class="task-ready-state"><strong>Preview ready</strong><span>Review the generated sequence above, then press Run Preview. The task will use exactly this bound preview.</span></div>`
      : `<div class="empty-state">No task running. Return to Plan and build a preview first.</div>`;
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
  const lastCompleted = execution.last_completed_step || {};
  const recovery = execution.recovery || {};
  const latest = execution.latest_capture || {};
  const candidates = execution.candidate_objects || [];
  const ignored = execution.ignored_objects || [];
  const warnings = execution.warnings || [];
  const terminalDetail = execution.terminal_reason || execution.phase || "-";
  if (!localStatusIsNewer) {
    elements.taskStatus.textContent =
      execution.status === "failed"
        ? `Failed: ${terminalDetail}`
        : `${execution.status}: ${execution.phase || "-"}`;
    renderWorkflowStepper(execution.status);
  }
  elements.taskRunMonitor.innerHTML = `
    ${execution.status === "failed" ? `<div class="task-run-failure"><strong>Task failed</strong><span>${escapeHtml(terminalDetail)}</span><small>${step.label ? `Stopped at ${escapeHtml(step.label)}` : "No motion step completed."}</small></div>` : ""}
    <div class="task-run-grid">
      <div class="run-metric"><span>Status</span><strong>${escapeHtml(execution.status)}</strong><small>${escapeHtml(terminalDetail)}</small></div>
      <div class="run-metric"><span>Completed</span><strong>${execution.completed_count || 0}</strong><small>remaining ${execution.remaining_count ?? "-"}</small></div>
      <div class="run-metric"><span>Current</span><strong>${current.color || "-"}</strong><small>${current.detection_id || current.drop_zone || "-"}</small></div>
      <div class="run-metric"><span>Step</span><strong>${step.index && step.total ? `${step.index}/${step.total}` : "-"}</strong><small>${step.label || "-"}</small></div>
      <div class="run-metric"><span>Capture</span><strong>${latest.detection_count ?? "-"}</strong><small>${latest.provider || latest.calibration_source || "-"}</small></div>
      <div class="run-metric"><span>Tool/object</span><strong>${execution.object_hold_state || "none"}</strong><small>${execution.holding_uncertain ? "holding uncertain" : "no object hold indicated"}</small></div>
      <div class="run-metric"><span>Last completed</span><strong>${lastCompleted.index && lastCompleted.total ? `${lastCompleted.index}/${lastCompleted.total}` : "-"}</strong><small>${lastCompleted.label || "-"}</small></div>
      <div class="run-metric"><span>Safe retreat</span><strong>${recovery.safe_retreat_available ? "available" : "unavailable"}</strong><small>${step.phase || execution.phase || "-"}</small></div>
    </div>
    ${execution.status === "waiting_for_selection" ? `<div class="task-waiting">Manual mode: choose a candidate with the Pick button in the detection table.</div>` : ""}
    ${execution.status === "waiting_for_confirmation" ? `<div class="task-waiting task-confirmation"><div><strong>Object complete.</strong><span>The robot is paused before the next capture or object.</span></div><button type="button" class="primary" data-task-continue>Continue</button></div>` : ""}
    <div class="task-object-queue compact">
      ${candidates.slice(0, 8).map((item) => `<div class="task-object-row"><span>${item.detection_id}</span><strong>${item.color}</strong><code>x ${format(item.robot?.x_mm)}, y ${format(item.robot?.y_mm)}</code><small>${item.drop_zone || "-"}</small>${execution.status === "waiting_for_selection" ? `<button type="button" class="ghost compact-action" data-runtime-select="${item.detection_id}">Pick</button>` : ""}</div>`).join("") || `<div class="empty-state">No candidate queue exposed.</div>`}
    </div>
    <div class="ignored-list">
      ${ignored.slice(0, 6).map((item) => `<div><span>${item.color || item.detection_id}</span><code>${item.reason || item.reason_code}</code></div>`).join("")}
      ${warnings.map((warning) => `<div><span>warning</span><code>${warning}</code></div>`).join("")}
    </div>
    <div class="task-recovery-options">
      <strong>Recovery options</strong>
      ${(recovery.options || execution.recovery_options || []).map((option) => `<div>${escapeHtml(option)}</div>`).join("") || `<div>None reported.</div>`}
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

function setPoseCandidateAngles() {
  return (
    normalizeJointAngles(state.draftAngles) ||
    normalizeJointAngles(state.robotState?.target_angles_deg) ||
    normalizeJointAngles(state.robotState?.reported_angles_deg) ||
    normalizeJointAngles(state.config?.joints?.map((joint) => joint.home_deg)) ||
    []
  );
}

function setPoseModalVisible(visible) {
  if (!elements.setPoseModal) return;
  elements.setPoseModal.hidden = !visible;
  if (!visible) state.pendingSetPoseAngles = null;
}

function setCurrentPoseKnown() {
  const angles = setPoseCandidateAngles();
  if (angles.length !== 4 || angles.some((angle) => !Number.isFinite(angle))) {
    showLocalError("Cannot set pose: joint angles are incomplete.");
    return;
  }
  state.pendingSetPoseAngles = angles;
  if (elements.setPoseAngles) {
    elements.setPoseAngles.innerHTML = angles
      .map((angle, index) => `<div class="log-line"><span>J${index + 1}</span><code>${format(angle, 2)} deg</code></div>`)
      .join("");
  }
  setPoseModalVisible(true);
}

async function confirmCurrentPoseKnown() {
  const angles = normalizeJointAngles(state.pendingSetPoseAngles);
  if (!angles) {
    setPoseModalVisible(false);
    return;
  }
  elements.confirmSetPoseBtn.disabled = true;
  elements.statusPill.textContent = "Setting known pose...";
  const payload = await postJson("/api/hardware/setpose", { angles_deg: angles });
  elements.confirmSetPoseBtn.disabled = false;
  if (payload.ok) {
    invalidatePendingIkPreview();
    releaseJointControlIntent();
    clearViewPreview();
    state.ikUserEdited = false;
    elements.statusPill.textContent = "Pose marked known.";
    setPoseModalVisible(false);
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

function programTargetPreviewIsFresh(step = selectedProgramStep()) {
  const preview = state.programTargetPreview;
  if (!step || !preview) return false;
  return Boolean(
    preview.stepId === step.id &&
    preview.programRevision === state.programRevision &&
    preview.previewId === state.previewId &&
    preview.previewId === state.latestPreview?.id &&
    Number(preview.startPoseRevision) === Number(state.robotState?.pose_revision) &&
    state.robotState?.motion_state !== "moving"
  );
}

function programTargetPreviewStatus(step = selectedProgramStep()) {
  if (!step) return { text: "Select a step to test its target.", tone: "" };
  if (state.programTargetPreviewPending) return { text: "Planning from the current reported pose...", tone: "warn" };
  if (state.programTargetMovePending) return { text: state.programTargetStatus || "Starting target move...", tone: "warn" };
  if (state.programTargetStatus) {
    const error = /failed|invalid|enable|first|blocked|stale|error|outside|cannot|requires?/i.test(state.programTargetStatus);
    return { text: state.programTargetStatus, tone: error ? "error" : "ready" };
  }
  if (programTargetPreviewIsFresh(step)) {
    return {
      text: "Preview ready from the current robot pose. Go to target will execute this exact path.",
      tone: "ready",
    };
  }
  if (state.programTargetPreview) {
    return { text: "Target preview is stale or belongs to another step. Preview this target again.", tone: "warn" };
  }
  return { text: "Preview draws the direct path from the current robot pose. It does not execute motion.", tone: "" };
}

function updateProgramTargetControls() {
  const step = selectedProgramStep();
  const previewButton = elements.programInspector?.querySelector('[data-program-target-action="preview"]');
  const goButton = elements.programInspector?.querySelector('[data-program-target-action="go"]');
  const status = elements.programInspector?.querySelector("#programTargetTestStatus");
  const validation = step ? programStepClientValidation(step, selectedProgramIndex()) : { ok: false };
  const previewDisabled = Boolean(
    !step ||
    step.enabled === false ||
    !validation.ok ||
    state.programTargetPreviewPending ||
    state.programTargetMovePending ||
    state.programExecutionActive ||
    state.robotState?.motion_state === "moving"
  );
  const goDisabled = Boolean(
    !programTargetPreviewIsFresh(step) ||
    programMotionGateReason() ||
    state.programTargetPreviewPending ||
    state.programTargetMovePending ||
    state.programExecutionActive
  );
  if (previewButton) previewButton.disabled = previewDisabled;
  if (goButton) {
    goButton.disabled = goDisabled;
    goButton.title = goDisabled
      ? programTargetPreviewIsFresh(step)
        ? programMotionGateReason()
        : "Preview this exact target from the current robot pose first"
      : "Execute the currently displayed target preview";
  }
  if (status) {
    const targetStatus = programTargetPreviewStatus(step);
    status.textContent = targetStatus.text;
    status.dataset.tone = targetStatus.tone;
  }
}

function programSelectedSourceIsReady() {
  const source = elements.programStepSource?.value;
  if (!["named_position", "vision_detection", "drop_zone", "end_effector"].includes(source)) return true;
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
  if (elements.alignShoulderBtn) {
    const robotState = state.robotState || {};
    const encoderEvidence = state.robotState?.encoder_evidence?.[1] || {};
    const measured = Number(encoderEvidence.measured_angle_deg);
    const target = Number(state.robotState?.target_angles_deg?.[1]);
    const correction = state.config?.encoders?.correction || {};
    const verification = state.config?.encoders?.verification || {};
    const alignThreshold = Math.max(
      Number(correction.deadband_deg ?? ENCODER_STANDARD_LIMITS.correction_deadband_deg),
      Number(verification.warning_tolerance_deg ?? ENCODER_STANDARD_LIMITS.warning_tolerance_deg),
      0.1
    );
    const alignError = Number.isFinite(measured) && Number.isFinite(target) ? measured - target : null;
    const needsAlign = Boolean(encoderEvidence.fresh) && alignError != null && Math.abs(alignError) > alignThreshold;
    const alignIdleStateReady = ["idle", "stopped"].includes(state.robotState?.motion_state);
    const encoderFaultOverrideReady = Boolean(robotState.encoder_fault) && robotState.motion_state === "fault";
    const alignMotionReady = alignIdleStateReady || encoderFaultOverrideReady;
    elements.alignShoulderBtn.disabled = !robotState.connected
      || Boolean(robotState.simulation)
      || !alignMotionReady
      || (encoderEvidence.measured_angle_deg == null && encoderEvidence.raw_angle_deg == null);
    elements.alignShoulderBtn.classList.toggle("attention", needsAlign);
    elements.alignShoulderBtn.title = encoderFaultOverrideReady
      ? "Clear the encoder fault and run shoulder alignment"
      : needsAlign
      ? `Shoulder is ${format(alignError, 2)} deg from the planning target. Press Align before normal hardware moves.`
      : "Move shoulder toward the current planning/target angle using encoder alignment";
  }
  elements.setPoseBtn.disabled =
    !state.robotState ||
    (!state.robotState.connected && !state.robotState.simulation) ||
    Boolean(state.previewAngles);
  elements.stopBtn.disabled = !state.robotState?.connected && !state.robotState?.simulation;
  if (elements.hardwareArmToggle) {
    const armingBlocked =
      !state.robotState?.hardware_armed &&
      (!state.robotState?.known_pose || state.robotState?.config_sync_status !== "synced");
    elements.hardwareArmToggle.disabled =
      !state.robotState?.connected ||
      state.robotState?.simulation ||
      armingBlocked;
    elements.hardwareArmToggle.title = state.robotState?.hardware_armed
      ? "Disarm hardware"
      : armingBlocked
        ? state.robotState?.config_sync_message || "Sync controller configuration and establish a known pose before arming"
        : "Arm hardware";
  }
  const syncDisabled =
    !state.robotState?.connected ||
    state.robotState?.simulation ||
    Boolean(state.robotState?.hardware_armed) ||
    state.robotState?.motion_state === "moving";
  const syncTitle = state.robotState?.hardware_armed
    ? "Disarm hardware before syncing controller configuration"
    : state.robotState?.motion_state === "moving"
      ? "Wait for motion to finish before syncing controller configuration"
      : state.robotState?.simulation
        ? "Controller sync is not required in simulation"
        : !state.robotState?.connected
          ? "Connect the ESP before syncing controller configuration"
          : "Save all settings and sync controller configuration to the ESP";
  [elements.syncHardwareBtn, elements.viewportSyncHardwareBtn].forEach((button) => {
    if (!button) return;
    button.disabled = syncDisabled;
    button.title = syncTitle;
  });
  elements.executeIkBtn.disabled = !state.previewId || !enabled;
  document.querySelectorAll("[data-position-go], [data-position-preview]").forEach((button) => {
    button.disabled = !enabled || state.robotState?.motion_state === "moving";
    button.title = state.robotState?.motion_state === "moving" ? "Wait for the current motion to finish" : "";
  });
  const enabledProgramSteps = state.programWaypoints.filter((step) => step.enabled !== false).length;
  const programGateReason = programMotionGateReason();
  const programLocked = programEditingLocked();
  elements.previewProgramBtn.disabled =
    state.programPreviewPending ||
    state.programPlanRestorePending ||
    state.programExecutionActive ||
    enabledProgramSteps === 0;
  elements.executeProgramBtn.disabled =
    !programPreviewIsFresh() ||
    Boolean(programGateReason) ||
    state.programPreviewPending ||
    state.programPlanRestorePending ||
    state.programExecutionActive;
  elements.clearProgramBtn.disabled = state.programWaypoints.length === 0 || programLocked;
  elements.addProgramStepBtn.disabled = programLocked || !programSelectedSourceIsReady();
  elements.programStepSource.disabled = programLocked;
  elements.programSourceItem.disabled = programLocked;
  elements.programInsertPosition.disabled = programLocked;
  updateProgramTargetControls();
  elements.stopProgramBtn.disabled = !state.programExecutionActive && state.robotState?.motion_state !== "moving";
  if (elements.cartesianJogToggle) elements.cartesianJogToggle.disabled = !enabled;
  if (elements.cartesianJogSpeedInput) elements.cartesianJogSpeedInput.disabled = !enabled;
  if (elements.cartesianJogPhiSpeedInput) elements.cartesianJogPhiSpeedInput.disabled = !enabled;
  document.querySelectorAll(".target-fader").forEach((fader) => {
    const disabled = !enabled || (cartesianJogEnabled() && !cartesianJogCanRun()) || (fader.dataset.faderKey === "phi" && ikAutoPhiEnabled());
    fader.classList.toggle("disabled", disabled);
    fader.setAttribute("aria-disabled", disabled ? "true" : "false");
  });
  if (elements.previewTaskBtn) elements.previewTaskBtn.disabled = !state.config || state.taskPreviewPending;
  if (elements.executeTaskBtn) elements.executeTaskBtn.disabled = !state.taskPreviewId || !enabled || taskDraftBlocksRun();
  if (elements.taskStopBtn) {
    const taskStatus = state.robotState?.task_execution?.status;
    elements.taskStopBtn.disabled = !["queued", "running", "capturing", "planning", "executing", "waiting_for_selection", "waiting_for_confirmation"].includes(taskStatus);
  }
}

function encoderPoseTrackingPreviewToleranceDeg() {
  const tracking = state.config?.encoders?.pose_tracking || {};
  if (tracking.enabled === false) return 0.1;
  const value = Number(
    tracking.preview_stale_tolerance_deg ??
    ENCODER_STANDARD_LIMITS.pose_tracking_preview_stale_tolerance_deg
  );
  return Math.max(
    0.1,
    Number.isFinite(value) ? value : ENCODER_STANDARD_LIMITS.pose_tracking_preview_stale_tolerance_deg
  );
}

function previewStartPoseIsStaleForCurrentState(robotState) {
  const previewStart = normalizeJointAngles(state.latestPreview?.start_reported_angles_deg);
  const reported = normalizeJointAngles(robotState?.reported_angles_deg);
  if (!state.latestPreview || !previewStart || !reported) return false;
  const trackingApplied = robotStateUsesEncoderTrackedShoulder(robotState);
  return reported.some((value, index) => {
    const allowed = trackingApplied && index === 1
      ? encoderPoseTrackingPreviewToleranceDeg()
      : 0.1;
    return Math.abs(value - previewStart[index]) > allowed;
  });
}

function renderState(robotState) {
  const incomingUpdatedAt = Number(robotState?.updated_at || 0);
  if (incomingUpdatedAt && incomingUpdatedAt < state.lastRobotStateUpdatedAt) return;
  const previousPoseRevision = Number(state.robotState?.pose_revision ?? robotState?.pose_revision ?? 0);
  const incomingPoseRevision = Number(robotState?.pose_revision ?? 0);
  state.lastRobotStateUpdatedAt = Math.max(state.lastRobotStateUpdatedAt, incomingUpdatedAt);
  state.robotState = robotState;
  const poseRevisionChanged = incomingPoseRevision !== previousPoseRevision;
  syncEncoderTrackedShoulderUi(robotState);
  const previewIsStale = Boolean(
    state.latestPreview &&
    previewStartPoseIsStaleForCurrentState(robotState)
  );
  const localIntentIsStale = Boolean(
    !state.latestPreview &&
    poseRevisionChanged &&
    state.intentBasePoseRevision != null &&
    state.intentBasePoseRevision !== incomingPoseRevision
  );
  if (previewIsStale || localIntentIsStale) {
    const staleProgramPreview = state.latestPreview?.mode === "program" && state.latestPreview?.source !== "task";
    const preserveRequestedIkTarget = state.ikUserEdited;
    invalidatePendingIkPreview();
    releaseJointControlIntent();
    clearViewPreview();
    state.ikUserEdited = preserveRequestedIkTarget;
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
  if (elements.toolStatus && !state.toolSwitchPending) {
    elements.toolStatus.classList.remove("warn", "error");
    elements.toolStatus.textContent = robotState.tool_state || "unknown";
  }
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
  renderModelTruthSummary();
  elements.lastCommand.textContent = robotState.last_command || "-";
  elements.lastError.textContent = robotState.last_error || "-";
  elements.portStatus.textContent = robotState.serial_port || (robotState.simulation ? "simulation" : "-");
  elements.hardwareArmToggle.checked = Boolean(robotState.hardware_armed);
  renderHardwareStatus(robotState);
  renderEncoderStatus(robotState);
  renderSetupChecklist();
  const hardwareSuffix = robotState.simulation ? "" : ` - ${robotState.hardware_mode}/${robotState.config_sync_status}`;
  elements.statusPill.textContent = robotState.last_error || `${robotState.motion_state}${robotState.live_motion_enabled ? " - live real" : ""}${hardwareSuffix}`;
  renderMotionExecution(robotState);
  renderTaskExecution(robotState);
  syncProgramExecutionState(robotState);
  renderProgramWorkflowStatus();
  renderProgramPreviewSummary();
  renderProgramRunMonitor(robotState);
  scheduleDiagnosticsRender();
  if (state.cartesianJogActiveAxes.size === 0) setIkTargetFromFk(fk);
  updateDisabledState();
  if (!state.settingsDirtyScopes.size) updateSettingsSaveBar();
}

function renderPreview(preview, options = {}) {
  releaseJointControlIntent();
  if (!options.preserveTaskPreview) state.taskPreviewId = null;
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
    const continuityOk = candidate.configuration_continuous !== false;
    const selectable = candidate.valid && continuityOk;
    const selectedAngles = ik.selected?.angles_deg || [];
    const isSelected =
      candidate.angles_deg?.length === selectedAngles.length &&
      candidate.angles_deg.every((angle, index) => Math.abs(Number(angle) - Number(selectedAngles[index])) < 1e-6);
    item.className = `candidate ${selectable ? "" : "invalid"} ${isSelected ? "selected" : ""}`;
    const candidatePhi = candidate.target_phi_deg ?? candidate.fk?.tool_phi_deg;
    const phiDetail = candidate.auto_phi
      ? `chosen phi ${format(candidatePhi, 2)} deg`
      : `phi ${format(candidate.phi_error_deg, 3)} deg`;
    item.innerHTML = `
      <div class="candidate-title">
        <span>${escapeHtml(candidate.solution_family || candidate.branch.replace("_", " "))}</span>
        <span>${selectable ? "valid" : continuityOk ? "IK rejected" : "motion blocked"}</span>
      </div>
      <div class="angle-list">
        ${candidate.angles_deg.map((angle, index) => `<div><span>J${index + 1}</span><strong>${format(angle, 2)} deg</strong></div>`).join("")}
      </div>
      <div class="small">FK error ${format(candidate.position_error_mm, 3)} mm, ${phiDetail}</div>
      <div class="small">Joint travel ${format(candidate.joint_travel_deg, 1)} deg · base ${format(candidate.base_delta_deg, 1)} deg · tool winding ${format(candidate.tool_winding_delta_deg, 1)} deg</div>
      <div class="small">${escapeHtml([...(candidate.reasons || []), ...(candidate.continuity_violations || [])].join("; "))}</div>
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
    ${motionContractHtml(preview, trajectory)}
    <div class="log-line"><span>Path type</span><code>${pathLayerDescription(preview, trajectory)}</code></div>
    <div class="log-line"><span>Branch</span><code>${ik.selected_branch || "-"}</code></div>
    <div class="log-line"><span>Target phi</span><code>${preview.target?.phi_auto ? `auto -> ${format(preview.target.phi_deg, 2)} deg` : `${format(preview.target?.phi_deg, 2)} deg`}</code></div>
    <div class="log-line"><span>TCP calibration</span><code>${calibrationApplied ? "applied at Cartesian command layer" : "not applied"}</code></div>
    <div class="log-line"><span>Model command</span><code>${commandTarget.x_mm !== undefined ? `x ${format(commandTarget.x_mm, 2)}, y ${format(commandTarget.y_mm, 2)}, z ${format(commandTarget.z_mm, 2)}` : preview.mode === "program" ? "per Cartesian waypoint" : "-"}</code></div>
    <div class="log-line"><span>Execute</span><code id="motionProgressLine">idle - 0%</code></div>
    <div class="log-line"><span>Segments</span><code>${segmentText}</code></div>
  `;

  if (state.previewAngles) state.view.setPreviewAngles(state.previewAngles);

  const taskFocusCartesian = taskPreviewFocusStep(preview)?.trajectory?.cartesian_waypoints;
  const taskFocusTarget =
    Array.isArray(taskFocusCartesian) && taskFocusCartesian.length
      ? taskFocusCartesian[taskFocusCartesian.length - 1]
      : null;
  const target =
    taskFocusTarget ||
    (preview.target?.x_mm !== undefined
      ? preview.target
      : trajectory.cartesian_waypoints?.[trajectory.cartesian_waypoints.length - 1]);
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
  stopProgramPlayback({ reset: true });
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
    const reasons = [...(candidate.reasons || []), ...(candidate.continuity_violations || [])];
    item.className = "candidate invalid";
    item.textContent = `${candidate.solution_family || candidate.branch}: ${reasons.join("; ") || "not selected"}`;
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
    const requestedTarget = ikTargetPayload();
    const payload = await postJson("/api/path/preview", {
      target: requestedTarget,
      mode: elements.ikModeSelect.value,
      branch: elements.ikBranchSelect.value,
      settings: pathSettings(),
    });
    if (requestSeq === state.ikPreviewWantedSeq) {
      if (payload.ok) {
        renderPreview(payload.preview);
        setIkTargetFromRequestedTarget(requestedTarget);
      }
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
    state.latestPreview = null;
    state.previewAngles = null;
    state.previewBasePoseRevision = null;
    state.taskPreviewId = null;
    // Keep the requested Cartesian target in the controls. Reported FK may
    // reflect the calibration-adjusted model command rather than that target.
    state.ikUserEdited = true;
  } else if (/preview|configuration|model|start pose/i.test(payload.error || "")) {
    clearViewPreview();
    elements.previewStatus.textContent = payload.error || "Preview is stale";
  }
  if (payload.state) renderState(payload.state);
  else syncJointControls();
  updateDisabledState();
}

function activeProgramRecord() {
  return state.programLibrary.find((program) => program.id === state.programActiveId) || null;
}

function programEditingLocked() {
  return Boolean(
    state.programReadOnly ||
    state.programExecutionActive ||
    state.programSaving ||
    state.programPlanRestorePending
  );
}

function setProgramLibraryStatus(message, tone = "") {
  if (!elements.programLibraryStatus) return;
  elements.programLibraryStatus.textContent = message;
  elements.programLibraryStatus.classList.toggle("warn", tone === "warn");
  elements.programLibraryStatus.classList.toggle("error", tone === "error");
}

function resetProgramPreviewState(reason = "") {
  clearProgramTargetPreview();
  state.programPreview = null;
  state.programPreviewFailure = null;
  state.programPreviewRevision = null;
  state.programValidationRevision = null;
  state.programPlanRestorePending = false;
  state.programSavedPlanStatus = "";
  state.programHasPreviewed = false;
  state.programExecutionFailed = false;
  state.programExecutionAwaitingStart = false;
  state.programExecutionError = "";
  state.programLastEditReason = reason;
  clearActiveProgramPreview();
}

function setActiveProgramStage(stage) {
  const allowed = ["library", "build", "preview", "run"];
  state.activeProgramStage = allowed.includes(stage) ? stage : "library";
  elements.programPanels.forEach((panel) => {
    panel.hidden = panel.dataset.programPanel !== state.activeProgramStage;
  });
  elements.programWorkflow?.querySelectorAll("[data-program-stage]").forEach((button) => {
    button.classList.toggle("active", button.dataset.programStage === state.activeProgramStage);
  });
}

function currentProgramPayload({ copy = false } = {}) {
  const name = String(state.programName || elements.programNameInput?.value || "Untitled program").trim() || "Untitled program";
  return {
    id: copy ? null : state.programActiveId,
    schema_version: 1,
    name: copy ? `${name} Copy` : name,
    description: String(state.programDescription || "").trim(),
    steps: clonePlain(state.programWaypoints),
  };
}

function replaceCurrentProgram(program, { stage = "build" } = {}) {
  state.programActiveId = program?.id || null;
  state.programName = String(program?.name || "Untitled program");
  state.programDescription = String(program?.description || "");
  state.programReadOnly = Boolean(program?.read_only);
  state.programWaypoints = clonePlain(program?.steps || []);
  state.programSelectedId = state.programWaypoints[0]?.id || null;
  state.programNextId = Math.max(1, state.programWaypoints.length + 1);
  state.programRevision += 1;
  state.programDirty = false;
  resetProgramPreviewState(program ? `Loaded ${state.programName}` : "New program");
  setActiveProgramStage(stage);
  renderProgramBuilder();
  renderProgramLibrary();
}

async function restoreSavedProgramPlan({ stageOnSuccess = "run", quiet = false } = {}) {
  const programId = state.programActiveId;
  const revision = state.programRevision;
  if (
    !programId ||
    state.programReadOnly ||
    state.programDirty ||
    state.programPlanRestorePending ||
    state.programExecutionActive
  ) {
    return false;
  }
  state.programPlanRestorePending = true;
  state.programSavedPlanStatus = quiet ? "" : "Checking saved plan...";
  renderProgramBuilder({ inspector: false });
  let payload;
  try {
    const response = await fetch(`/api/programs/${encodeURIComponent(programId)}/restore-plan`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ program_revision: revision }),
    });
    payload = await response.json();
  } catch (error) {
    if (programId === state.programActiveId && revision === state.programRevision) {
      state.programPlanRestorePending = false;
      state.programSavedPlanStatus = `Could not load the saved plan: ${error.message || error}`;
      renderProgramBuilder({ inspector: false });
    }
    return false;
  }
  if (programId !== state.programActiveId || revision !== state.programRevision) {
    if (programId === state.programActiveId) state.programPlanRestorePending = false;
    return false;
  }
  state.programPlanRestorePending = false;
  if (!payload.ok) {
    state.programSavedPlanStatus = payload.error || "Saved plan is not reusable from the current robot pose.";
    if (!quiet && activeProgramRecord()?.cached_plan?.available) {
      state.programLastEditReason = state.programSavedPlanStatus;
    }
    renderProgramBuilder({ inspector: false });
    return false;
  }

  clearProgramTargetPreview();
  renderPreview(payload.preview);
  state.programPreview = payload.preview;
  state.programPreviewRevision = revision;
  state.programValidationRevision = revision;
  state.programPreviewFailure = null;
  state.programHasPreviewed = true;
  state.programExecutionFailed = false;
  state.programSavedPlanStatus = "Saved plan loaded for the current starting pose.";
  state.programPlaybackElapsedS = 0;
  if (stageOnSuccess) setActiveProgramStage(stageOnSuccess);
  renderProgramBuilder({ inspector: false });
  return true;
}

async function loadLibraryProgram(program) {
  replaceCurrentProgram(program, { stage: "build" });
  await restoreSavedProgramPlan();
}

function startNewProgram() {
  replaceCurrentProgram(
    {
      id: null,
      name: "Untitled program",
      description: "",
      read_only: false,
      steps: [],
    },
    { stage: "build" }
  );
  state.programDirty = false;
  setProgramLibraryStatus("New draft");
}

function upsertProgramLibraryRecord(program) {
  const index = state.programLibrary.findIndex((item) => item.id === program.id);
  if (index >= 0) state.programLibrary[index] = clonePlain(program);
  else state.programLibrary.push(clonePlain(program));
}

async function loadProgramLibrary({ preserveActive = true } = {}) {
  const payload = await requestJson("/api/programs");
  if (!payload.ok) {
    setProgramLibraryStatus(payload.error || "Could not load programs", "error");
    return;
  }
  state.programLibrary = Array.isArray(payload.programs) ? payload.programs : [];
  if (preserveActive && state.programActiveId) {
    const active = activeProgramRecord();
    if (active && !state.programDirty) {
      state.programName = active.name;
      state.programDescription = active.description || "";
      state.programReadOnly = Boolean(active.read_only);
    }
  }
  renderProgramLibrary();
}

function programLibraryRowHtml(program) {
  const isActive = program.id === state.programActiveId;
  const template = Boolean(program.read_only || program.template);
  const steps = Array.isArray(program.steps) ? program.steps.length : 0;
  const metadata = program.metadata || {};
  const cachedPlan = program.cached_plan || {};
  const adaptiveText = metadata.adaptive ? "geometry-adaptive" : "saved targets";
  return `
    <div class="program-library-row ${isActive ? "active" : ""}" data-program-id="${escapeHtml(program.id)}">
      <div class="program-library-row-header">
        <strong>${escapeHtml(program.name)}</strong>
        <span class="program-library-kind">${template ? "Read-only template" : "User program"}</span>
      </div>
      <p>${escapeHtml(program.description || "No description")}</p>
      <div class="program-library-row-meta">
        <span>${steps} step${steps === 1 ? "" : "s"}</span>
        <span>${template ? adaptiveText : "persistent"}</span>
        ${cachedPlan.available ? `<span>saved plan</span>` : ""}
        ${metadata.radius_mm ? `<span>${format(metadata.radius_mm, 1)} mm radius</span>` : ""}
      </div>
      <div class="program-library-row-actions">
        <button type="button" class="ghost" data-program-library-action="load" data-program-id="${escapeHtml(program.id)}">Load</button>
        <button type="button" data-program-library-action="copy" data-program-id="${escapeHtml(program.id)}">${template ? "Copy" : "Duplicate"}</button>
        ${template ? "" : `<button type="button" class="ghost danger-text" data-program-library-action="delete" data-program-id="${escapeHtml(program.id)}">Delete</button>`}
      </div>
    </div>
  `;
}

function renderProgramLibrary() {
  if (!elements.builtInProgramList || !elements.userProgramList) return;
  const builtIns = state.programLibrary.filter((program) => program.read_only || program.template);
  const users = state.programLibrary.filter((program) => !program.read_only && !program.template);
  elements.builtInProgramList.innerHTML = builtIns.length
    ? builtIns.map(programLibraryRowHtml).join("")
    : `<div class="program-library-empty">No built-in demos are available.</div>`;
  elements.userProgramList.innerHTML = users.length
    ? users.map(programLibraryRowHtml).join("")
    : `<div class="program-library-empty">No saved programs yet.</div>`;

  if (elements.programNameInput && document.activeElement !== elements.programNameInput) {
    elements.programNameInput.value = state.programName;
  }
  if (elements.programDescriptionInput && document.activeElement !== elements.programDescriptionInput) {
    elements.programDescriptionInput.value = state.programDescription;
  }
  const locked = programEditingLocked();
  elements.programNameInput.disabled = state.programReadOnly || state.programExecutionActive;
  elements.programDescriptionInput.disabled = state.programReadOnly || state.programExecutionActive;
  elements.saveProgramBtn.disabled =
    locked ||
    state.programReadOnly ||
    !String(state.programName || "").trim();
  elements.copyProgramBtn.disabled = state.programSaving || state.programExecutionActive || !state.programWaypoints.length;
  elements.newProgramBtn.disabled = state.programSaving || state.programExecutionActive;
  elements.programTemplateNotice.hidden = !state.programReadOnly;
  elements.programTemplateNotice.textContent = state.programReadOnly
    ? "This built-in is read-only. You can preview and run it, or use Save as copy to make an editable program."
    : "";

  if (state.programSaving) setProgramLibraryStatus("Saving...");
  else if (state.programReadOnly) setProgramLibraryStatus("Template");
  else if (state.programDirty) setProgramLibraryStatus("Unsaved changes", "warn");
  else if (state.programActiveId) setProgramLibraryStatus("Saved");
  else setProgramLibraryStatus("Draft");
}

async function saveCurrentProgram({ copy = false } = {}) {
  if (state.programSaving) return;
  if (state.programReadOnly && !copy) {
    showLocalError("Built-in templates are read-only. Use Save as copy.");
    return;
  }
  state.programSaving = true;
  renderProgramLibrary();
  let payload;
  if (copy && state.programReadOnly && state.programActiveId) {
    payload = await postJson(`/api/programs/${encodeURIComponent(state.programActiveId)}/copy`, {
      name: `${state.programName} Copy`,
    });
  } else {
    const program = currentProgramPayload({ copy });
    const updating = Boolean(program.id && !copy);
    payload = await requestJson(
      updating ? `/api/programs/${encodeURIComponent(program.id)}` : "/api/programs",
      { method: updating ? "PUT" : "POST", body: program }
    );
  }
  state.programSaving = false;
  if (!payload.ok) {
    setProgramLibraryStatus(payload.error || "Save failed", "error");
    renderProgramLibrary();
    return;
  }
  upsertProgramLibraryRecord(payload.program);
  replaceCurrentProgram(payload.program, { stage: state.activeProgramStage });
  setProgramLibraryStatus(copy ? "Copy saved" : "Saved");
}

async function deleteProgram(programId) {
  const program = state.programLibrary.find((item) => item.id === programId);
  if (!program || program.read_only) return;
  const payload = await requestJson(`/api/programs/${encodeURIComponent(programId)}`, { method: "DELETE" });
  if (!payload.ok) {
    setProgramLibraryStatus(payload.error || "Delete failed", "error");
    return;
  }
  state.programLibrary = state.programLibrary.filter((item) => item.id !== programId);
  if (state.programActiveId === programId) startNewProgram();
  else renderProgramLibrary();
  setProgramLibraryStatus("Deleted");
}

async function copyLibraryProgram(programId) {
  const payload = await postJson(`/api/programs/${encodeURIComponent(programId)}/copy`, {});
  if (!payload.ok) {
    setProgramLibraryStatus(payload.error || "Copy failed", "error");
    return;
  }
  upsertProgramLibraryRecord(payload.program);
  replaceCurrentProgram(payload.program, { stage: "build" });
  setProgramLibraryStatus("Copy ready");
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

function activeProgramTool() {
  const tools = state.config?.tools || {};
  const name = tools.active || state.robotState?.active_tool || "gripper";
  const preset = tools.presets?.[name] || {};
  return {
    name,
    label: preset.label || name,
    type: preset.type || "generic",
  };
}

function programToolActions() {
  const tool = activeProgramTool();
  if (tool.type === "electromagnet") {
    return [
      { value: "on", label: `Turn ${tool.label} on` },
      { value: "off", label: `Turn ${tool.label} off` },
    ];
  }
  return [
    { value: "open", label: `Open ${tool.label}` },
    { value: "close", label: `Close ${tool.label}` },
    { value: "set", label: `Set ${tool.label} value` },
  ];
}

function programSourceOptions(source = elements.programStepSource.value) {
  if (source === "named_position") {
    const records = state.config?.position_library?.positions || {};
    const entries = Object.keys(records).length ? sortedPositionEntries(records) : Object.entries(state.config?.named_positions || {});
    return entries.map(([name, record]) => ({
      value: name,
      label: record?.display_name && record.display_name !== name ? `${record.display_name} (${name})` : name,
    }));
  }
  if (source === "vision_detection") {
    return programDetectionEntries().map(({ id, detection }) => ({
      value: id,
      label: `${detection.label || detection.color || "object"} · ${id}`,
    }));
  }
  if (source === "drop_zone") {
    return Object.keys(taskDestinations()).sort().map((name) => ({ value: name, label: name }));
  }
  if (source === "end_effector") return programToolActions();
  return [];
}

function programSourceHint(source = elements.programStepSource.value, optionCount = null) {
  const count = optionCount ?? programSourceOptions(source).length;
  if (source === "named_position") return count ? "Adds a saved joint or Cartesian Position Library record." : "No Position Library records are configured.";
  if (source === "manual_cartesian") return "Starts from the current IK target. Edit X, Y, Z, phi, and move mode after adding.";
  if (source === "manual_joint") return "Starts from the reported joint pose. Edit each joint after adding.";
  if (source === "end_effector") return `Adds an action for the active ${activeProgramTool().label}. Tool safety checks still apply during execution.`;
  if (source === "vision_detection") return count ? "Uses the selected detection's robot-frame coordinates." : "Refresh detections in Tasks before adding a vision target.";
  if (source === "drop_zone") return count ? "Adds a configured Cartesian task destination." : "No task destinations are configured.";
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
  const needsItem = ["named_position", "vision_detection", "drop_zone", "end_effector"].includes(source);
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
      label: uniqueProgramLabel(positionDisplayName(sourceItem, state.config?.position_library?.positions?.[sourceItem] || {})),
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
    const zone = taskDestinations()[sourceItem];
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
  if (source === "end_effector") {
    const tool = activeProgramTool();
    const action = sourceItem || programToolActions()[0]?.value || "open";
    return {
      ...base,
      label: uniqueProgramLabel(`${tool.label} ${action}`),
      type: "tool",
      mode: "tool",
      tool: tool.name,
      action,
      value: action === "set" ? Number(state.robotState?.tool_value ?? 0.5) : undefined,
      settle_ms: 150,
    };
  }
  return null;
}

function clearActiveProgramPreview() {
  stopProgramPlayback({ reset: true });
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

function clearProgramTargetPreview() {
  state.programTargetPreview = null;
  state.programTargetPreviewPending = false;
  state.programTargetMovePending = false;
  state.programTargetStatus = "";
}

function markProgramEdited(reason = "Program edited", options = {}) {
  state.programRevision += 1;
  state.programDirty = true;
  state.programLastEditReason = reason;
  stopProgramPlayback({ reset: true });
  clearProgramTargetPreview();
  state.programExecutionFailed = false;
  state.programExecutionAwaitingStart = false;
  state.programExecutionError = "";
  clearActiveProgramPreview();
  renderProgramBuilder({ inspector: options.inspector !== false });
}

function insertProgramStep(step) {
  if (programEditingLocked()) {
    showLocalError(state.programReadOnly ? "Copy this template before editing it." : "Program editing is currently locked.");
    return;
  }
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
  const sourceItem = ["named_position", "vision_detection", "drop_zone", "end_effector"].includes(source)
    ? (source === elements.programStepSource.value ? elements.programSourceItem.value : options[0]?.value)
    : "";
  insertProgramStep(createProgramStep(source, sourceItem || ""));
}

function duplicateProgramStep(index) {
  if (programEditingLocked()) return;
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
  if (programEditingLocked()) return;
  const [removed] = state.programWaypoints.splice(index, 1);
  if (!removed) return;
  const next = state.programWaypoints[Math.min(index, state.programWaypoints.length - 1)];
  state.programSelectedId = next?.id || null;
  markProgramEdited(`Deleted ${removed.label || `step ${index + 1}`}`);
}

function moveProgramStep(index, direction) {
  if (programEditingLocked()) return;
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
  if (step.type === "tool") {
    if (!["open", "close", "set", "on", "off"].includes(step.action)) errors.push("end-effector action is invalid");
    if (step.action === "set" && (!Number.isFinite(Number(step.value)) || Number(step.value) < 0 || Number(step.value) > 1)) {
      errors.push("end-effector value must be between 0 and 1");
    }
    if (!Number.isFinite(Number(step.settle_ms)) || Number(step.settle_ms) < 0) errors.push("settle time must be zero or greater");
  } else if (step.type === "joint") {
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
  if (step.type !== "tool") {
    Object.entries(step.settings || {}).forEach(([key, value]) => {
      if (!Number.isFinite(Number(value)) || Number(value) <= 0) {
        errors.push(`${key.replaceAll("_", " ")} must be positive`);
      }
    });
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
  if (step.type === "tool") {
    const value = step.action === "set" ? ` ${format(step.value, 2)}` : "";
    return `${step.tool || activeProgramTool().name} · ${String(step.action || "action").toUpperCase()}${value} · settle ${format(step.settle_ms, 0)} ms`;
  }
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
        <strong>Build the first step</strong>
        <p>Choose a source above, add the step, then select it to edit motion limits, target values, or end-effector behavior.</p>
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
          <input type="checkbox" data-program-action="toggle" data-program-index="${index}" ${step.enabled === false ? "" : "checked"} ${programEditingLocked() ? "disabled" : ""} />
          <span>${index + 1}</span>
        </label>
        <div class="program-step-copy">
          <div class="program-step-heading">
            <strong>${escapeHtml(step.label || `Step ${index + 1}`)}</strong>
            <span class="program-validation ${status}">${status === "valid" ? "Valid" : status === "invalid" ? "Invalid" : status === "disabled" ? "Disabled" : status === "stale" ? "Stale" : "Not previewed"}</span>
          </div>
          <div class="program-step-meta">
            <span>${step.type === "tool" ? "End effector" : step.type === "joint" ? "Joint target" : "Cartesian target"}</span>
            <span class="program-mode ${step.type === "tool" ? "tool" : step.mode === "linear" ? "linear" : "joint"}">${step.type === "tool" ? String(step.action || "Action") : step.mode === "linear" ? "Linear TCP" : "Joint move"}</span>
            <span>${duration}</span>
          </div>
          <code>${escapeHtml(programStepValues(step))}</code>
          ${errors.length ? `<div class="program-step-error" title="${escapeHtml(errors[0])}">${escapeHtml(conciseProgramError(errors[0]))}</div>` : ""}
        </div>
      </div>
      <div class="program-step-actions">
        <button type="button" class="ghost" data-program-action="up" data-program-index="${index}" ${index === 0 || programEditingLocked() ? "disabled" : ""} aria-label="Move step up">↑</button>
        <button type="button" class="ghost" data-program-action="down" data-program-index="${index}" ${index === stepCount - 1 || programEditingLocked() ? "disabled" : ""} aria-label="Move step down">↓</button>
        <button type="button" class="ghost" data-program-action="duplicate" data-program-index="${index}" ${programEditingLocked() ? "disabled" : ""}>Duplicate</button>
        <button type="button" class="ghost danger-text" data-program-action="delete" data-program-index="${index}" ${programEditingLocked() ? "disabled" : ""}>Delete</button>
      </div>
    `;
    elements.programList.appendChild(item);
  });
}

function programMotionLimitFields(step, disabled) {
  if (step.type === "tool") return "";
  const defaults = pathSettings();
  const linear = step.type === "cartesian" && step.mode === "linear";
  const fields = linear
    ? [
        ["tcp_speed_mm_s", "TCP speed limit", "mm/s"],
        ["tcp_accel_mm_s2", "TCP acceleration limit", "mm/s²"],
        ["phi_speed_deg_s", "Tool rotation speed", "deg/s"],
        ["phi_accel_deg_s2", "Tool rotation acceleration", "deg/s²"],
      ]
    : [
        ["global_speed_deg_s", "Joint-space speed limit", "deg/s"],
        ["global_accel_deg_s2", "Joint-space acceleration limit", "deg/s²"],
      ];
  return `
    <div class="program-motion-limits">
      <div class="subsection-heading">
        <h4>Step motion limits</h4>
        <p class="hint">Leave a field blank to inherit the current value from Settings.</p>
      </div>
      <div class="program-value-grid">
        ${fields.map(([key, label, unit]) => `
          <label>${label}
            <span class="input-with-unit">
              <input type="number" min="0.001" step="0.1" value="${Number.isFinite(Number(step.settings?.[key])) ? step.settings[key] : ""}" placeholder="${format(defaults[key], 1)}" data-program-field="setting" data-program-setting-key="${key}" ${disabled} />
              <span>${unit}</span>
            </span>
          </label>
        `).join("")}
      </div>
    </div>
  `;
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
  const disabled = programEditingLocked() ? "disabled" : "";
  const isTool = step.type === "tool";
  let valueFields = "";
  if (isTool) {
    const tool = activeProgramTool();
    const actionOptions = programToolActions();
    if (!actionOptions.some((option) => option.value === step.action)) {
      actionOptions.push({ value: step.action, label: String(step.action || "Unknown action") });
    }
    valueFields = `
      <div class="program-value-grid">
        <label>Action
          <select data-program-field="tool_action" ${disabled}>
            ${actionOptions.map((option) => `<option value="${option.value}" ${step.action === option.value ? "selected" : ""}>${escapeHtml(option.label)}</option>`).join("")}
          </select>
        </label>
        <label>Settle time
          <span class="input-with-unit"><input type="number" min="0" step="10" value="${Number.isFinite(Number(step.settle_ms)) ? step.settle_ms : 150}" data-program-field="settle_ms" ${disabled} /><span>ms</span></span>
        </label>
        ${step.action === "set" ? `
          <label>Tool value
            <span class="input-with-unit"><input type="number" min="0" max="1" step="0.01" value="${Number.isFinite(Number(step.value)) ? step.value : 0.5}" data-program-field="tool_value" ${disabled} /><span>0..1</span></span>
          </label>
        ` : ""}
        <div class="program-source-readout"><span>Configured tool</span><strong>${escapeHtml(tool.label)}</strong></div>
      </div>
      <p class="field-help">This action runs at its exact position in the sequence. Hardware execution still requires the configured tool, connection, and Armed state.</p>
    `;
  } else if (step.type === "joint") {
    valueFields = `<div class="program-value-grid">
      ${(step.angles_deg || []).map((value, jointIndex) => `
        <label>J${jointIndex + 1} <span class="input-with-unit"><input type="number" step="0.1" value="${Number.isFinite(Number(value)) ? value : ""}" data-program-field="angle" data-program-value-index="${jointIndex}" ${disabled} /><span>deg</span></span></label>
      `).join("")}
    </div>`;
  } else {
    valueFields = `<div class="program-value-grid">
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
  }

  elements.programInspector.innerHTML = `
    <div class="program-inspector-grid">
      <label>Label <input type="text" value="${escapeHtml(step.label || "")}" data-program-field="label" ${disabled} /></label>
      ${isTool ? `<div class="program-inspector-type"><span>Type</span><strong>End-effector action</strong></div>` : `
        <label>Move mode
          <select data-program-field="mode" ${step.type === "joint" || programEditingLocked() ? "disabled" : ""}>
            <option value="joint" ${step.mode === "joint" ? "selected" : ""}>Joint move</option>
            <option value="linear" ${step.mode === "linear" ? "selected" : ""}>Linear Cartesian move</option>
          </select>
        </label>
      `}
    </div>
    ${isTool ? "" : `<div class="program-inspector-type"><span>Type</span><strong>${step.type === "joint" ? "Joint target" : "Cartesian target"}</strong></div>`}
    ${valueFields}
    ${programMotionLimitFields(step, disabled)}
    ${isTool ? "" : `
      <details class="program-advanced">
        <summary>Advanced</summary>
        <div class="program-inspector-grid">
          <label>IK branch
            <select data-program-field="branch" ${step.type === "joint" || programEditingLocked() ? "disabled" : ""}>
              <option value="auto" ${step.branch === "auto" ? "selected" : ""}>Auto nearest</option>
              <option value="elbow_up" ${step.branch === "elbow_up" ? "selected" : ""}>Elbow up</option>
              <option value="elbow_down" ${step.branch === "elbow_down" ? "selected" : ""}>Elbow down</option>
            </select>
          </label>
          <div class="program-source-readout"><span>Source</span><strong>${escapeHtml(step.source_label || step.source || "manual")}</strong></div>
        </div>
      </details>
      <div class="program-target-test">
        <div>
          <strong>Test selected target</strong>
          <span>Preview or move directly from the robot's current reported pose. Other program steps are not included.</span>
        </div>
        <div class="program-target-test-actions">
          <button type="button" class="ghost" data-program-target-action="preview">Preview target</button>
          <button type="button" class="primary" data-program-target-action="go">Go to target</button>
        </div>
        <p id="programTargetTestStatus" class="program-target-test-status" aria-live="polite"></p>
      </div>
    `}
    <div class="program-inspector-actions">
      <button type="button" class="ghost" data-program-inspector-action="duplicate" ${disabled}>Duplicate step</button>
      <button type="button" class="ghost danger-text" data-program-inspector-action="delete" ${disabled}>Delete step</button>
    </div>
  `;
  updateProgramTargetControls();
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
  if (state.programReadOnly) return "template";
  return "editing";
}

function renderProgramWorkflowStatus() {
  const workflowState = programWorkflowState();
  const labels = {
    empty: "Empty",
    template: "Template",
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
    empty: "Choose a demo, load a saved program, or create a new one.",
    template: `${state.programName || "This demo"} is read-only. Preview it as-is, or save a copy before editing.`,
    editing: `${state.programName || "Untitled program"} is ready for editing. Build the sequence, then move to Preview.`,
    needs_preview: `${state.programLastEditReason || "The sequence changed"}. Preview the current version before execution.`,
    preview_valid: "This exact sequence passed preview and is ready when the robot safety gates are satisfied.",
    running: `Executing ${diagnostics.current_waypoint_total ? `waypoint ${diagnostics.current_waypoint_index || 0} of ${diagnostics.current_waypoint_total}` : "the validated program"}.`,
    failed: state.programExecutionError || state.programPreviewFailure?.error || "Preview or execution failed. Inspect the affected step below.",
  };
  elements.programStatusDetail.textContent = details[workflowState];

  const completed = new Set();
  if (state.programActiveId || state.programDirty || state.programWaypoints.length) completed.add("library");
  if (state.programWaypoints.length) completed.add("build");
  if (["preview_valid", "running"].includes(workflowState)) completed.add("preview");
  if (workflowState === "running") completed.add("run");
  elements.programWorkflow.querySelectorAll("[data-program-stage]").forEach((stage) => {
    stage.classList.toggle("active", stage.dataset.programStage === state.activeProgramStage);
    stage.classList.toggle("done", completed.has(stage.dataset.programStage));
  });
}

function renderProgramRunMonitor(robotState = state.robotState) {
  if (!elements.programRunMonitor) return;
  const diagnostics = robotState?.motion_diagnostics || {};
  const trajectory = latestProgramTrajectory();
  const result = String(diagnostics.result || robotState?.motion_execution_state || "").toLowerCase();
  const status = state.programExecutionActive
    ? (result || "starting")
    : state.programExecutionFailed
      ? "failed"
      : programPreviewIsFresh()
        ? "ready"
        : "waiting for preview";
  const progress = clamp(Number(diagnostics.progress_ratio ?? 0), 0, 1);
  const activeStep = Number(diagnostics.active_step_index || 0);
  const totalSteps = Number(diagnostics.active_step_total || state.programWaypoints.length || 0);
  elements.programRunMonitor.innerHTML = `
    <div class="program-run-grid">
      <div class="program-run-metric"><span>Program</span><strong>${escapeHtml(state.programName || "Untitled program")}</strong><small>${state.programReadOnly ? "built-in template" : state.programActiveId ? "saved user program" : "unsaved draft"}</small></div>
      <div class="program-run-metric"><span>Status</span><strong>${escapeHtml(status)}</strong><small>${state.programExecutionError ? escapeHtml(state.programExecutionError) : `${format(progress * 100, 0)}% complete`}</small></div>
      <div class="program-run-metric"><span>Step</span><strong>${activeStep && totalSteps ? `${activeStep}/${totalSteps}` : "—"}</strong><small>${escapeHtml(diagnostics.active_step_label || "No active step")}</small></div>
      <div class="program-run-metric"><span>Planned duration</span><strong>${Number.isFinite(Number(trajectory.duration_s)) ? `${format(trajectory.duration_s, 2)} s` : "—"}</strong><small>${trajectory.waypoint_count || 0} path points</small></div>
    </div>
  `;
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
  const actionCount = state.programWaypoints.filter((step) => step.enabled !== false && step.type === "tool").length;
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
  const motionSummaryHtml = state.programPreview
    ? motionContractHtml(state.programPreview, trajectory)
    : "";
  elements.programPreviewSummary.innerHTML = `
    <div class="program-summary-grid">
      <div><span>Steps</span><strong>${stepCount}</strong></div>
      <div><span>Moves / actions</span><strong>${Math.max(0, moveCount - actionCount)} / ${actionCount}</strong></div>
      <div><span>Duration</span><strong>${Number.isFinite(Number(trajectory.duration_s)) ? `${format(trajectory.duration_s, 2)} s` : "—"}</strong></div>
      <div><span>Preview</span><strong class="program-summary-status ${status.toLowerCase().replace(" ", "-")}">${status}</strong></div>
    </div>
    <div class="program-summary-messages">
      ${motionSummaryHtml ? `<div class="path-summary compact-summary">${motionSummaryHtml}</div>` : ""}
      ${stale ? `<div class="program-summary-message warning"><strong>Preview is stale</strong><span>${escapeHtml(state.programLastEditReason || "The sequence changed after preview.")}</span></div>` : ""}
      ${errors.slice(0, 5).map((error) => `<div class="program-summary-message error"><strong>Needs attention</strong><span>${escapeHtml(error)}</span></div>`).join("")}
      ${calibrationWarnings.slice(0, 3).map((warning) => `<div class="program-summary-message warning"><strong>Warning</strong><span>${escapeHtml(warning)}</span></div>`).join("")}
      ${fresh && !errors.length ? `<div class="program-summary-message success"><strong>Preview matches</strong><span>${trajectory.waypoint_count || 0} planned path points are ready for execution.</span></div>` : ""}
      ${state.programSavedPlanStatus ? `<div class="program-summary-message ${fresh ? "success" : "neutral"}"><strong>Saved plan</strong><span>${escapeHtml(state.programSavedPlanStatus)}</span></div>` : ""}
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

function programPlaybackTrajectory() {
  return programPreviewIsFresh() ? state.programPreview?.trajectory || null : null;
}

function programPlaybackDuration() {
  return Math.max(0, Number(programPlaybackTrajectory()?.duration_s) || 0);
}

function programPlaybackAnglesAt(elapsedS) {
  const trajectory = programPlaybackTrajectory();
  const waypoints = trajectory?.waypoints || [];
  const times = trajectory?.time_from_start_s || [];
  if (!waypoints.length) return null;
  if (waypoints.length === 1 || elapsedS <= Number(times[0] || 0)) return waypoints[0].map(Number);
  const finalTime = Number(times[times.length - 1] || 0);
  if (elapsedS >= finalTime) return waypoints[waypoints.length - 1].map(Number);
  let nextIndex = 1;
  while (nextIndex < times.length && Number(times[nextIndex]) < elapsedS) nextIndex += 1;
  const previousIndex = Math.max(0, nextIndex - 1);
  const startTime = Number(times[previousIndex] || 0);
  const endTime = Number(times[nextIndex] || startTime);
  const ratio = endTime > startTime ? Math.min(1, Math.max(0, (elapsedS - startTime) / (endTime - startTime))) : 1;
  return waypoints[previousIndex].map((value, jointIndex) => {
    const start = Number(value);
    const end = Number(waypoints[nextIndex]?.[jointIndex] ?? start);
    return start + (end - start) * ratio;
  });
}

function programPlaybackStepAt(elapsedS) {
  const results = (programPlaybackTrajectory()?.step_results || []).filter((result) => result.enabled !== false);
  if (!results.length) return null;
  return results.find((result) => {
    const start = Number(result.start_time_s || 0);
    const end = Number(result.end_time_s ?? start);
    return elapsedS >= start && (elapsedS < end || (end === start && Math.abs(elapsedS - start) < 0.001));
  }) || results[results.length - 1];
}

function updateProgramPlaybackUi() {
  if (!elements.programPlayback) return;
  const duration = programPlaybackDuration();
  const fresh = Boolean(programPlaybackTrajectory());
  const elapsed = Math.min(duration, Math.max(0, Number(state.programPlaybackElapsedS) || 0));
  const ratio = duration > 0 ? elapsed / duration : 0;
  const activeStep = fresh ? programPlaybackStepAt(elapsed) : null;
  elements.programPlaybackProgress.disabled = !fresh || duration <= 0;
  elements.programPlaybackProgress.value = String(Math.round(ratio * 1000));
  elements.programPlaybackToggle.disabled = !fresh || duration <= 0;
  elements.programPlaybackRestart.disabled = !fresh || duration <= 0;
  elements.programPlaybackRate.disabled = !fresh || duration <= 0;
  elements.programPlaybackRate.value = String(state.programPlaybackRate);
  elements.programPlaybackToggle.textContent = state.programPlaybackPlaying ? "Pause" : elapsed >= duration && duration > 0 ? "Replay" : "Play";
  elements.programPlaybackTime.textContent = `${format(elapsed, 2)} / ${format(duration, 2)} s`;
  elements.programPlaybackStep.textContent = !fresh
    ? "Preview the program to animate its timing."
    : activeStep
      ? `Step ${Number(activeStep.index) + 1}: ${activeStep.label || activeStep.type}${activeStep.type === "tool" ? ` · ${String(activeStep.mode || activeStep.action || "action")}` : ""}`
      : "Ready to play the planned movement.";
}

function seekProgramPlayback(elapsedS) {
  const duration = programPlaybackDuration();
  state.programPlaybackElapsedS = Math.min(duration, Math.max(0, Number(elapsedS) || 0));
  const angles = programPlaybackAnglesAt(state.programPlaybackElapsedS);
  if (angles && state.view) {
    state.viewPreviewSource = "program";
    state.view.setPreviewAngles(angles);
  }
  updateProgramPlaybackUi();
}

function stopProgramPlayback(options = {}) {
  if (state.programPlaybackFrame !== null) cancelAnimationFrame(state.programPlaybackFrame);
  state.programPlaybackFrame = null;
  state.programPlaybackPlaying = false;
  if (options.reset) state.programPlaybackElapsedS = 0;
  updateProgramPlaybackUi();
}

function programPlaybackTick(nowMs) {
  if (!state.programPlaybackPlaying) return;
  const duration = programPlaybackDuration();
  state.programPlaybackElapsedS = Math.min(
    duration,
    Math.max(0, (nowMs - state.programPlaybackStartedAtMs) / 1000 * state.programPlaybackRate),
  );
  seekProgramPlayback(state.programPlaybackElapsedS);
  if (state.programPlaybackElapsedS >= duration) {
    state.programPlaybackPlaying = false;
    state.programPlaybackFrame = null;
    updateProgramPlaybackUi();
    return;
  }
  state.programPlaybackFrame = requestAnimationFrame(programPlaybackTick);
}

function startProgramPlayback(options = {}) {
  const duration = programPlaybackDuration();
  if (!duration) return;
  if (options.restart || state.programPlaybackElapsedS >= duration) state.programPlaybackElapsedS = 0;
  if (state.programPlaybackFrame !== null) cancelAnimationFrame(state.programPlaybackFrame);
  state.programPlaybackPlaying = true;
  state.programPlaybackStartedAtMs = performance.now() - (state.programPlaybackElapsedS / state.programPlaybackRate) * 1000;
  seekProgramPlayback(state.programPlaybackElapsedS);
  state.programPlaybackFrame = requestAnimationFrame(programPlaybackTick);
}

function renderProgramBuilder(options = {}) {
  renderProgramLibrary();
  renderProgramSourceOptions();
  renderProgramList();
  if (options.inspector !== false) renderProgramInspector();
  renderProgramWorkflowStatus();
  renderProgramPreviewSummary();
  renderProgramRunMonitor();
  setActiveProgramStage(state.activeProgramStage);
  updateProgramPlaybackUi();
  updateDisabledState();
}

function updateProgramStepFromControl(control) {
  const step = selectedProgramStep();
  if (!step || !control?.dataset?.programField) return;
  const field = control.dataset.programField;
  if (field === "label") step.label = control.value;
  if (field === "mode") step.mode = control.value === "linear" && step.type === "cartesian" ? "linear" : "joint";
  if (field === "branch") step.branch = control.value;
  if (field === "tool_action") {
    step.action = control.value;
    if (step.action === "set" && !Number.isFinite(Number(step.value))) step.value = 0.5;
  }
  if (field === "tool_value") step.value = control.value === "" ? Number.NaN : Number(control.value);
  if (field === "settle_ms") step.settle_ms = control.value === "" ? Number.NaN : Number(control.value);
  if (field === "setting") {
    step.settings = step.settings || {};
    const key = control.dataset.programSettingKey;
    if (control.value === "") delete step.settings[key];
    else step.settings[key] = Number(control.value);
    if (!Object.keys(step.settings).length) delete step.settings;
  }
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

async function previewSelectedProgramTarget() {
  const step = selectedProgramStep();
  const index = selectedProgramIndex();
  if (!step || index < 0 || state.programTargetPreviewPending) return;
  const validation = programStepClientValidation(step, index);
  if (step.enabled === false || !validation.ok) {
    state.programTargetStatus = step.enabled === false
      ? "Enable this step before previewing its target."
      : validation.errors[0] || "The selected target is invalid.";
    updateProgramTargetControls();
    return;
  }

  const revision = state.programRevision;
  state.programTargetPreviewPending = true;
  state.programTargetStatus = "";
  updateProgramTargetControls();
  const payload = await postJson("/api/path/preview", {
    mode: "program",
    branch: step.branch || elements.ikBranchSelect.value,
    settings: pathSettings(),
    waypoints: [clonePlain(step)],
    program_revision: revision,
  });
  state.programTargetPreviewPending = false;
  if (revision !== state.programRevision || selectedProgramStep()?.id !== step.id) {
    clearProgramTargetPreview();
    updateProgramTargetControls();
    return;
  }
  if (payload.ok) {
    stopProgramPlayback({ reset: true });
    renderPreview(payload.preview);
    state.programTargetPreview = {
      previewId: payload.preview.id,
      stepId: step.id,
      programRevision: revision,
      startPoseRevision: payload.preview.start_pose_revision,
    };
    state.programTargetStatus = "";
    state.programPreviewRevision = null;
    state.programLastEditReason = "A Build target preview replaced the full-program preview";
  } else {
    clearProgramTargetPreview();
    state.programTargetStatus = payload.error || "Target preview failed.";
    renderPreviewFailure(payload);
  }
  renderProgramBuilder({ inspector: false });
  updateProgramTargetControls();
}

async function executeSelectedProgramTarget() {
  const step = selectedProgramStep();
  if (!programTargetPreviewIsFresh(step)) {
    state.programTargetStatus = "Preview this exact target from the current robot pose first.";
    updateProgramTargetControls();
    return;
  }
  const gateReason = programMotionGateReason();
  if (gateReason) {
    state.programTargetStatus = gateReason;
    updateProgramTargetControls();
    return;
  }

  const previewId = state.programTargetPreview.previewId;
  state.programTargetMovePending = true;
  state.programTargetStatus = `Starting move to ${step.label || "selected target"}...`;
  updateProgramTargetControls();
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
    clearProgramTargetPreview();
    state.programTargetStatus = `Move accepted for ${step.label || "selected target"}. Watch the HUD and 3D arm for progress.`;
  } else {
    clearProgramTargetPreview();
    state.programTargetStatus = payload.error || "Target move failed.";
    if (/preview|configuration|model|start pose/i.test(payload.error || "")) clearViewPreview();
  }
  if (payload.state) renderState(payload.state);
  renderProgramBuilder({ inspector: false });
  updateProgramTargetControls();
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
  state.programSavedPlanStatus = "";
  renderProgramBuilder({ inspector: false });
  const payload = await postJson("/api/path/preview", {
    mode: "program",
    branch: elements.ikBranchSelect.value,
    settings: pathSettings(),
    waypoints: clonePlain(state.programWaypoints),
    program_revision: revision,
    program_id:
      state.programActiveId && !state.programDirty && !state.programReadOnly
        ? state.programActiveId
        : null,
  });
  state.programPreviewPending = false;
  if (revision !== state.programRevision) {
    renderProgramBuilder();
    return;
  }
  state.programValidationRevision = revision;
  if (payload.ok) {
    clearProgramTargetPreview();
    renderPreview(payload.preview);
    state.programPreview = payload.preview;
    state.programPreviewRevision = revision;
    state.programPreviewFailure = null;
    state.programExecutionFailed = false;
    if (payload.plan_cache?.saved) {
      state.programSavedPlanStatus = "Plan saved automatically for this program and starting pose.";
      const record = activeProgramRecord();
      if (record) {
        record.cached_plan = {
          available: true,
          saved_at: payload.plan_cache.saved_at,
          start_reported_angles_deg: payload.plan_cache.start_reported_angles_deg,
          duration_s: payload.plan_cache.duration_s,
          waypoint_count: payload.plan_cache.waypoint_count,
        };
      }
    } else if (payload.plan_cache?.error) {
      state.programSavedPlanStatus = `Plan is valid but was not saved: ${payload.plan_cache.error}`;
    } else if (!state.programActiveId || state.programDirty || state.programReadOnly) {
      state.programSavedPlanStatus = state.programReadOnly
        ? "Copy this template to save and reuse its compiled plan."
        : "Save the program first to persist this compiled plan.";
    }
    state.programPlaybackElapsedS = 0;
    startProgramPlayback({ restart: true });
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
  stopProgramPlayback();
  state.programExecutionActive = true;
  state.programExecutionAwaitingStart = true;
  state.programExecutionFailed = false;
  state.programExecutionError = "";
  setActiveProgramStage("run");
  renderProgramBuilder({ inspector: false });
  let payload;
  try {
    payload = await postJson("/api/path/execute", {
      preview_id: previewId,
      program_revision: state.programRevision,
    });
  } catch (error) {
    state.programExecutionActive = false;
    state.programExecutionAwaitingStart = false;
    state.programExecutionFailed = true;
    state.programExecutionError = `Could not start program execution: ${error.message || error}`;
    showLocalError(state.programExecutionError);
    renderProgramBuilder({ inspector: false });
    return;
  }
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
  if (result === "failed") {
    state.programExecutionActive = false;
    state.programExecutionAwaitingStart = false;
    state.programExecutionFailed = true;
    state.programExecutionError = diagnostics.error || robotState?.last_error || "Program failed.";
  } else if (result === "stopped") {
    state.programExecutionActive = false;
    state.programExecutionAwaitingStart = false;
    state.programExecutionFailed = false;
    state.programExecutionError = "";
    state.programLastEditReason = "Execution stopped";
  } else if (result === "reached") {
    const activeStepIndex = Number(diagnostics.active_step_index || 0);
    const activeStepTotal = Number(diagnostics.active_step_total || 0);
    if (activeStepTotal > 0 && activeStepIndex < activeStepTotal) return;
    state.programExecutionActive = false;
    state.programExecutionAwaitingStart = false;
    state.programExecutionFailed = false;
    state.programExecutionError = "";
    state.programLastEditReason = "Execution finished";
    void restoreSavedProgramPlan({ stageOnSuccess: "run", quiet: true });
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
    encoders: readEncoderPayload(),
    joints,
    color_profiles: taskColorProfiles(),
    task_destinations: {
      schema_version: 1,
      destinations: taskDestinationsForSave(),
    },
    drop_zones: taskDestinationsForSave(),
    tasks: {
      ...(state.config.tasks || {}),
      color_sorting: {
        ...(state.config.tasks?.color_sorting || {}),
        orientation_policy: elements.orientationPolicySelect?.value || state.config.tasks?.color_sorting?.orientation_policy || "prefer_downward",
      },
    },
    calibration: {
      ...(state.config.calibration || {}),
      measurement_reference: {
        ...(state.config.calibration?.measurement_reference || {}),
        frame: "robot_base",
        workspace_plane_z_mm: readNumber(
          elements.tcpCalWorkspacePlaneZInput,
          state.config.calibration?.measurement_reference?.workspace_plane_z_mm || 0
        ),
        z_reference: "robot_base",
        measured_point: elements.tcpCalMeasuredPointSelect?.value || "active_tcp",
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

async function saveAndSyncHardware() {
  const syncButtons = [elements.syncHardwareBtn, elements.viewportSyncHardwareBtn].filter(Boolean);
  syncButtons.forEach((button) => {
    button.disabled = true;
  });

  try {
    const saved = await saveAllSettings();
    if (!saved) return;
    updateSettingsSaveBar({
      mode: "saving",
      title: "Syncing controller",
      detail: "Sending the saved hardware configuration to the ESP...",
    });
    if (elements.statusPill) elements.statusPill.textContent = "Syncing ESP configuration...";
    const payload = await postJson("/api/hardware/sync");
    if (payload.state) renderState(payload.state);
    updateSettingsSaveBar(
      payload.ok
        ? { title: "All settings saved", detail: payload.message || "Saved locally and synced to the controller." }
        : {
            mode: "error",
            title: "Controller sync failed",
            detail: payload.message || payload.error || "Hardware settings remain saved locally.",
          }
    );
    if (elements.statusPill) {
      elements.statusPill.textContent = payload.ok
        ? payload.message || "ESP configuration synced."
        : payload.message || payload.error || "ESP sync failed.";
    }
  } catch (error) {
    const message = error?.message || String(error);
    showLocalError(`Controller sync failed: ${message}`);
    updateSettingsSaveBar({
      mode: "error",
      title: "Controller sync failed",
      detail: message,
    });
  } finally {
    updateDisabledState();
  }
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

function applyConfig(config, options = {}) {
  const previousConfigId = state.config?.app_version?.running_config_id;
  const nextConfigId = config?.app_version?.running_config_id;
  const replacingConfig = Boolean(state.config && previousConfigId !== nextConfigId);
  state.config = config;
  state.linkDraft = null;
  state.dhDraftRows = null;
  state.taskColorProfilesDraft = clonePlain(config.color_profiles || {});
  state.taskDestinationsDraft = savedTaskDestinations();
  if (state.taskDetectionMinAreaPx == null || replacingConfig) {
    state.taskDetectionMinAreaPx = configuredDetectionMinAreaPx(config.camera || {});
  }
  state.positionLibraryDraft = savedPositionLibraryRecords();
  state.positionLibraryErrors = {};
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
  elements.tcpSpeedInput.value = format(pathDefaults.tcp_speed_mm_s ?? 60, 1);
  elements.phiSpeedInput.value = format(pathDefaults.phi_speed_deg_s ?? 45, 1);
  elements.tcpAccelInput.value = format(pathDefaults.tcp_accel_mm_s2 ?? 360, 1);
  elements.phiAccelInput.value = format(pathDefaults.phi_accel_deg_s2 ?? 240, 1);
  if (elements.cartesianJogSpeedInput) elements.cartesianJogSpeedInput.value = format(pathDefaults.tcp_speed_mm_s ?? 60, 1);
  if (elements.cartesianJogPhiSpeedInput) elements.cartesianJogPhiSpeedInput.value = format(pathDefaults.phi_speed_deg_s ?? 45, 1);
  elements.waypointRateInput.value = format(pathDefaults.waypoint_rate_hz ?? state.config.motion.command_rate_limit_hz, 0);
  elements.cartesianStepInput.value = format(pathDefaults.cartesian_step_mm ?? 10, 0);
  elements.plannerTypeSelect.value = pathDefaults.planner_type || "s_curve";
  elements.jerkPercentInput.value = format(pathDefaults.jerk_percent ?? 25, 0);
  elements.blendPercentInput.value = format(pathDefaults.blend_percent ?? 0, 0);
  if (elements.tcpCalWorkspacePlaneZInput) {
    elements.tcpCalWorkspacePlaneZInput.value = format(
      state.config.calibration?.measurement_reference?.workspace_plane_z_mm ?? 0,
      2
    );
  }
  if (elements.tcpCalMeasuredPointSelect) {
    elements.tcpCalMeasuredPointSelect.value =
      state.config.calibration?.measurement_reference?.measured_point || "active_tcp";
  }
  syncPlannerControls();
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
  syncDetectionTuningControls();
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
  if (replacingConfig && !options.preserveTaskDetections) {
    invalidateTaskDetections("Robot configuration changed - refresh detections");
  }
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
    const payload = await postJson("/api/home", { settings: pathSettings() });
    if (payload.ok) {
      invalidatePendingIkPreview();
      releaseJointControlIntent();
      clearViewPreview();
      state.ikUserEdited = false;
    }
    if (payload.state) renderState(payload.state);
  });
  elements.alignShoulderBtn?.addEventListener("click", async () => {
    const payload = await postJson("/api/encoder/shoulder/align", { settings: pathSettings() });
    if (payload.ok) {
      releaseJointControlIntent();
      state.encoderCalibrationMessage = payload.verification || "shoulder alignment checked";
    } else {
      showLocalError(payload.error || "Shoulder alignment did not run");
    }
    if (payload.state) renderState(payload.state);
  });
  elements.setPoseBtn.addEventListener("click", setCurrentPoseKnown);
  elements.cancelSetPoseBtn?.addEventListener("click", () => setPoseModalVisible(false));
  elements.confirmSetPoseBtn?.addEventListener("click", confirmCurrentPoseKnown);
  elements.setPoseModal?.addEventListener("click", (event) => {
    if (event.target === elements.setPoseModal) setPoseModalVisible(false);
  });
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
    if (state.diagnosticsRenderTimer) window.clearTimeout(state.diagnosticsRenderTimer);
    state.diagnosticsRenderTimer = null;
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
  elements.encoderCalibration?.addEventListener("input", markEncoderDraftDirty);
  elements.encoderCalibration?.addEventListener("change", markEncoderDraftDirty);
  elements.encoderCalibration?.addEventListener("click", (event) => {
    const uiButton = event.target.closest("[data-encoder-ui-action]");
    if (uiButton) {
      event.preventDefault();
      if (uiButton.disabled) return;
      uiButton.disabled = true;
      void handleEncoderUiAction(uiButton.dataset.encoderUiAction).finally(() => {
        uiButton.disabled = false;
      });
      return;
    }
    const actionButton = event.target.closest("[data-encoder-action]");
    if (actionButton) {
      event.preventDefault();
      if (actionButton.disabled) return;
      actionButton.disabled = true;
      void handleEncoderCalibrationAction(actionButton.dataset.encoderAction).finally(() => {
        actionButton.disabled = false;
      });
    }
  });
  [elements.syncHardwareBtn, elements.viewportSyncHardwareBtn].forEach((button) => {
    button?.addEventListener("click", saveAndSyncHardware);
  });
  [
    elements.globalSpeedInput,
    elements.globalAccelInput,
    elements.tcpSpeedInput,
    elements.tcpAccelInput,
    elements.phiSpeedInput,
    elements.phiAccelInput,
    elements.waypointRateInput,
    elements.cartesianStepInput,
    elements.plannerTypeSelect,
    elements.jerkPercentInput,
    elements.blendPercentInput,
  ].forEach((input) => {
    const handleMotionSettingsChange = () => {
      if (input === elements.plannerTypeSelect) syncPlannerControls();
      if (input === elements.tcpSpeedInput && elements.cartesianJogSpeedInput) {
        elements.cartesianJogSpeedInput.value = elements.tcpSpeedInput.value;
      }
      if (input === elements.phiSpeedInput && elements.cartesianJogPhiSpeedInput) {
        elements.cartesianJogPhiSpeedInput.value = elements.phiSpeedInput.value;
      }
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
    elements.workspaceMarginInput,
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
  elements.positionLibraryList?.addEventListener("input", (event) => {
    const input = event.target.closest("[data-position-field]");
    if (!input) return;
    updatePositionLibraryDraft(input.dataset.positionId, input.dataset.positionField, input.value);
  });
  elements.positionLibraryList?.addEventListener("click", (event) => {
    const previewButton = event.target.closest("[data-position-preview]");
    const goButton = event.target.closest("[data-position-go]");
    const duplicateButton = event.target.closest("[data-position-duplicate]");
    const deleteButton = event.target.closest("[data-position-delete]");
    if (previewButton) {
      previewNamedPosition(previewButton.dataset.positionPreview);
      return;
    }
    if (goButton) {
      moveNamedPosition(goButton.dataset.positionGo);
      return;
    }
    if (duplicateButton) {
      duplicatePositionLibraryRecord(duplicateButton.dataset.positionDuplicate);
      return;
    }
    if (deleteButton) {
      const positionId = deleteButton.dataset.positionDelete;
      if (CORE_POSITION_IDS.has(positionId)) return;
      const records = ensurePositionLibraryDraft();
      delete records[positionId];
      delete state.positionLibraryErrors[positionId];
      markPositionLibraryDirty("Position removed. Save Library to persist.");
      renderPositionLibrary();
    }
  });
  elements.addJointPositionBtn?.addEventListener("click", () => addPositionLibraryRecord("joint"));
  elements.addCartesianPositionBtn?.addEventListener("click", () => addPositionLibraryRecord("cartesian"));
  elements.saveCurrentPositionBtn?.addEventListener("click", saveCurrentReportedPosition);
  elements.savePositionLibraryBtn?.addEventListener("click", () => savePositionLibrary());
  elements.resetPositionLibraryBtn?.addEventListener("click", resetPositionLibraryDraft);
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
  elements.saveTaskMappingsBtn?.addEventListener("click", saveTaskMappingEdits);
  elements.discardTaskMappingsBtn?.addEventListener("click", discardTaskMappingEdits);
  [
    elements.taskModeSelect,
    elements.executionStrategySelect,
    elements.cycleConfirmationSelect,
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
    elements.cycleConfirmationSelect,
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
  ].forEach((input) => input?.addEventListener("change", () => {
    if (input === elements.orientationPolicySelect) syncTaskOrientationControls();
    invalidateTaskPreview("Task settings changed");
  }));
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
    const continueButton = event.target.closest("[data-task-continue]");
    if (continueButton) {
      continueTask();
      return;
    }
    const button = event.target.closest("[data-runtime-select]");
    if (!button) return;
    selectRuntimeDetection(button.dataset.runtimeSelect);
  });
  elements.previewTaskBtn.addEventListener("click", previewTask);
  elements.executeTaskBtn.addEventListener("click", executeTask);
  elements.taskStopBtn?.addEventListener("click", stopTask);
  elements.toolValidationBtn?.addEventListener("click", setActiveToolDimensionsValidation);
  elements.viewCameraBtn?.addEventListener("click", () => {
    setCameraPopupVisible(true);
    detectVision();
  });
  elements.detectionMinAreaInput?.addEventListener("change", updateDetectionMinAreaFromInput);
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
  elements.tcpCalFitPhysicalBtn?.addEventListener("click", fitTcpPhysicalModel);
  elements.tcpCalApplyPhysicalBtn?.addEventListener("click", applyTcpPhysicalModel);
  elements.tcpCalSaveManualOffsetsBtn?.addEventListener("click", saveManualTcpCalibrationOffsets);
  elements.tcpCalFitBtn?.addEventListener("click", fitTcpCalibration);
  elements.tcpCalApplyEnableBtn?.addEventListener("click", applyTcpCalibrationEnableState);
  elements.tcpCalibrationTargetList?.addEventListener("click", (event) => {
    const button = event.target.closest("[data-tcp-cal-target]");
    if (!button) return;
    const point = state.tcpCalibrationTargets[Number(button.dataset.tcpCalTarget)];
    if (point) {
      setTcpCalibrationTarget(point.intended_target);
      elements.tcpCalRoleSelect.value = point.recommended_role || "fit";
    }
  });
  elements.tcpCalibrationSamples?.addEventListener("click", (event) => {
    const button = event.target.closest("[data-tcp-cal-delete]");
    if (button) deleteTcpCalibrationSample(button.dataset.tcpCalDelete);
  });
  elements.tcpCalWorkspacePlaneZInput?.addEventListener("input", () => {
    if (elements.tcpCalReferenceConfirmInput) elements.tcpCalReferenceConfirmInput.checked = false;
    markSettingsDirty("calibration", "Measurement reference changed. Save and reconfirm it before calibration motion.");
    renderTcpCalibration();
  });
  elements.tcpCalMeasuredPointSelect?.addEventListener("change", () => {
    if (elements.tcpCalReferenceConfirmInput) elements.tcpCalReferenceConfirmInput.checked = false;
    markSettingsDirty("calibration", "Measured point changed. Save and reconfirm it before calibration motion.");
    renderTcpCalibration();
  });
  elements.tcpCalReferenceConfirmInput?.addEventListener("change", renderTcpCalibration);

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
    const stageButton = event.target.closest("[data-program-stage]");
    if (stageButton) {
      setActiveProgramStage(stageButton.dataset.programStage);
      renderProgramWorkflowStatus();
      return;
    }
    const libraryAction = event.target.closest("[data-program-library-action]");
    if (libraryAction) {
      const programId = libraryAction.dataset.programId;
      const action = libraryAction.dataset.programLibraryAction;
      const program = state.programLibrary.find((item) => item.id === programId);
      if (action === "load" && program) void loadLibraryProgram(program);
      if (action === "copy") copyLibraryProgram(programId);
      if (action === "delete") deleteProgram(programId);
      return;
    }
  });
  elements.newProgramBtn.addEventListener("click", startNewProgram);
  elements.saveProgramBtn.addEventListener("click", () => saveCurrentProgram());
  elements.copyProgramBtn.addEventListener("click", () => saveCurrentProgram({ copy: true }));
  elements.programNameInput.addEventListener("input", () => {
    if (state.programReadOnly) return;
    state.programName = elements.programNameInput.value;
    state.programDirty = true;
    setProgramLibraryStatus("Unsaved changes", "warn");
  });
  elements.programDescriptionInput.addEventListener("input", () => {
    if (state.programReadOnly) return;
    state.programDescription = elements.programDescriptionInput.value;
    state.programDirty = true;
    setProgramLibraryStatus("Unsaved changes", "warn");
  });
  elements.programStepSource.addEventListener("change", () => {
    renderProgramSourceOptions();
    updateDisabledState();
  });
  elements.programSourceItem.addEventListener("change", updateDisabledState);
  elements.addProgramStepBtn.addEventListener("click", () => addProgramStep());
  elements.clearProgramBtn.addEventListener("click", () => {
    if (programEditingLocked()) return;
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
  elements.programPlaybackToggle.addEventListener("click", () => {
    if (state.programPlaybackPlaying) stopProgramPlayback();
    else startProgramPlayback();
  });
  elements.programPlaybackRestart.addEventListener("click", () => startProgramPlayback({ restart: true }));
  elements.programPlaybackRate.addEventListener("change", () => {
    const wasPlaying = state.programPlaybackPlaying;
    const elapsed = state.programPlaybackElapsedS;
    state.programPlaybackRate = Number(elements.programPlaybackRate.value) || 1;
    state.programPlaybackElapsedS = elapsed;
    if (wasPlaying) startProgramPlayback();
    else updateProgramPlaybackUi();
  });
  elements.programPlaybackProgress.addEventListener("input", () => {
    stopProgramPlayback();
    seekProgramPlayback(programPlaybackDuration() * (Number(elements.programPlaybackProgress.value) / 1000));
  });
  elements.executeProgramBtn.addEventListener("click", executeProgram);
  elements.stopProgramBtn.addEventListener("click", async () => {
    const stoppingProgram = state.programExecutionActive;
    let payload;
    try {
      payload = await postJson("/api/stop");
    } catch (error) {
      showLocalError(`Could not stop motion: ${error.message || error}`);
      return;
    }
    if (payload.ok && stoppingProgram) {
      state.programExecutionActive = false;
      state.programExecutionAwaitingStart = false;
      state.programExecutionFailed = false;
      state.programExecutionError = "";
      state.programLastEditReason = "Execution stopped";
    }
    if (payload.state) renderState(payload.state);
    renderProgramBuilder({ inspector: false });
  });
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
    const targetAction = event.target.closest("[data-program-target-action]")?.dataset.programTargetAction;
    if (targetAction === "preview") {
      previewSelectedProgramTarget();
      return;
    }
    if (targetAction === "go") {
      executeSelectedProgramTarget();
      return;
    }
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
  await loadProgramLibrary();
  bindActions();
  await postJson("/api/live-motion", { enabled: false });
  await checkAppVersion();
  state.versionTimer = window.setInterval(checkAppVersion, 15000);
  connectWebSocket();
  await loadWorkspaceCalibrationStatus();
}

init();
