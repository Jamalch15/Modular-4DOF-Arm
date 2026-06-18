import { RobotView } from "/static/robot_view.js?v=20260618-settings-revamp-3";

const state = {
  config: null,
  robotState: null,
  ws: null,
  commandTimer: null,
  pendingAngles: null,
  draftAngles: null,
  commandedAngles: null,
  lastSentAngles: null,
  view: null,
  activeTab: "joint",
  previewId: null,
  latestPreview: null,
  previewAngles: null,
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
  hardwareDraftDirty: false,
  settingsDirtyScopes: new Set(),
  taskPreviewId: null,
  selectedSerialPort: null,
  latestDetections: [],
  cameraTimer: null,
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
  aprilTagBusy: false,
  aprilTagStatus: null,
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
  programWaypointType: $("#programWaypointType"),
  programMoveMode: $("#programMoveMode"),
  addCurrentJointWaypointBtn: $("#addCurrentJointWaypointBtn"),
  addIkWaypointBtn: $("#addIkWaypointBtn"),
  clearProgramBtn: $("#clearProgramBtn"),
  previewProgramBtn: $("#previewProgramBtn"),
  executeProgramBtn: $("#executeProgramBtn"),
  programList: $("#programList"),
  programStatus: $("#programStatus"),
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
  previewTaskBtn: $("#previewTaskBtn"),
  executeTaskBtn: $("#executeTaskBtn"),
  taskStatus: $("#taskStatus"),
  taskSummary: $("#taskSummary"),
  detectVisionBtn: $("#detectVisionBtn"),
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
  aprilTagStatus: $("#aprilTagStatus"),
  cameraSourceInput: $("#cameraSourceInput"),
  cameraWidthInput: $("#cameraWidthInput"),
  cameraHeightInput: $("#cameraHeightInput"),
  cameraFxInput: $("#cameraFxInput"),
  cameraFyInput: $("#cameraFyInput"),
  cameraCxInput: $("#cameraCxInput"),
  cameraCyInput: $("#cameraCyInput"),
  cameraDistortionInput: $("#cameraDistortionInput"),
  saveCameraIntrinsicsBtn: $("#saveCameraIntrinsicsBtn"),
  resetAprilTagBtn: $("#resetAprilTagBtn"),
  captureAprilTagBtn: $("#captureAprilTagBtn"),
  collectAprilTagBtn: $("#collectAprilTagBtn"),
  saveAprilTagBtn: $("#saveAprilTagBtn"),
  verifyAprilTagBtn: $("#verifyAprilTagBtn"),
  aprilTagFrame: $("#aprilTagFrame"),
  aprilTagPlaceholder: $("#aprilTagPlaceholder"),
  aprilTagMetrics: $("#aprilTagMetrics"),
  aprilTagDetections: $("#aprilTagDetections"),
};

function format(value, decimals = 1) {
  return Number(value || 0).toFixed(decimals);
}

function readNumber(input, fallback = 0) {
  const value = Number(input?.value);
  return Number.isFinite(value) ? value : fallback;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
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

const robotSettingsScopes = new Set(["geometry", "joints", "motion", "tooling", "hardware"]);

function savedSettingsDetail() {
  if (state.robotState?.simulation) return "Saved locally. Controller sync is not required in simulation.";
  if (!state.robotState?.connected) return "Saved locally. Connect the controller to sync hardware settings.";
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
  elements.hardwareStatus.innerHTML = `
    <div class="log-line"><span>Coverage</span><code>${robotState.hardware_mode || "simulated"} (${robotState.hardware_enabled_axes || "0000"})</code></div>
    <div class="log-line"><span>Sync</span><code>${robotState.config_sync_status || "unknown"}</code></div>
    <div class="log-line"><span>Axes</span><code>${axisText || "-"}</code></div>
    <div class="log-line"><span>Message</span><code>${robotState.config_sync_message || "-"}</code></div>
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

function renderOperatorPanels() {
  if (!state.config) return;
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
    Object.keys(state.config.drop_zones || {}).forEach((name) => {
      const option = document.createElement("option");
      option.value = name;
      option.textContent = name;
      elements.dropZoneSelect.appendChild(option);
    });
  }

  if (elements.sortColorSelect) elements.sortColorSelect.innerHTML = "";
  if (elements.visionProfileList) elements.visionProfileList.innerHTML = "";
  Object.entries(state.config.color_profiles || {}).forEach(([name, profile]) => {
    if (elements.sortColorSelect) {
      const option = document.createElement("option");
      option.value = name;
      option.textContent = name;
      elements.sortColorSelect.appendChild(option);
    }
    if (elements.visionProfileList) {
      const line = document.createElement("div");
      line.className = "log-line";
      line.innerHTML = `<span>${name}</span><code>${profile.enabled === false ? "off" : "on"} -> ${profile.drop_zone || "-"}</code>`;
      elements.visionProfileList.appendChild(line);
    }
  });
  renderToolControls();
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
  elements.diagnosticsSummary.innerHTML = `
    <div class="log-line"><span>Pose source</span><code>${robotState.pose_source || "unknown"}</code></div>
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
  elements.taskSummary.innerHTML = `
    <div class="log-line"><span>Steps</span><code>${steps.length}</code></div>
    <div class="log-line"><span>Moves</span><code>${sequence?.waypoints?.length || 0}</code></div>
    <div class="log-line"><span>Duration</span><code>${format(trajectory.duration_s, 2)} s</code></div>
    <div class="log-line"><span>Drop zone</span><code>${sequence?.drop_zone || "-"}</code></div>
  `;
}

async function previewTask() {
  elements.taskStatus.textContent = "Previewing...";
  const task = elements.taskModeSelect.value;
  const objectTarget = ikTargetPayload();
  const request =
    task === "color_sorting"
      ? {
          task,
          detections: state.latestDetections.length
            ? state.latestDetections
            : [{ color: elements.sortColorSelect.value, robot: objectTarget, ok: true }],
          settings: pathSettings(),
          branch: elements.ikBranchSelect.value,
        }
      : {
          task,
          object_target: objectTarget,
          drop_zone: elements.dropZoneSelect.value,
          settings: pathSettings(),
          branch: elements.ikBranchSelect.value,
        };
  const payload = await postJson("/api/task/preview", request);
  if (payload.ok) {
    renderPreview(payload.preview);
    state.taskPreviewId = payload.preview_id;
    renderTaskSummary(payload.sequence, payload.preview);
    elements.executeTaskBtn.disabled = false;
    elements.taskStatus.textContent = "Preview ready";
  } else {
    renderPreviewFailure(payload);
    elements.taskStatus.textContent = payload.error || "Task preview failed";
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
    state.ikUserEdited = false;
  }
  if (payload.state) renderState(payload.state);
  else syncJointControls();
  updateDisabledState();
  elements.taskStatus.textContent = payload.ok ? "Task running" : payload.error || "Task failed";
  await refreshDiagnostics();
}

async function detectVision() {
  elements.visionSummary.innerHTML = `<div class="log-line"><span>Status</span><code>Detecting...</code></div>`;
  const payload = await fetch("/api/vision/frame").then((response) => response.json());
  elements.visionSummary.innerHTML = "";
  if (!payload.ok) {
    elements.visionSummary.innerHTML = `<div class="log-line"><span>Error</span><code>${payload.error || "-"}</code></div>`;
    if (elements.cameraPlaceholder) elements.cameraPlaceholder.hidden = false;
    return;
  }
  state.latestDetections = (payload.detections || []).filter((detection) => detection.ok);
  if (payload.image_b64 && elements.cameraFrame) {
    elements.cameraFrame.src = payload.image_b64;
    elements.cameraFrame.hidden = false;
    if (elements.cameraPlaceholder) elements.cameraPlaceholder.hidden = true;
  }
  renderDetectionList(payload.detections || []);
  if (state.view) state.view.setObjectDetections(state.latestDetections);
  (payload.detections || []).forEach((detection) => {
    const line = document.createElement("div");
    line.className = "log-line";
    const center = detection.center_px ? `px ${format(detection.center_px.x, 0)}, ${format(detection.center_px.y, 0)}` : detection.reason || "-";
    const robot = detection.robot ? ` robot ${format(detection.robot.x_mm)}, ${format(detection.robot.y_mm)}` : "";
    line.innerHTML = `<span>${detection.color}</span><code>${detection.ok ? center + robot : detection.reason}</code>`;
    elements.visionSummary.appendChild(line);
  });
  await refreshDiagnostics();
}

function setAprilTagBusy(busy, status = null) {
  state.aprilTagBusy = Boolean(busy);
  [
    elements.saveCameraIntrinsicsBtn,
    elements.resetAprilTagBtn,
    elements.captureAprilTagBtn,
    elements.collectAprilTagBtn,
    elements.saveAprilTagBtn,
    elements.verifyAprilTagBtn,
  ].forEach((button) => {
    if (button) button.disabled = state.aprilTagBusy;
  });
  if (status && elements.aprilTagStatus) elements.aprilTagStatus.textContent = status;
}

function renderCameraIntrinsics(camera = state.config?.camera || {}) {
  const resolution = camera.resolution || {};
  const intrinsics = camera.intrinsics || {};
  if (elements.cameraSourceInput) elements.cameraSourceInput.value = String(camera.source_index ?? 0);
  if (elements.cameraWidthInput) elements.cameraWidthInput.value = String(resolution.width ?? 1280);
  if (elements.cameraHeightInput) elements.cameraHeightInput.value = String(resolution.height ?? 720);
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
  const fx = readNumber(elements.cameraFxInput, NaN);
  const fy = readNumber(elements.cameraFyInput, NaN);
  const cx = readNumber(elements.cameraCxInput, NaN);
  const cy = readNumber(elements.cameraCyInput, NaN);
  if (![fx, fy, cx, cy].every(Number.isFinite) || fx <= 0 || fy <= 0) {
    throw new Error("Enter valid positive fx/fy and finite cx/cy camera intrinsics.");
  }
  if (![4, 5, 8, 12, 14].includes(distortion.length)) {
    throw new Error("Distortion must contain 4, 5, 8, 12, or 14 comma-separated values.");
  }
  camera.source_index = Math.max(0, Math.round(readNumber(elements.cameraSourceInput, 0)));
  camera.resolution = {
    width: Math.max(1, Math.round(readNumber(elements.cameraWidthInput, 1280))),
    height: Math.max(1, Math.round(readNumber(elements.cameraHeightInput, 720))),
  };
  camera.intrinsics = {
    ...(camera.intrinsics || {}),
    source: "manual",
    fx_px: fx,
    fy_px: fy,
    cx_px: cx,
    cy_px: cy,
    camera_matrix: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
    distortion_coefficients: distortion,
  };
  return camera;
}

async function saveCameraIntrinsics() {
  setAprilTagBusy(true, "Saving intrinsics...");
  try {
    const payload = await postJson("/api/vision/settings", { camera: cameraSettingsDraft() });
    if (payload.ok && payload.config) {
      applyConfig(payload.config);
      elements.aprilTagStatus.textContent = "Intrinsics saved";
      clearSettingsDirty("camera");
    } else if (elements.aprilTagStatus) {
      elements.aprilTagStatus.textContent = payload.error || "Could not save intrinsics";
    }
  } catch (error) {
    showLocalError(error?.message || String(error));
    elements.aprilTagStatus.textContent = "Invalid intrinsics";
  } finally {
    setAprilTagBusy(false);
  }
}

function renderAprilTagDetections(detections = []) {
  if (!elements.aprilTagDetections) return;
  elements.aprilTagDetections.innerHTML = "";
  if (!detections.length) {
    const empty = document.createElement("div");
    empty.className = "program-item";
    empty.textContent = "No tags detected.";
    elements.aprilTagDetections.appendChild(empty);
    return;
  }
  detections.forEach((detection) => {
    const item = document.createElement("div");
    item.className = `program-item ${detection.configured ? "" : "invalid"}`;
    const center = detection.center_px || {};
    item.innerHTML = `
      <div class="program-title"><span>Tag ${detection.id}</span><span>${detection.configured ? "configured" : "unknown"}</span></div>
      <code>px ${format(center.x, 1)}, ${format(center.y, 1)} - area ${format(detection.area_px, 0)}</code>
    `;
    elements.aprilTagDetections.appendChild(item);
  });
}

function renderAprilTagCalibration(payload = {}) {
  state.aprilTagStatus = payload;
  const result = payload.result || payload.live_result || payload.saved_result || null;
  const session = payload.session || {};
  const metrics = result?.metrics || {};
  const pose = result?.camera_to_robot || {};
  const position = pose.position_mm || [];
  const comparison = payload.comparison || {};
  const accepted = Boolean(result?.accepted);
  const sessionResult = result?.frames_used != null;
  const enoughSamples = !sessionResult || result?.minimum_samples_met !== false;
  const enoughRequiredTagSamples = !sessionResult || result?.required_tag_samples_met !== false;
  const saveReady = accepted && enoughSamples && enoughRequiredTagSamples;
  const planarOnly = Boolean(result?.planar?.ok && !result?.ok);
  if (elements.aprilTagStatus) {
    elements.aprilTagStatus.textContent = payload.comparison && accepted
      ? `Verified ${format(metrics.confidence, 2)}`
      : saveReady
        ? `Ready ${format(metrics.confidence, 2)}`
        : accepted
          ? "Pose good; collect all samples"
      : planarOnly
        ? "Planar only"
        : result?.error || "Uncalibrated";
  }
  if (elements.aprilTagMetrics) {
    elements.aprilTagMetrics.innerHTML = `
      <div class="log-line"><span>Samples</span><code>${session.frame_count || result?.frames_used || 0} / ${session.minimum_samples || 0}</code></div>
      <div class="log-line"><span>Tags</span><code>${(result?.tags_used || session.tag_ids || []).join(", ") || "-"}</code></div>
      <div class="log-line"><span>Tag samples</span><code>${Object.entries(result?.tag_observation_counts || session.tag_observation_counts || {}).map(([id, count]) => `${id}:${count}`).join(" ") || "-"}</code></div>
      <div class="log-line"><span>Reprojection</span><code>${result?.ok ? `${format(metrics.reprojection_rmse_px, 2)} px RMSE` : "-"}</code></div>
      <div class="log-line"><span>Camera XYZ</span><code>${position.length ? position.map((value) => format(value, 1)).join(", ") + " mm" : "-"}</code></div>
      <div class="log-line"><span>Tilt</span><code>${result?.ok ? `${format(metrics.tilt_from_down_deg, 1)} deg from down` : "-"}</code></div>
      <div class="log-line"><span>Verify delta</span><code>${comparison.position_delta_mm != null ? `${format(comparison.position_delta_mm, 1)} mm / ${format(comparison.orientation_delta_deg, 2)} deg` : "-"}</code></div>
      <div class="log-line"><span>Message</span><code>${result?.error || (accepted ? "quality checks passed" : "collect samples and configure intrinsics")}</code></div>
    `;
  }
  if (payload.image_b64 && elements.aprilTagFrame) {
    elements.aprilTagFrame.src = payload.image_b64;
    elements.aprilTagFrame.hidden = false;
    if (elements.aprilTagPlaceholder) elements.aprilTagPlaceholder.hidden = true;
  }
  renderAprilTagDetections(payload.detections || []);
  const settings = payload.settings || state.config?.camera?.calibration?.apriltag || {};
  if (state.view) state.view.setAprilTagCalibration({ settings, result });
}

async function loadAprilTagStatus() {
  const response = await fetch("/api/vision/apriltag/status");
  const payload = await response.json();
  if (payload.ok) renderAprilTagCalibration(payload);
}

async function resetAprilTagCalibration() {
  setAprilTagBusy(true, "Resetting...");
  try {
    const payload = await postJson("/api/vision/apriltag/reset");
    if (payload.ok) renderAprilTagCalibration(payload);
    else if (elements.aprilTagStatus) elements.aprilTagStatus.textContent = payload.error || "Reset failed";
  } finally {
    setAprilTagBusy(false);
  }
}

async function captureAprilTags(sampleCount = 1, accumulate = true) {
  setAprilTagBusy(true, sampleCount > 1 ? "Collecting..." : "Capturing...");
  try {
    const payload = await postJson("/api/vision/apriltag/capture", {
      sample_count: sampleCount,
      sample_interval_ms: 80,
      accumulate,
    });
    if (payload.ok) renderAprilTagCalibration(payload);
    else if (elements.aprilTagStatus) elements.aprilTagStatus.textContent = payload.error || "Capture failed";
  } finally {
    setAprilTagBusy(false);
  }
}

async function saveAprilTagCalibration() {
  setAprilTagBusy(true, "Saving pose...");
  try {
    const payload = await postJson("/api/vision/apriltag/save", { require_all_tags: true });
    if (payload.ok) {
      if (payload.config) applyConfig(payload.config);
      renderAprilTagCalibration(payload);
    } else {
      renderAprilTagCalibration(payload);
    }
  } finally {
    setAprilTagBusy(false);
  }
}

async function verifyAprilTagCalibration() {
  setAprilTagBusy(true, "Verifying...");
  try {
    const payload = await postJson("/api/vision/apriltag/verify", { accumulate: false });
    if (payload.ok) renderAprilTagCalibration(payload);
    else if (elements.aprilTagStatus) elements.aprilTagStatus.textContent = payload.error || "Verification failed";
  } finally {
    setAprilTagBusy(false);
  }
}

function renderDetectionList(detections) {
  if (!elements.detectionList) return;
  elements.detectionList.innerHTML = "";
  if (!detections.length) {
    const empty = document.createElement("div");
    empty.className = "program-item";
    empty.textContent = "No detections yet.";
    elements.detectionList.appendChild(empty);
    return;
  }
  detections.forEach((detection, index) => {
    const item = document.createElement("div");
    item.className = `program-item ${detection.ok ? "" : "invalid"}`;
    const robot = detection.robot ? `robot ${format(detection.robot.x_mm)}, ${format(detection.robot.y_mm)}` : "uncalibrated";
    item.innerHTML = `
      <div class="program-title"><span>${index + 1}. ${detection.color || "object"}</span><span>${detection.ok ? "ready" : "ignored"}</span></div>
      <code>${detection.ok ? robot : detection.reason || "not usable"}</code>
    `;
    elements.detectionList.appendChild(item);
  });
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
  if (userEdited) state.ikUserEdited = true;
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

function cartesianJogPayload(dtS = null) {
  return {
    ...state.cartesianJogVelocity,
    dt_s: dtS,
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
  const now = performance.now();
  const previous = state.cartesianJogLastSentMs || now;
  const dtS = clamp((now - previous) / 1000, 0.01, 0.08);
  state.cartesianJogLastSentMs = now;
  state.cartesianJogInFlight = true;
  state.cartesianJogQueued = false;
  try {
    const payload = await postJson("/api/cartesian-jog", cartesianJogPayload(dtS));
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
  elements.hardwareArmToggle.disabled = !state.robotState?.connected || state.robotState?.simulation;
  elements.executeIkBtn.disabled = !state.previewId || !enabled;
  elements.executeProgramBtn.disabled = !state.previewId || !enabled || state.latestPreview?.mode !== "program";
  if (elements.cartesianJogToggle) elements.cartesianJogToggle.disabled = !enabled;
  if (elements.cartesianJogSpeedInput) elements.cartesianJogSpeedInput.disabled = !enabled;
  if (elements.cartesianJogPhiSpeedInput) elements.cartesianJogPhiSpeedInput.disabled = !enabled;
  document.querySelectorAll(".target-fader").forEach((fader) => {
    const disabled = !enabled || (cartesianJogEnabled() && !cartesianJogCanRun()) || (fader.dataset.faderKey === "phi" && ikAutoPhiEnabled());
    fader.classList.toggle("disabled", disabled);
    fader.setAttribute("aria-disabled", disabled ? "true" : "false");
  });
  if (elements.previewTaskBtn) elements.previewTaskBtn.disabled = !state.config;
  if (elements.executeTaskBtn) elements.executeTaskBtn.disabled = !state.taskPreviewId || !enabled;
}

function renderState(robotState) {
  state.robotState = robotState;
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
  const hardwareSuffix = robotState.simulation ? "" : ` - ${robotState.hardware_mode}/${robotState.config_sync_status}`;
  elements.statusPill.textContent = robotState.last_error || `${robotState.motion_state}${robotState.live_motion_enabled ? " - live real" : ""}${hardwareSuffix}`;
  renderMotionExecution(robotState);
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
  state.viewPreviewSource = preview.mode === "program" ? "program" : "ik";
  const ik = preview.ik || {};
  const trajectory = preview.trajectory || {};
  const candidates = ik.candidates || [];
  elements.previewStatus.textContent = `Preview ready: ${preview.mode}`;
  elements.programStatus.textContent = preview.mode === "program" ? `Ready - ${trajectory.waypoint_count || 0} pts` : elements.programStatus.textContent;
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
  elements.ikPathSummary.innerHTML = `
    <h3>Path</h3>
    <div class="log-line"><span>Mode</span><code>${trajectory.mode || preview.mode || "-"}</code></div>
    <div class="log-line"><span>Profile</span><code>${trajectory.profile || "-"}</code></div>
    <div class="log-line"><span>Path type</span><code>${pathLayerDescription(preview, trajectory)}</code></div>
    <div class="log-line"><span>Duration</span><code>${format(trajectory.duration_s, 2)} s</code></div>
    <div class="log-line"><span>Waypoints</span><code>${trajectory.waypoint_count || 0}</code></div>
    <div class="log-line"><span>Branch</span><code>${ik.selected_branch || "-"}</code></div>
    <div class="log-line"><span>Target phi</span><code>${preview.target?.phi_auto ? `auto -> ${format(preview.target.phi_deg, 2)} deg` : `${format(preview.target?.phi_deg, 2)} deg`}</code></div>
    <div class="log-line"><span>Execute</span><code id="motionProgressLine">idle - 0%</code></div>
    <div class="log-line"><span>Segments</span><code>${segmentText}</code></div>
  `;

  if (state.previewAngles) state.view.setPreviewAngles(state.previewAngles);

  const target = preview.target?.x_mm !== undefined ? preview.target : trajectory.cartesian_waypoints?.[trajectory.cartesian_waypoints.length - 1];
  state.view.setTargetPoint(target || null);
  state.view.setPathWaypoints(trajectory.waypoints || []);
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
  }
  if (payload.state) renderState(payload.state);
  else syncJointControls();
  updateDisabledState();
}

function renderProgramList() {
  elements.programList.innerHTML = "";
  elements.programStatus.textContent = `${state.programWaypoints.length} waypoint${state.programWaypoints.length === 1 ? "" : "s"}`;
  if (!state.programWaypoints.length) {
    const empty = document.createElement("div");
    empty.className = "program-item";
    empty.textContent = "No waypoints yet.";
    elements.programList.appendChild(empty);
    return;
  }
  state.programWaypoints.forEach((waypoint, index) => {
    const item = document.createElement("div");
    item.className = "program-item";
    const label =
      waypoint.type === "joint"
        ? waypoint.angles_deg.map((value) => format(value, 1)).join(", ")
        : formatCartesianTarget(waypoint.target);
    item.innerHTML = `
      <div class="program-title">
        <span>${index + 1}. ${waypoint.type} / ${waypoint.mode}</span>
        <span>${waypoint.type === "joint" ? "deg" : "mm"}</span>
      </div>
      <code>${label}</code>
      <div class="button-row">
        <button type="button" class="ghost" data-program-action="up" data-program-index="${index}">Up</button>
        <button type="button" class="ghost" data-program-action="down" data-program-index="${index}">Down</button>
        <button type="button" class="danger" data-program-action="delete" data-program-index="${index}">Delete</button>
      </div>
    `;
    elements.programList.appendChild(item);
  });
}

function addCurrentJointWaypoint() {
  const angles = jointControlAngles() || [];
  if (angles.length !== state.config.joints.length) return;
  state.programWaypoints.push({ type: "joint", mode: "joint", angles_deg: angles });
  renderProgramList();
}

function addIkWaypoint() {
  const type = elements.programWaypointType.value;
  const mode = elements.programMoveMode.value;
  if (type === "joint") {
    const selected = state.latestPreview?.ik?.selected?.angles_deg;
    const fallback = jointControlAngles();
    const angles = normalizeJointAngles(selected) || fallback;
    if (!angles) return;
    state.programWaypoints.push({ type: "joint", mode: "joint", angles_deg: angles });
  } else {
    state.programWaypoints.push({
      type: "cartesian",
      mode,
      target: ikTargetPayload(),
      branch: elements.ikBranchSelect.value,
    });
  }
  renderProgramList();
}

async function previewProgram() {
  if (!state.programWaypoints.length) {
    elements.programStatus.textContent = "Add waypoints first";
    return;
  }
  elements.programStatus.textContent = "Previewing...";
  const payload = await postJson("/api/path/preview", {
    mode: "program",
    branch: elements.ikBranchSelect.value,
    settings: pathSettings(),
    waypoints: state.programWaypoints,
  });
  if (payload.ok) renderPreview(payload.preview);
  else {
    renderPreviewFailure(payload);
    elements.programStatus.textContent = payload.error || "Program preview failed";
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
  state.config = config;
  state.linkDraft = null;
  state.dhDraftRows = null;
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
  renderProgramList();
  state.view.setConfig(state.config);
  syncJointControls();
  renderCameraIntrinsics(state.config.camera);
  updateSettingsSaveBar();
}

function clearViewPreview() {
  state.previewId = null;
  state.latestPreview = null;
  state.previewAngles = null;
  state.taskPreviewId = null;
  state.viewPreviewSource = null;
  if (state.view) state.view.clearPreview();
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
        detectVision();
      }
      if (target === "settings") {
        loadAprilTagStatus();
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
        ? { title: "All settings saved", detail: "Saved locally and synced to the controller." }
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
    input?.addEventListener("input", () => markSettingsDirty("motion", "Motion defaults changed. Save all settings to persist them."));
    input?.addEventListener("change", () => markSettingsDirty("motion", "Motion defaults changed. Save all settings to persist them."));
  });
  elements.perJointTuning?.addEventListener("input", () => markSettingsDirty("motion", "Per-joint motion limits changed. Save all settings to persist them."));
  elements.perJointTuning?.addEventListener("change", () => markSettingsDirty("motion", "Per-joint motion limits changed. Save all settings to persist them."));
  [
    elements.cameraSourceInput,
    elements.cameraWidthInput,
    elements.cameraHeightInput,
    elements.cameraFxInput,
    elements.cameraFyInput,
    elements.cameraCxInput,
    elements.cameraCyInput,
    elements.cameraDistortionInput,
  ].forEach((input) => {
    input?.addEventListener("input", () => markSettingsDirty("camera", "Camera model changed. Save all settings or use Save camera settings."));
    input?.addEventListener("change", () => markSettingsDirty("camera", "Camera model changed. Save all settings or use Save camera settings."));
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
  elements.previewTaskBtn.addEventListener("click", previewTask);
  elements.executeTaskBtn.addEventListener("click", executeTask);
  elements.detectVisionBtn.addEventListener("click", detectVision);
  elements.saveCameraIntrinsicsBtn?.addEventListener("click", saveCameraIntrinsics);
  elements.resetAprilTagBtn?.addEventListener("click", resetAprilTagCalibration);
  elements.captureAprilTagBtn?.addEventListener("click", () => captureAprilTags(1, true));
  elements.collectAprilTagBtn?.addEventListener("click", () => captureAprilTags(12, true));
  elements.saveAprilTagBtn?.addEventListener("click", saveAprilTagCalibration);
  elements.verifyAprilTagBtn?.addEventListener("click", verifyAprilTagCalibration);

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

  elements.addCurrentJointWaypointBtn.addEventListener("click", addCurrentJointWaypoint);
  elements.addIkWaypointBtn.addEventListener("click", addIkWaypoint);
  elements.clearProgramBtn.addEventListener("click", () => {
    state.programWaypoints = [];
    renderProgramList();
  });
  elements.previewProgramBtn.addEventListener("click", previewProgram);
  elements.executeProgramBtn.addEventListener("click", executePreview);
  elements.programList.addEventListener("click", (event) => {
    const button = event.target.closest("[data-program-action]");
    if (!button) return;
    const index = Number(button.dataset.programIndex);
    const action = button.dataset.programAction;
    if (action === "delete") state.programWaypoints.splice(index, 1);
    if (action === "up" && index > 0) {
      [state.programWaypoints[index - 1], state.programWaypoints[index]] = [state.programWaypoints[index], state.programWaypoints[index - 1]];
    }
    if (action === "down" && index < state.programWaypoints.length - 1) {
      [state.programWaypoints[index + 1], state.programWaypoints[index]] = [state.programWaypoints[index], state.programWaypoints[index + 1]];
    }
    renderProgramList();
  });

  elements.saveCalibrationBtn.addEventListener("click", saveAllSettings);
  elements.discardSettingsBtn?.addEventListener("click", discardSettingsChanges);
  bindFaders();
  bindPanelChrome();
}

async function init() {
  state.view = new RobotView($("#robotViewport"));
  await loadConfig();
  bindActions();
  await postJson("/api/live-motion", { enabled: false });
  connectWebSocket();
  await loadAprilTagStatus();
}

init();
