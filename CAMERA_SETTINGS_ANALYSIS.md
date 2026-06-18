# Camera Settings Display Code Analysis

## Summary
Camera settings are **fully implemented** in the codebase and should be displaying correctly. The system follows this flow:
1. **Backend** sends camera settings via `camera_settings()` function in `app/demo_settings.py`
2. **API** exposes settings through `/api/config` and `/api/vision/settings` endpoints
3. **Frontend** receives and displays settings via `renderCameraIntrinsics()` in JavaScript
4. **HTML** provides a complete settings tab with camera intrinsics and AprilTag calibration controls

---

## 1. HTML Camera Settings Display Code

**File**: [pc_app/app/static/index.html](pc_app/app/static/index.html#L355-L400)

The camera settings are displayed in a dedicated settings section:

```html
<section class="panel-section settings-section" id="settingsCamera">
  <div class="section-header">
    <div>
      <h2>Camera and workspace calibration</h2>
      <p class="hint">Logitech C270 at 1280×720. AprilTag 36h11 IDs 0–3 define workspace Z=0.</p>
    </div>
    <span id="aprilTagStatus" class="status-chip">Uncalibrated</span>
  </div>

  <!-- Camera model (intrinsics) section -->
  <div class="subsection-heading">
    <h3>Camera model</h3>
    <p class="hint">Image size and pinhole intrinsics. The current C270 values are an estimated starting point.</p>
  </div>

  <div class="control-grid camera-intrinsics-grid">
    <label>Camera index <input id="cameraSourceInput" type="number" min="0" step="1" /></label>
    <label>Image width <input id="cameraWidthInput" type="number" min="1" step="1" /></label>
    <label>Image height <input id="cameraHeightInput" type="number" min="1" step="1" /></label>
    <label>Horizontal focal length (fx) <input id="cameraFxInput" type="number" min="0.01" step="0.01" /></label>
    <label>Vertical focal length (fy) <input id="cameraFyInput" type="number" min="0.01" step="0.01" /></label>
    <label>Principal point X (cx) <input id="cameraCxInput" type="number" step="0.01" /></label>
    <label>Principal point Y (cy) <input id="cameraCyInput" type="number" step="0.01" /></label>
    <label>Lens distortion (k1, k2, p1, p2, k3)
      <input id="cameraDistortionInput" type="text" />
    </label>
  </div>

  <div class="button-row">
    <button id="saveCameraIntrinsicsBtn" type="button">Save camera settings</button>
  </div>

  <!-- AprilTag calibration section -->
  <div class="subsection-heading">
    <h3>AprilTag pose calibration</h3>
    <p class="hint">Collect stable frames, save the solved camera pose, then verify it against a fresh image.</p>
  </div>
  
  <div class="button-row calibration-actions">
    <button id="resetAprilTagBtn" type="button" class="ghost">Reset Samples</button>
    <button id="captureAprilTagBtn" type="button">Capture</button>
    <button id="collectAprilTagBtn" type="button" class="primary">Collect 12</button>
    <button id="saveAprilTagBtn" type="button">Save Pose</button>
    <button id="verifyAprilTagBtn" type="button">Verify</button>
  </div>
  
  <div class="camera-frame april-tag-frame">
    <img id="aprilTagFrame" alt="AprilTag calibration frame" />
    <div id="aprilTagPlaceholder">No AprilTag frame yet</div>
  </div>
  
  <div id="aprilTagMetrics" class="path-summary"></div>
  <div id="aprilTagDetections" class="program-list compact-list"></div>
</section>
```

---

## 2. Frontend JavaScript - Camera Settings Rendering

**File**: [pc_app/app/static/app.js](pc_app/app/static/app.js#L1387-L1460)

### Element References
Lines 180-230 register all camera settings input elements:

```javascript
cameraSourceInput: $("#cameraSourceInput"),
cameraWidthInput: $("#cameraWidthInput"),
cameraHeightInput: $("#cameraHeightInput"),
cameraFxInput: $("#cameraFxInput"),
cameraFyInput: $("#cameraFyInput"),
cameraCxInput: $("#cameraCxInput"),
cameraCyInput: $("#cameraCyInput"),
cameraDistortionInput: $("#cameraDistortionInput"),
saveCameraIntrinsicsBtn: $("#saveCameraIntrinsicsBtn"),
aprilTagStatus: $("#aprilTagStatus"),
aprilTagFrame: $("#aprilTagFrame"),
aprilTagPlaceholder: $("#aprilTagPlaceholder"),
aprilTagMetrics: $("#aprilTagMetrics"),
aprilTagDetections: $("#aprilTagDetections"),
```

### Render Camera Intrinsics Function
Lines 1387-1400:

```javascript
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
```

### Apply Config Function (calls renderCameraIntrinsics)
Lines 2600-2620:

```javascript
function applyConfig(config) {
  state.config = config;
  // ... other initialization ...
  
  renderCameraIntrinsics(state.config.camera);  // LINE 2619
  
  updateSettingsSaveBar();
}
```

### Save Camera Intrinsics Function
Lines 1443-1459:

```javascript
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
```

### Event Listeners for Camera Settings
Lines 2991-3005:

```javascript
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
  input?.addEventListener("input", () =>
    markSettingsDirty("camera", "Camera model changed. Save all settings or use Save camera settings.")
  );
  input?.addEventListener("change", () =>
    markSettingsDirty("camera", "Camera model changed. Save all settings or use Save camera settings.")
  );
});
```

---

## 3. Settings Tab Navigation

**File**: [pc_app/app/static/index.html](pc_app/app/static/index.html#L245-L260)

The Settings tab has a navigation menu that includes Camera:

```html
<nav class="settings-section-nav" aria-label="Settings sections">
  <button type="button" data-settings-target="settingsGeometry">Geometry</button>
  <button type="button" data-settings-target="settingsJoints">Joints</button>
  <button type="button" data-settings-target="settingsMotion">Motion</button>
  <button type="button" data-settings-target="settingsTooling">Tooling</button>
  <button type="button" data-settings-target="settingsCamera">Camera</button>
  <button type="button" data-settings-target="settingsHardware">Hardware</button>
</nav>
```

---

## 4. Backend - Camera Settings Functions

**File**: [pc_app/app/demo_settings.py](pc_app/app/demo_settings.py#L68-L135)

```python
def camera_settings(config: RobotConfig) -> dict[str, Any]:
    defaults = {
        "source_index": 0,
        "enabled": False,
        "resolution": {
            "width": 1280,
            "height": 720,
        },
        "intrinsics": {
            "source": "uncalibrated",
            "fx_px": None,
            "fy_px": None,
            "cx_px": None,
            "cy_px": None,
            "distortion_coefficients": [0.0, 0.0, 0.0, 0.0, 0.0],
        },
        "calibration": {
            "image_points": [],
            "robot_points": [],
            "apriltag": {
                "enabled": True,
                "dictionary": "DICT_APRILTAG_36H11",
                "tag_size_mm": 40.0,
                "required_ids": [0, 1, 2, 3],
                "min_tags_for_pose": 2,
                "min_samples": 12,
                "max_samples": 120,
                "max_reprojection_error_px": 2.5,
                "max_tilt_from_down_deg": 45.0,
                "tags": {...},
                "result": None,
            },
        },
    }
    # Merges with raw config if present
    raw = config.raw.get("camera")
    if isinstance(raw, dict):
        merged = deepcopy(defaults)
        # Deep merging logic...
        return merged
    return defaults
```

---

## 5. Backend - API Endpoints

**File**: [pc_app/app/main.py](pc_app/app/main.py)

### Get Config Endpoint (includes camera settings)
Lines 1937+:

```python
@app.get("/api/config")
async def get_config() -> dict[str, Any]:
    return {"ok": True, "config": public_config()}
```

### Public Config Function
Lines 282-309:

```python
def public_config() -> dict[str, Any]:
    return {
        "joints": [asdict(joint) for joint in config.joints],
        "links_mm": asdict(config.links),
        # ... other config ...
        "camera": camera_settings(config),  # <-- CAMERA SETTINGS INCLUDED
        "color_profiles": color_profiles(config),
        "drop_zones": drop_zones(config),
        "task_defaults": task_defaults(config),
        # ... more config ...
    }
```

### Save Vision Settings Endpoint
Lines 2089-2102:

```python
@app.post("/api/vision/settings")
async def save_vision_settings(request: VisionSettingsRequest) -> dict[str, Any]:
    updates = request.__dict__
    try:
        save_calibration_updates(ensure_local_config(), updates)
        reload_runtime_config()
    except Exception as exc:
        state.set_error(f"could not save vision settings: {exc}")
        await broadcast_state()
        return {"ok": False, "error": state.last_error, "state": state.to_dict()}
    log_event("vision", "vision settings saved")
    await broadcast_state()
    return {"ok": True, "config": public_config(), "state": state.to_dict()}
```

### Vision Config Endpoint
Lines 2057-2065:

```python
@app.get("/api/vision/config")
async def get_vision_config() -> dict[str, Any]:
    return {
        "ok": True,
        "camera": camera_settings(config),
        "color_profiles": color_profiles(config),
        "drop_zones": drop_zones(config),
    }
```

---

## 6. Camera Calibration (AprilTag) Code

**File**: [pc_app/app/apriltag_calibration.py](pc_app/app/apriltag_calibration.py#L1-100)

### April Tag Settings
Lines 50-65:

```python
def april_tag_settings(camera: dict[str, Any]) -> dict[str, Any]:
    calibration = camera.get("calibration") if isinstance(camera.get("calibration"), dict) else {}
    raw = calibration.get("apriltag") if isinstance(calibration.get("apriltag"), dict) else {}
    defaults = {
        "enabled": True,
        "dictionary": "DICT_APRILTAG_36H11",
        "tag_size_mm": 40.0,
        "required_ids": [0, 1, 2, 3],
        "min_tags_for_pose": 2,
        "min_samples": 12,
        "max_samples": 120,
        "max_reprojection_error_px": 2.5,
        "max_tilt_from_down_deg": 45.0,
        # ... tag definitions ...
        "result": None,
    }
    defaults.update(raw)
    return defaults
```

---

## 7. Workflow: Settings Tab Navigation

**File**: [pc_app/app/static/app.js](pc_app/app/static/app.js#L3007-3015)

The settings section navigation allows users to scroll to the camera section:

```javascript
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
```

---

## Data Flow Diagram

```
Config File (robot.example.yaml or robot.local.yaml)
         ↓
demo_settings.camera_settings()
         ↓
public_config() includes "camera": camera_settings(config)
         ↓
GET /api/config
         ↓
Frontend receives config via fetch()
         ↓
applyConfig(config) called
         ↓
renderCameraIntrinsics(state.config.camera)
         ↓
Camera input fields populated:
  - #cameraSourceInput
  - #cameraWidthInput
  - #cameraHeightInput
  - #cameraFxInput
  - #cameraFyInput
  - #cameraCxInput
  - #cameraCyInput
  - #cameraDistortionInput
  - #aprilTagStatus
```

---

## Potential Issues Preventing Display

If camera settings **aren't showing**, check these:

### 1. **Settings Tab Not Active**
   - The `#settingsTab` must have `.active` class
   - Check if tab navigation is working: `elements.settingsSectionNav`

### 2. **Camera Settings Section Hidden/Collapsed**
   - Verify `#settingsCamera` has `.active` or correct CSS visibility
   - Settings might be scrolled out of view

### 3. **Element Null Checks**
   - All input references use `?.` null coalescing
   - If any element ID doesn't match HTML, values won't populate silently

### 4. **Config Not Loaded**
   - Verify `/api/config` returns camera data
   - Check browser console for network errors
   - Check `state.config?.camera` in DevTools console

### 5. **CSS Display Issues**
   - `.settings-section` must be visible in Settings tab
   - `.control-grid.camera-intrinsics-grid` must display labels correctly

### 6. **Camera Settings Elements Not Visible in DOM**
   - Right-click on input field → Inspect Element
   - Check if element exists but is hidden/styled with `display: none`
   - Check parent containers for CSS overflow issues

---

## Test Verification Steps

1. **Navigate to Settings tab** → Click "Camera" button in navigation
2. **Expected to see:**
   - "Camera and workspace calibration" heading
   - Camera model subsection with 8 input fields
   - AprilTag pose calibration subsection with 5 buttons
   - Camera frame display area
   
3. **Check browser console:**
   ```javascript
   console.log(state.config?.camera)  // Should show full camera object
   console.log(elements.cameraSourceInput)  // Should be HTMLInputElement, not null
   ```

4. **Verify API response:**
   ```javascript
   fetch('/api/config').then(r => r.json()).then(c => console.log(c.camera))
   ```

---

## Related Files
- [pc_app/app/static/robot_view.js](pc_app/app/static/robot_view.js#L646) - Vision visualization (has `setAprilTagCalibration()`)
- [pc_app/tests/test_apriltag_calibration.py](pc_app/tests/test_apriltag_calibration.py) - Camera settings tests
- [pc_app/tools/calibrate_apriltags.py](pc_app/tools/calibrate_apriltags.py) - Standalone calibration tool
