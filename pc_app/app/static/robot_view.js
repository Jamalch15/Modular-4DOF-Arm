import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

function armPose(anglesDeg, links, geometryPreset = {}) {
  const rows = Array.isArray(links.dh_rows) && links.dh_rows.length
    ? links.dh_rows
    : [
        { joint_index: 0, theta_offset_deg: 0, d_mm: links.base_height_mm || 0, a_mm: 0, alpha_deg: 90 },
        { joint_index: 1, theta_offset_deg: 90, d_mm: 0, a_mm: links.upper_arm_mm || 0, alpha_deg: 0 },
        { joint_index: 2, theta_offset_deg: 0, d_mm: 0, a_mm: links.forearm_mm || 0, alpha_deg: 0 },
        {
          joint_index: 3,
          theta_offset_deg: 0,
          d_mm: 0,
          a_mm: (links.wrist_mm || 0) + (links.tool_mm || 0),
          alpha_deg: 0,
        },
      ];
  let transform = identity4();
  const frames = [robotPointFromDh(transform)];
  const frameTransforms = [transform];
  const segments = [];
  const dimensions = geometryPreset.dimensions_mm || {};
  const baseSideOffsetMm = Number(
    links.base_side_offset_mm ?? links.base_side_offset ?? dimensions.L_2 ?? 0
  );
  rows.forEach((row, fallbackIndex) => {
    const usesMeasuredBaseSupport = fallbackIndex === 0 && hasMeasuredBaseSupport(dimensions);
    const normalizedIndex = dhJointIndex(row, fallbackIndex);
    const theta =
      Number(anglesDeg[normalizedIndex] || 0) * Number(row.direction_sign ?? 1) +
      Number(row.zero_offset_deg || 0) +
      Number(row.theta_offset_deg || 0);
    const dMm = Number(row.d_mm || 0);
    const aMm = Number(row.a_mm || 0);
    const sideMm = fallbackIndex === 0 ? baseSideOffsetMm : 0;
    const afterTheta = multiply4(transform, rotationZ(theta));
    const afterD = multiply4(afterTheta, translation4(0, 0, dMm));
    const afterSide = multiply4(afterD, translation4(0, sideMm, 0));
    const afterA = multiply4(afterSide, translation4(aMm, 0, 0));
    if (usesMeasuredBaseSupport) {
      addMeasuredBaseSupportSegments(segments, fallbackIndex, transform, afterTheta, dimensions);
    } else {
      addDhSegment(segments, fallbackIndex, "d", transform, afterD, dMm);
      addDhSegment(segments, fallbackIndex, "side", afterD, afterSide, sideMm);
    }
    if (!usesMeasuredBaseSupport) {
      addDhSegment(segments, fallbackIndex, "a", afterD, afterA, aMm);
    }
    transform = multiply4(afterA, rotationX(Number(row.alpha_deg || 0)));
    frames.push(robotPointFromDh(transform));
    frameTransforms.push(transform);
  });
  const tcp = toolTcpPoint(transform, links);
  const lastFrame = frames[frames.length - 1];
  return { frames, frameTransforms, segments, tcp, hasTcpOffset: !samePoint(tcp, lastFrame) };
}

function dhJointIndex(row, fallbackIndex) {
  if (row.joint_index !== undefined && row.joint_index !== null) {
    return Number(row.joint_index);
  }
  if (row.joint !== undefined && row.joint !== null) {
    return Number(row.joint) - 1;
  }
  return fallbackIndex;
}

function identity4() {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

function multiply4(a, b) {
  return a.map((row, rowIndex) =>
    row.map((_, columnIndex) =>
      [0, 1, 2, 3].reduce((sum, index) => sum + a[rowIndex][index] * b[index][columnIndex], 0)
    )
  );
}

function rotationZ(thetaDeg) {
  const theta = (thetaDeg * Math.PI) / 180;
  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  return [
    [ct, -st, 0, 0],
    [st, ct, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

function rotationX(alphaDeg) {
  const alpha = (alphaDeg * Math.PI) / 180;
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);
  return [
    [1, 0, 0, 0],
    [0, ca, -sa, 0],
    [0, sa, ca, 0],
    [0, 0, 0, 1],
  ];
}

function translation4(xMm, yMm, zMm) {
  return [
    [1, 0, 0, xMm],
    [0, 1, 0, yMm],
    [0, 0, 1, zMm],
    [0, 0, 0, 1],
  ];
}

function robotPointFromDh(transform) {
  return { x: transform[1][3], y: -transform[0][3], z: transform[2][3] };
}

function robotPointFromDhVector(vector) {
  return { x: vector[1], y: -vector[0], z: vector[2] };
}

function dhSegmentLabel(rowIndex, kind) {
  const labels = [
    { d: "L1+L3", side: "L2", a: "a1" },
    { d: "s4*L4", a: "L5" },
    { d: "s6*L6", a: "L7" },
    { d: "s8*L8", a: "L9" },
  ];
  return labels[rowIndex]?.[kind] || `${kind}${rowIndex + 1}`;
}

function hasMeasuredBaseSupport(dimensions) {
  return ["L_1", "L_2", "L_3"].every((name) => Number(dimensions[name] || 0) > 0);
}

function addMeasuredBaseSupportSegments(segments, rowIndex, startTransform, afterTheta, dimensions) {
  const l1 = Number(dimensions.L_1 || 0);
  const l2 = Number(dimensions.L_2 || 0);
  const l3 = Number(dimensions.L_3 || 0);
  const afterL1 = multiply4(afterTheta, translation4(0, 0, l1));
  const afterL2 = multiply4(afterL1, translation4(0, l2, 0));
  const afterL3 = multiply4(afterL2, translation4(0, 0, l3));
  addDhSegment(segments, rowIndex, "support", startTransform, afterL1, l1, "L1");
  addDhSegment(segments, rowIndex, "bracket", afterL1, afterL2, l2, "L2");
  addDhSegment(segments, rowIndex, "support", afterL2, afterL3, l3, "L3");
}

function addDhSegment(segments, rowIndex, kind, startTransform, endTransform, signedLengthMm, label = null) {
  const start = robotPointFromDh(startTransform);
  const end = robotPointFromDh(endTransform);
  if (Math.abs(Number(signedLengthMm || 0)) <= 0.0001 || samePoint(start, end)) return;
  segments.push({
    kind,
    rowIndex,
    label: label || dhSegmentLabel(rowIndex, kind),
    signedLengthMm,
    lengthMm: Math.abs(Number(signedLengthMm)),
    start,
    end,
  });
}

function samePoint(a, b, tolerance = 0.0001) {
  return (
    Math.abs(a.x - b.x) <= tolerance &&
    Math.abs(a.y - b.y) <= tolerance &&
    Math.abs(a.z - b.z) <= tolerance
  );
}

function toolTcpPoint(transform, links) {
  const offset = links.tool_tcp_offset_mm || {};
  const toolX = Number(offset.x ?? offset.x_mm ?? 0);
  const toolY = Number(offset.y ?? offset.y_mm ?? 0);
  const toolZ = Number(offset.z ?? offset.z_mm ?? 0);
  // Config uses tool +Z as the forward TCP axis. In this DH model the visible
  // link/tool extension is local DH +X, matching the backend FK mapping.
  const local = [
    toolZ,
    toolX,
    toolY,
    1,
  ];
  const vector = [0, 1, 2, 3].map((rowIndex) =>
    [0, 1, 2, 3].reduce((sum, columnIndex) => sum + transform[rowIndex][columnIndex] * local[columnIndex], 0)
  );
  return robotPointFromDhVector(vector);
}

function makeCylinderBetween(start, end, radius, material) {
  const startVec = start.clone();
  const endVec = end.clone();
  const direction = new THREE.Vector3().subVectors(endVec, startVec);
  const length = direction.length();
  const geometry = new THREE.CylinderGeometry(radius, radius, Math.max(length, 1), 16);
  const mesh = new THREE.Mesh(geometry, material);
  const midpoint = new THREE.Vector3().addVectors(startVec, endVec).multiplyScalar(0.5);
  mesh.position.copy(midpoint);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.normalize());
  return mesh;
}

function robotToScene(point) {
  return new THREE.Vector3(point.x, point.z, -point.y);
}

function sceneDirectionFromDh(transform, direction) {
  const dhVector = [0, 1, 2].map((rowIndex) =>
    [0, 1, 2].reduce((sum, columnIndex) => sum + transform[rowIndex][columnIndex] * direction[columnIndex], 0)
  );
  const robotVector = robotPointFromDhVector(dhVector);
  return new THREE.Vector3(robotVector.x, robotVector.z, -robotVector.y).normalize();
}

function makeTextSprite(text, color = "#dce4ee") {
  const canvas = document.createElement("canvas");
  const context = canvas.getContext("2d");
  const fontSize = 30;
  context.font = `700 ${fontSize}px Segoe UI, Arial, sans-serif`;
  const metrics = context.measureText(text);
  canvas.width = Math.ceil(metrics.width + 22);
  canvas.height = 44;

  context.font = `700 ${fontSize}px Segoe UI, Arial, sans-serif`;
  context.fillStyle = "rgba(10, 14, 22, 0.72)";
  context.fillRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = color;
  context.textBaseline = "middle";
  context.fillText(text, 11, canvas.height / 2);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  const material = new THREE.SpriteMaterial({
    map: texture,
    transparent: true,
    depthWrite: false,
  });
  material.userData.disposeWithObject = true;
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(canvas.width * 0.23, canvas.height * 0.23, 1);
  return sprite;
}

function segmentMidpoint(segment) {
  const start = segment.start;
  const end = segment.end;
  return robotToScene({
    x: (start.x + end.x) / 2,
    y: (start.y + end.y) / 2,
    z: (start.z + end.z) / 2,
  });
}

function segmentMaterial(segment, materials) {
  if (segment.kind === "support") {
    return materials.support || materials.base || materials.dhOffset;
  }
  if (segment.kind === "bracket") {
    return materials.dhOffset || materials.linkAlt;
  }
  if (segment.kind === "side") {
    return materials.dhOffset || materials.linkAlt;
  }
  if (segment.kind === "d") {
    return segment.rowIndex === 0 ? materials.base || materials.dhOffset : materials.dhOffset || materials.linkAlt;
  }
  return segment.rowIndex === 2 ? materials.linkAlt : materials.link;
}

function segmentRadius(segment, radiusScale) {
  if (segment.kind === "support") {
    return (segment.label === "L1" ? 16 : 10) * radiusScale;
  }
  if (segment.kind === "bracket") {
    return 5 * radiusScale;
  }
  if (segment.kind === "side") {
    return 5 * radiusScale;
  }
  if (segment.kind === "d") {
    return (segment.rowIndex === 0 ? 16 : 5) * radiusScale;
  }
  return (segment.rowIndex >= 3 ? 7 : 12) * radiusScale;
}

function fallbackFrameSegments(frames) {
  return frames.slice(0, -1).map((start, index) => ({
    kind: "a",
    rowIndex: index,
    label: `J${index + 1}`,
    start,
    end: frames[index + 1],
  }));
}

function makeFrameAxes(transform, frameIndex, radiusScale) {
  const group = new THREE.Group();
  const origin = robotToScene(robotPointFromDh(transform));
  const length = (frameIndex === 0 ? 48 : 34) * radiusScale;
  [
    { direction: [1, 0, 0], color: 0xff6374 },
    { direction: [0, 1, 0], color: 0x53d18e },
    { direction: [0, 0, 1], color: 0x6aa7ff },
  ].forEach((axis) => {
    const helper = new THREE.ArrowHelper(
      sceneDirectionFromDh(transform, axis.direction),
      origin,
      length,
      axis.color,
      8 * radiusScale,
      4 * radiusScale
    );
    helper.line.material.userData.disposeWithObject = true;
    helper.cone.material.userData.disposeWithObject = true;
    group.add(helper);
  });
  const label = makeTextSprite(`F${frameIndex}`, "#dce4ee");
  label.position.copy(origin).add(new THREE.Vector3(0, length * 0.75, 0));
  label.scale.multiplyScalar(0.72);
  group.add(label);
  return group;
}

function makeJointHub(transform, frameIndex, radiusScale, material) {
  const origin = robotToScene(robotPointFromDh(transform));
  const axis = sceneDirectionFromDh(transform, [0, 0, 1]);
  const length = (frameIndex === 1 ? 34 : 30) * radiusScale;
  const radius = (frameIndex === 1 ? 17 : 14) * radiusScale;
  return makeCylinderBetween(
    origin.clone().addScaledVector(axis, -length / 2),
    origin.clone().addScaledVector(axis, length / 2),
    radius,
    material
  );
}

function makeArmObjects(pose, materials, radiusScale = 1, options = {}) {
  const group = new THREE.Group();
  const points = pose.frames.map(robotToScene);
  const segments = pose.segments?.length ? pose.segments : fallbackFrameSegments(pose.frames);

  segments.forEach((segment) => {
    group.add(
      makeCylinderBetween(
        robotToScene(segment.start),
        robotToScene(segment.end),
        segmentRadius(segment, radiusScale),
        segmentMaterial(segment, materials)
      )
    );
  });

  points.forEach((point, index) => {
    const isBaseFrame = index === 0;
    const isToolMount = index === points.length - 1 && points.length > 4;
    const transform = pose.frameTransforms?.[index];
    if (isBaseFrame) return;
    if (!isToolMount && transform) {
      group.add(makeJointHub(transform, index, radiusScale, materials.joint));
      return;
    }
    const sphere = new THREE.Mesh(new THREE.SphereGeometry(7 * radiusScale, 18, 12), materials.joint);
    sphere.position.set(point.x, point.y, point.z);
    group.add(sphere);
  });

  if (pose.hasTcpOffset) {
    const wristPoint = points[points.length - 1];
    const tcpPoint = robotToScene(pose.tcp);
    group.add(makeCylinderBetween(wristPoint, tcpPoint, 2.5 * radiusScale, materials.tool));

    const tcpMarker = new THREE.Mesh(
      new THREE.SphereGeometry(5.5 * radiusScale, 18, 12),
      materials.tool
    );
    tcpMarker.position.copy(tcpPoint);
    group.add(tcpMarker);
  }

  if (options.showDhHelpers) {
    segments.forEach((segment) => {
      const label = makeTextSprite(segment.label, segment.kind === "a" ? "#5ee6c5" : "#f2b45b");
      label.position.copy(segmentMidpoint(segment));
      label.position.y += (segment.kind === "d" ? 14 : 19) * radiusScale;
      if (segment.kind === "support" || segment.kind === "bracket") {
        label.position.x += 16 * radiusScale;
        label.position.z += 8 * radiusScale;
        label.scale.multiplyScalar(1.22);
      }
      group.add(label);
    });
    (pose.frameTransforms || []).forEach((transform, frameIndex) => {
      group.add(makeFrameAxes(transform, frameIndex, radiusScale));
    });
  }

  return group;
}

function disposeMaterial(material) {
  if (Array.isArray(material)) {
    material.forEach(disposeMaterial);
    return;
  }
  if (!material) return;
  if (!material.userData?.disposeWithObject) return;
  if (material.map) material.map.dispose();
  material.dispose();
}

function disposeObject(object) {
  object.traverse((child) => {
    if (child.geometry) child.geometry.dispose();
    if (child.material) disposeMaterial(child.material);
  });
}

function clearGroup(group) {
  while (group.children.length) {
    const child = group.children[0];
    group.remove(child);
    disposeObject(child);
  }
}

function removeChildrenByKind(group, kind) {
  group.children
    .filter((child) => child.userData.kind === kind)
    .forEach((child) => {
      group.remove(child);
      disposeObject(child);
    });
}

function sameAngles(a, b) {
  return (
    Array.isArray(a) &&
    Array.isArray(b) &&
    a.length === b.length &&
    a.every((value, index) => Math.abs(Number(value) - Number(b[index])) < 0.0005)
  );
}

function activeGeometryPreset(config) {
  const geometry = config?.geometry || {};
  const presets = geometry.presets || {};
  const activeName = geometry.active_preset || Object.keys(presets)[0];
  return activeName && presets[activeName] ? presets[activeName] : {};
}

export class RobotView {
  constructor(container) {
    this.container = container;
    this.config = {};
    this.links = {};
    this.geometryPreset = {};
    this.angles = [0, 0, 0, 0];
    this.previewVisible = true;
    this.pathVisible = true;
    this.framesVisible = true;
    this.previewAngles = null;
    this.lastRenderedAngles = null;
    this.lastConfigSignature = "";
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x171f2d);

    this.camera = new THREE.PerspectiveCamera(45, 1, 1, 2000);

    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: false,
      powerPreference: "high-performance",
      preserveDrawingBuffer: true,
    });
    this.renderer.setClearColor(0x171f2d, 1);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(this.renderer.domElement);
    this.renderer.domElement.addEventListener("webglcontextlost", (event) => {
      event.preventDefault();
      this.container.dataset.webglStatus = "lost";
    });
    this.renderer.domElement.addEventListener("webglcontextrestored", () => {
      this.container.dataset.webglStatus = "ready";
      this.renderRobot();
      this.render();
    });
    this.container.dataset.webglStatus = "ready";

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.screenSpacePanning = true;
    this.controls.minDistance = 120;
    this.controls.maxDistance = 1200;
    this.controls.addEventListener("change", () => this.render());

    this.armGroup = new THREE.Group();
    this.previewGroup = new THREE.Group();
    this.overlayGroup = new THREE.Group();
    this.objectGroup = new THREE.Group();
    this.scene.add(this.armGroup);
    this.scene.add(this.previewGroup);
    this.scene.add(this.overlayGroup);
    this.scene.add(this.objectGroup);

    const ambient = new THREE.AmbientLight(0xffffff, 0.8);
    const key = new THREE.DirectionalLight(0xffffff, 1.3);
    key.position.set(260, 520, 360);
    this.scene.add(ambient, key);

    this.grid = new THREE.GridHelper(720, 18, 0x2d3748, 0x202938);
    this.scene.add(this.grid);

    this.axes = new THREE.AxesHelper(180);
    this.scene.add(this.axes);

    this.materials = {
      base: new THREE.MeshStandardMaterial({ color: 0x0d1318, roughness: 0.55 }),
      support: new THREE.MeshStandardMaterial({ color: 0x334257, roughness: 0.55 }),
      link: new THREE.MeshStandardMaterial({ color: 0x0f6f69, roughness: 0.48 }),
      linkAlt: new THREE.MeshStandardMaterial({ color: 0xb98225, roughness: 0.5 }),
      dhOffset: new THREE.MeshStandardMaterial({ color: 0xf2b45b, roughness: 0.52 }),
      joint: new THREE.MeshStandardMaterial({ color: 0xdce4ee, roughness: 0.4 }),
      tool: new THREE.MeshStandardMaterial({ color: 0xff6374, roughness: 0.42 }),
    };
    this.previewMaterials = {
      base: new THREE.MeshStandardMaterial({
        color: 0x627086,
        roughness: 0.5,
        transparent: true,
        opacity: 0.32,
        depthWrite: false,
      }),
      support: new THREE.MeshStandardMaterial({
        color: 0x8fa4bf,
        roughness: 0.5,
        transparent: true,
        opacity: 0.36,
        depthWrite: false,
      }),
      link: new THREE.MeshStandardMaterial({
        color: 0x6f96d1,
        roughness: 0.48,
        transparent: true,
        opacity: 0.34,
        depthWrite: false,
      }),
      linkAlt: new THREE.MeshStandardMaterial({
        color: 0xa58bd8,
        roughness: 0.48,
        transparent: true,
        opacity: 0.34,
        depthWrite: false,
      }),
      dhOffset: new THREE.MeshStandardMaterial({
        color: 0xf2b45b,
        roughness: 0.48,
        transparent: true,
        opacity: 0.38,
        depthWrite: false,
      }),
      joint: new THREE.MeshStandardMaterial({
        color: 0xffffff,
        roughness: 0.4,
        transparent: true,
        opacity: 0.45,
        depthWrite: false,
      }),
      tool: new THREE.MeshStandardMaterial({
        color: 0x2f6bd1,
        roughness: 0.42,
        transparent: true,
        opacity: 0.55,
        depthWrite: false,
      }),
    };
    this.pathMaterial = new THREE.LineBasicMaterial({ color: 0x6aa7ff, linewidth: 2 });
    this.actualPathMaterial = new THREE.LineDashedMaterial({
      color: 0x53d18e,
      linewidth: 2,
      dashSize: 14,
      gapSize: 8,
    });
    this.targetMaterial = new THREE.MeshStandardMaterial({
      color: 0xffd24a,
      emissive: 0x332300,
      roughness: 0.35,
    });
    this.objectMaterials = {
      red: new THREE.MeshStandardMaterial({ color: 0xff526d, emissive: 0x28040c, roughness: 0.4 }),
      green: new THREE.MeshStandardMaterial({ color: 0x53d18e, emissive: 0x052211, roughness: 0.4 }),
      blue: new THREE.MeshStandardMaterial({ color: 0x6aa7ff, emissive: 0x061629, roughness: 0.4 }),
      yellow: new THREE.MeshStandardMaterial({ color: 0xffd24a, emissive: 0x2d2203, roughness: 0.4 }),
      default: new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0x121212, roughness: 0.4 }),
    };

    this.resetCamera();
    window.addEventListener("resize", () => this.resize());
    this.resize();
    this.animate();
  }

  setConfig(config) {
    this.config = config || {};
    this.links = this.config.links_mm || {};
    this.geometryPreset = activeGeometryPreset(this.config);
    this.lastConfigSignature = JSON.stringify(this.links);
    this.lastRenderedAngles = null;
    this.previewAngles = null;
    this.renderRobot();
  }

  setAngles(angles) {
    const normalizedAngles = angles.map(Number);
    if (sameAngles(normalizedAngles, this.lastRenderedAngles)) return;
    this.angles = normalizedAngles;
    this.container.dataset.currentAngles = this.angles.map((angle) => angle.toFixed(3)).join(",");
    this.renderRobot();
  }

  setPreviewAngles(angles) {
    if (!angles || angles.length !== 4) {
      clearGroup(this.previewGroup);
      this.previewAngles = null;
      delete this.container.dataset.previewAngles;
      this.render();
      return;
    }
    const normalizedAngles = angles.map(Number);
    if (sameAngles(normalizedAngles, this.previewAngles)) return;
    clearGroup(this.previewGroup);
    this.previewAngles = normalizedAngles;
    this.container.dataset.previewAngles = normalizedAngles.map((angle) => angle.toFixed(3)).join(",");
    const pose = armPose(normalizedAngles, this.links, this.geometryPreset);
    this.previewGroup.add(makeArmObjects(pose, this.previewMaterials, 0.82));
    this.previewGroup.visible = this.previewVisible;
    this.render();
  }

  setTargetPoint(point) {
    removeChildrenByKind(this.overlayGroup, "target");
    if (!point) {
      delete this.container.dataset.targetPoint;
      this.render();
      return;
    }

    this.container.dataset.targetPoint = [
      Number(point.x_mm || 0).toFixed(3),
      Number(point.y_mm || 0).toFixed(3),
      Number(point.z_mm || 0).toFixed(3),
    ].join(",");
    const marker = new THREE.Mesh(new THREE.SphereGeometry(10, 24, 16), this.targetMaterial);
    marker.position.copy(
      robotToScene({
        x: Number(point.x_mm || 0),
        y: Number(point.y_mm || 0),
        z: Number(point.z_mm || 0),
      })
    );
    marker.userData.kind = "target";
    this.overlayGroup.add(marker);
    this.render();
  }

  setPathWaypoints(waypoints) {
    removeChildrenByKind(this.overlayGroup, "path");
    removeChildrenByKind(this.overlayGroup, "plannedPath");
    if (!waypoints || waypoints.length < 2) {
      delete this.container.dataset.pathWaypointCount;
      this.render();
      return;
    }

    this.container.dataset.pathWaypointCount = String(waypoints.length);
    const pathPoints = waypoints.map((angles) => {
      return robotToScene(armPose(angles.map(Number), this.links, this.geometryPreset).tcp);
    });
    const geometry = new THREE.BufferGeometry().setFromPoints(pathPoints);
    const line = new THREE.Line(geometry, this.pathMaterial);
    line.userData.kind = "plannedPath";
    line.visible = this.pathVisible;
    this.overlayGroup.add(line);
    this.render();
  }

  setActualTcpPath(points) {
    removeChildrenByKind(this.overlayGroup, "actualPath");
    if (!points || points.length < 2) {
      delete this.container.dataset.actualPathPointCount;
      this.render();
      return;
    }

    this.container.dataset.actualPathPointCount = String(points.length);
    const pathPoints = points.map((point) => {
      return robotToScene({
        x: Number(point.x_mm ?? point.x ?? 0),
        y: Number(point.y_mm ?? point.y ?? 0),
        z: Number(point.z_mm ?? point.z ?? 0),
      });
    });
    const geometry = new THREE.BufferGeometry().setFromPoints(pathPoints);
    const line = new THREE.Line(geometry, this.actualPathMaterial);
    line.computeLineDistances();
    line.userData.kind = "actualPath";
    line.visible = this.pathVisible;
    this.overlayGroup.add(line);
    this.render();
  }

  setObjectDetections(detections) {
    clearGroup(this.objectGroup);
    if (!Array.isArray(detections) || detections.length === 0) {
      delete this.container.dataset.objectMarkerCount;
      this.render();
      return;
    }

    let count = 0;
    detections.forEach((detection) => {
      const robot = detection.robot || {};
      if (robot.x_mm == null || robot.y_mm == null) return;
      const colorName = String(detection.color || "default").toLowerCase();
      const material = this.objectMaterials[colorName] || this.objectMaterials.default;
      const marker = new THREE.Mesh(new THREE.SphereGeometry(8, 24, 16), material);
      marker.position.copy(
        robotToScene({
          x: Number(robot.x_mm),
          y: Number(robot.y_mm),
          z: Number(robot.z_mm || 0),
        })
      );
      marker.userData.kind = "object";
      this.objectGroup.add(marker);
      count += 1;
    });
    this.container.dataset.objectMarkerCount = String(count);
    this.render();
  }

  setPreviewVisible(visible) {
    this.previewVisible = Boolean(visible);
    this.previewGroup.visible = this.previewVisible;
    this.render();
  }

  setPathVisible(visible) {
    this.pathVisible = Boolean(visible);
    this.overlayGroup.children
      .filter((child) => ["path", "plannedPath", "actualPath"].includes(child.userData.kind))
      .forEach((child) => {
        child.visible = this.pathVisible;
      });
    this.render();
  }

  setFramesVisible(visible) {
    this.framesVisible = Boolean(visible);
    this.grid.visible = this.framesVisible;
    this.axes.visible = this.framesVisible;
    this.renderRobot();
    this.render();
  }

  clearPreview() {
    clearGroup(this.previewGroup);
    clearGroup(this.overlayGroup);
    this.previewAngles = null;
    delete this.container.dataset.previewAngles;
    delete this.container.dataset.targetPoint;
    delete this.container.dataset.pathWaypointCount;
    delete this.container.dataset.actualPathPointCount;
    this.render();
  }

  resize() {
    const width = Math.max(this.container.clientWidth, 1);
    const height = Math.max(this.container.clientHeight, 1);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
    this.render();
  }

  resetCamera() {
    const target = new THREE.Vector3(80, 120, -140);
    this.camera.position.set(620, 520, 660);
    this.camera.lookAt(target);
    if (this.controls) {
      this.controls.target.copy(target);
      this.controls.update();
    }
    this.render();
  }

  renderRobot() {
    clearGroup(this.armGroup);
    this.lastRenderedAngles = this.angles.slice();
    const pose = armPose(this.angles, this.links, this.geometryPreset);

    const base = new THREE.Mesh(
      new THREE.CylinderGeometry(54, 66, 28, 32),
      this.materials.base
    );
    base.position.set(0, 14, 0);
    this.armGroup.add(base);

    this.armGroup.add(makeArmObjects(pose, this.materials, 1, { showDhHelpers: this.framesVisible }));

    this.render();
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    this.controls.update();
    this.render();
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }
}
