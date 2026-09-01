/**
 * Advanced Ballistics Simulation & Projectile Motion Engine
 * Numerical Runge-Kutta 4th Order (RK4) integration with aerodynamic drag,
 * wind effects, optics zeroing, line of sight (LOS), Mach/transonic analysis,
 * DOPE drop chart, interactive canvas rendering, and real-time analytical telemetry.
 */

const GRAIN_TO_KG = 0.00006479891; // 1 gr = 0.06479891 grammi
const GRAIN_TO_G = 0.06479891;
const SPEED_OF_SOUND = 343.2; // m/s nell'aria a 20°C al livello del mare

// --- Presets Data ---
const PRESETS = {
  '9mm': {
    name: '9x19mm Parabellum',
    v0: 380,
    angle: 15,
    y0: 1.5,
    massGrains: 124, // 124 gr (~8.04 g)
    diameter: 9.0, // mm
    cd: 0.295,
    bcG1: 0.145,
    wind: 0,
    airDensity: 1.225,
    sightHeight: 3.5,
    zeroDist: 50
  },
  'sniper': {
    name: '.308 Winchester (7.62mm)',
    v0: 850,
    angle: 5,
    y0: 1.5,
    massGrains: 168, // 168 gr Sierra MatchKing (~10.89 g)
    diameter: 7.62, // mm
    cd: 0.220,
    bcG1: 0.462,
    wind: 0,
    airDensity: 1.225,
    sightHeight: 4.5,
    zeroDist: 100
  },
  'cannon': {
    name: 'Palla di Cannone 12-lb',
    v0: 250,
    angle: 35,
    y0: 2.0,
    massGrains: 83333, // ~5.4 kg
    diameter: 115.0, // mm
    cd: 0.47,
    bcG1: 0.850,
    wind: 0,
    airDensity: 1.225,
    sightHeight: 5.0,
    zeroDist: 200
  },
  'arrow': {
    name: 'Freccia da Tiro con l\'Arco',
    v0: 70,
    angle: 30,
    y0: 1.6,
    massGrains: 385, // 385 gr (~25 g)
    diameter: 7.0, // mm
    cd: 1.20,
    bcG1: 0.055,
    wind: 0,
    airDensity: 1.225,
    sightHeight: 2.0,
    zeroDist: 30
  },
  'baseball': {
    name: 'Palla da Baseball (Lancio)',
    v0: 42,
    angle: 38,
    y0: 1.8,
    massGrains: 2237, // ~145 g
    diameter: 74.0, // mm
    cd: 0.30,
    bcG1: 0.120,
    wind: 0,
    airDensity: 1.225,
    sightHeight: 0,
    zeroDist: 30
  },
  'ideal': {
    name: 'Lancio nel Vuoto Ideale (45°)',
    v0: 100,
    angle: 45,
    y0: 0,
    massGrains: 15432, // ~1 kg
    diameter: 20.0,
    cd: 0.0,
    bcG1: 1.0,
    wind: 0,
    airDensity: 1.225,
    sightHeight: 0,
    zeroDist: 100
  }
};

// --- Application State ---
const state = {
  v0: 380, // m/s
  angleDeg: 15, // deg
  y0: 1.5, // m
  massGrains: 124, // grani (gr)
  diameterMm: 9.0, // mm
  cd: 0.295,
  gravity: 9.80665, // m/s^2
  airDensity: 1.225, // kg/m^3
  wind: 0, // m/s

  // Optics & Zeroing
  sightHeightCm: 4.5, // cm
  zeroDistanceM: 100, // m
  dragModel: 'cd', // 'cd', 'g1', 'g7'
  dopeStep: 50, // m

  // Visualization Toggles
  compareVacuum: true,
  showLos: true,
  showMach: true,
  showVectors: true,
  showTarget: true,
  showSweep: false,

  target: {
    x: 450, // m
    y: 0, // m
    radius: 4, // m
    isDragging: false
  },

  // Playback & Animation
  isPlaying: false,
  currentTime: 0,
  playbackSpeed: 1.0,
  historyTrajectories: [], // store previous runs for comparison

  // Viewport / Camera Transformation
  camera: {
    panX: 60, // screen px offset
    panY: 60, // screen px offset from bottom
    scale: 1.0, // dynamic pixels per meter
    isPanning: false,
    startX: 0,
    startY: 0
  },

  // Simulation Results Cache
  trajectory: [], // [{t, x, y, vx, vy, v, mach, ax, ay, fd, ek, yLos, dropCm, mrad, moa}, ...]
  idealTrajectory: [],
  sweepTrajectories: [],
  transonicPoint: null, // Point where Mach = 1.0
  losSlope: 0, // Line of Sight slope
  losIntercept: 0, // Line of Sight y0 + sightHeight
  metrics: {
    range: 0,
    maxHeight: 0,
    apexX: 0,
    apexTime: 0,
    flightTime: 0,
    impactVelocity: 0,
    impactMach: 0,
    impactAngle: 0,
    impactEnergy: 0,
    momentum: 0,
    idealRange: 0,
    idealMaxHeight: 0,
    idealFlightTime: 0,
    transonicDist: null,
    transonicTime: null
  },

  // Visual Particles
  particles: []
};

// --- DOM Elements ---
const dom = {
  // Inputs
  velocity: document.getElementById('input-velocity'),
  angle: document.getElementById('input-angle'),
  height: document.getElementById('input-height'),
  mass: document.getElementById('input-mass'),
  diameter: document.getElementById('input-diameter'),
  cd: document.getElementById('input-cd'),
  gravity: document.getElementById('select-gravity'),
  airDensity: document.getElementById('input-air-density'),
  wind: document.getElementById('input-wind'),

  // Optics Inputs
  sightHeight: document.getElementById('input-sight-height'),
  zeroDistance: document.getElementById('input-zero-distance'),
  dragModel: document.getElementById('select-drag-model'),
  valSightHeight: document.getElementById('val-sight-height'),
  valZeroDistance: document.getElementById('val-zero-distance'),
  valBcDisplay: document.getElementById('val-bc-display'),

  // Badges
  valVelocity: document.getElementById('val-velocity'),
  valVelocityKmh: document.getElementById('val-velocity-kmh'),
  valAngle: document.getElementById('val-angle'),
  valHeight: document.getElementById('val-height'),
  valMass: document.getElementById('val-mass'),
  valMassGrams: document.getElementById('val-mass-grams'),
  valDiameter: document.getElementById('val-diameter'),
  valCd: document.getElementById('val-cd'),
  valAirDensity: document.getElementById('val-air-density'),
  valWind: document.getElementById('val-wind'),

  // Toggles
  toggleVacuum: document.getElementById('toggle-vacuum-compare'),
  toggleLos: document.getElementById('toggle-los'),
  toggleMach: document.getElementById('toggle-mach'),
  toggleVectors: document.getElementById('toggle-vectors'),
  toggleTarget: document.getElementById('toggle-target'),
  toggleSweep: document.getElementById('toggle-sweep'),

  // Presets
  presetBtns: document.querySelectorAll('.preset-btn'),

  // Metrics Display
  resRange: document.getElementById('res-range'),
  resMaxHeight: document.getElementById('res-max-height'),
  resFlightTime: document.getElementById('res-flight-time'),
  resImpactVelocity: document.getElementById('res-impact-velocity'),
  resImpactAngle: document.getElementById('res-impact-angle'),
  resImpactEnergy: document.getElementById('res-impact-energy'),
  resMomentum: document.getElementById('res-momentum'),
  subRangeIdeal: document.getElementById('sub-range-ideal'),
  subHeightIdeal: document.getElementById('sub-height-ideal'),
  subTimeIdeal: document.getElementById('sub-time-ideal'),

  // HUD
  hudTime: document.getElementById('hud-time'),
  hudX: document.getElementById('hud-x'),
  hudY: document.getElementById('hud-y'),
  hudV: document.getElementById('hud-v'),
  hudMach: document.getElementById('hud-mach'),
  hudDrop: document.getElementById('hud-drop'),
  hudFd: document.getElementById('hud-fd'),
  hudEk: document.getElementById('hud-ek'),

  // Target Banner
  targetBanner: document.getElementById('target-banner'),
  targetPosLabel: document.getElementById('target-pos-label'),
  targetStatusLabel: document.getElementById('target-status-label'),

  // Playback Controls
  btnPlayPause: document.getElementById('btn-play-pause'),
  iconPlay: document.getElementById('icon-play'),
  iconPause: document.getElementById('icon-pause'),
  labelPlay: document.getElementById('label-play'),
  btnStep: document.getElementById('btn-step'),
  btnRestart: document.getElementById('btn-restart'),
  timeScrubber: document.getElementById('time-scrubber'),
  scrubCurrentTime: document.getElementById('scrub-current-time'),
  scrubMaxTime: document.getElementById('scrub-max-time'),
  speedPills: document.querySelectorAll('.speed-pill'),

  // Canvas
  canvas: document.getElementById('ballistics-canvas'),
  canvasWrapper: document.getElementById('canvas-wrapper'),
  btnZoomIn: document.getElementById('btn-zoom-in'),
  btnZoomOut: document.getElementById('btn-zoom-out'),
  btnZoomFit: document.getElementById('btn-zoom-fit'),
  btnClearTrails: document.getElementById('btn-clear-trails'),

  // Analytical Charts & DOPE Tab
  tabBtns: document.querySelectorAll('.tab-btn'),
  tabPanes: document.querySelectorAll('.tab-pane'),
  chartVelocity: document.getElementById('chart-velocity'),
  chartHeight: document.getElementById('chart-height'),
  chartEnergy: document.getElementById('chart-energy'),

  // DOPE Table elements
  selectDopeStep: document.getElementById('select-dope-step'),
  dopeZeroLabel: document.getElementById('dope-zero-label'),
  dopeSightLabel: document.getElementById('dope-sight-label'),
  dopeTransonicLabel: document.getElementById('dope-transonic-label'),
  dopeTableBody: document.getElementById('dope-table-body'),

  // Top Buttons
  btnExportCsv: document.getElementById('btn-export-csv'),
  btnResetDefaults: document.getElementById('btn-reset-defaults')
};

// Canvas 2D contexts
const ctx = dom.canvas.getContext('2d');
const ctxVel = dom.chartVelocity.getContext('2d');
const ctxHeight = dom.chartHeight.getContext('2d');
const ctxEnergy = dom.chartEnergy.getContext('2d');

// --- Physics Engine (RK4 Numerical Integration) ---

/**
 * Calculates accelerations and forces at state (x, y, vx, vy)
 */
function getDerivatives(vx, vy, massKg, areaM2, cd, rho, g, wind) {
  const vxRel = vx - wind;
  const vyRel = vy;
  const vRel = Math.hypot(vxRel, vyRel);

  // Drag Force: Fd = 0.5 * rho * Cd * A * vRel^2
  // Components: Fdx = -0.5 * rho * Cd * A * vRel * vxRel
  //             Fdy = -0.5 * rho * Cd * A * vRel * vyRel
  const dragCoeff = 0.5 * rho * cd * areaM2;
  const fdx = -dragCoeff * vRel * vxRel;
  const fdy = -dragCoeff * vRel * vyRel;

  const ax = fdx / massKg;
  const ay = -g + (fdy / massKg);

  const fd = Math.hypot(fdx, fdy);

  return { ax, ay, fdx, fdy, fd, vRel };
}

/**
 * Simulates projectile trajectory using Runge-Kutta 4th order integration
 */
function simulateTrajectory(params) {
  const {
    v0,
    angleDeg,
    y0,
    massKg,
    diameterM,
    cd,
    gravity: g,
    airDensity: rho,
    wind
  } = params;

  const areaM2 = Math.PI * Math.pow(diameterM / 2, 2);
  const angleRad = (angleDeg * Math.PI) / 180;

  let x = 0;
  let y = Math.max(0, y0);
  let vx = v0 * Math.cos(angleRad);
  let vy = v0 * Math.sin(angleRad);
  let t = 0;

  const dt = 0.002; // 2ms fine integration step for high precision
  const trajectory = [];

  let apexY = y;
  let apexX = x;
  let apexTime = 0;
  let passedApex = false;
  let transonicPoint = null;

  // Initial State Record
  const initialDeriv = getDerivatives(vx, vy, massKg, areaM2, cd, rho, g, wind);
  const initialV = Math.hypot(vx, vy);
  const initialEk = 0.5 * massKg * (initialV * initialV);
  const initialMach = initialV / SPEED_OF_SOUND;

  trajectory.push({
    t: 0,
    x: 0,
    y,
    vx,
    vy,
    v: initialV,
    mach: initialMach,
    ax: initialDeriv.ax,
    ay: initialDeriv.ay,
    fd: initialDeriv.fd,
    ek: initialEk
  });

  const maxSteps = 100000;
  let step = 0;

  while (step < maxSteps) {
    step++;

    // RK4 Stage 1
    const k1_vx = vx;
    const k1_vy = vy;
    const d1 = getDerivatives(vx, vy, massKg, areaM2, cd, rho, g, wind);
    const k1_ax = d1.ax;
    const k1_ay = d1.ay;

    // RK4 Stage 2
    const vx2 = vx + 0.5 * dt * k1_ax;
    const vy2 = vy + 0.5 * dt * k1_ay;
    const k2_vx = vx2;
    const k2_vy = vy2;
    const d2 = getDerivatives(vx2, vy2, massKg, areaM2, cd, rho, g, wind);
    const k2_ax = d2.ax;
    const k2_ay = d2.ay;

    // RK4 Stage 3
    const vx3 = vx + 0.5 * dt * k2_ax;
    const vy3 = vy + 0.5 * dt * k2_ay;
    const k3_vx = vx3;
    const k3_vy = vy3;
    const d3 = getDerivatives(vx3, vy3, massKg, areaM2, cd, rho, g, wind);
    const k3_ax = d3.ax;
    const k3_ay = d3.ay;

    // RK4 Stage 4
    const vx4 = vx + dt * k3_ax;
    const vy4 = vy + dt * k3_ay;
    const k4_vx = vx4;
    const k4_vy = vy4;
    const d4 = getDerivatives(vx4, vy4, massKg, areaM2, cd, rho, g, wind);
    const k4_ax = d4.ax;
    const k4_ay = d4.ay;

    // Update position and velocity
    const nextX = x + (dt / 6) * (k1_vx + 2 * k2_vx + 2 * k3_vx + k4_vx);
    const nextY = y + (dt / 6) * (k1_vy + 2 * k2_vy + 2 * k3_vy + k4_vy);
    const nextVx = vx + (dt / 6) * (k1_ax + 2 * k2_ax + 2 * k3_ax + k4_ax);
    const nextVy = vy + (dt / 6) * (k1_ay + 2 * k2_ay + 2 * k3_ay + k4_ay);
    const nextT = t + dt;

    const currentV = Math.hypot(nextVx, nextVy);
    const currentMach = currentV / SPEED_OF_SOUND;

    // Detect Transonic Barrier Crossing (Mach 1.0)
    if (!transonicPoint && initialMach >= 1.0 && currentMach <= 1.0) {
      transonicPoint = {
        x: nextX,
        y: nextY,
        t: nextT,
        v: currentV,
        mach: 1.0
      };
    }

    // Track Apex
    if (nextY > apexY) {
      apexY = nextY;
      apexX = nextX;
      apexTime = nextT;
    } else if (!passedApex && nextVy <= 0) {
      passedApex = true;
    }

    // Check Ground Impact (y <= 0)
    if (nextY <= 0) {
      const fraction = y / (y - nextY);
      const impactT = t + fraction * dt;
      const impactX = x + fraction * (nextX - x);
      const impactVx = vx + fraction * (nextVx - vx);
      const impactVy = vy + fraction * (nextVy - vy);
      const impactV = Math.hypot(impactVx, impactVy);
      const impactEk = 0.5 * massKg * (impactV * impactV);
      const impactMach = impactV / SPEED_OF_SOUND;
      const impactDeriv = getDerivatives(impactVx, impactVy, massKg, areaM2, cd, rho, g, wind);

      trajectory.push({
        t: impactT,
        x: impactX,
        y: 0,
        vx: impactVx,
        vy: impactVy,
        v: impactV,
        mach: impactMach,
        ax: impactDeriv.ax,
        ay: impactDeriv.ay,
        fd: impactDeriv.fd,
        ek: impactEk
      });
      break;
    }

    x = nextX;
    y = nextY;
    vx = nextVx;
    vy = nextVy;
    t = nextT;

    const currentEk = 0.5 * massKg * (currentV * currentV);

    // Save sampled point
    if (step % 2 === 0 || step === 1) {
      trajectory.push({
        t,
        x,
        y,
        vx,
        vy,
        v: currentV,
        mach: currentMach,
        ax: d4.ax,
        ay: d4.ay,
        fd: d4.fd,
        ek: currentEk
      });
    }
  }

  // Final summary stats
  const lastPoint = trajectory[trajectory.length - 1];
  const impactAngle = Math.abs(Math.atan2(lastPoint.vy, lastPoint.vx) * (180 / Math.PI));
  const momentum = massKg * lastPoint.v;

  return {
    trajectory,
    range: lastPoint.x,
    maxHeight: apexY,
    apexX,
    apexTime,
    flightTime: lastPoint.t,
    impactVelocity: lastPoint.v,
    impactMach: lastPoint.mach,
    impactAngle,
    impactEnergy: lastPoint.ek,
    momentum,
    transonicPoint
  };
}

/**
 * Analytical Ideal (Vacuum) Trajectory
 */
function simulateIdealTrajectory(v0, angleDeg, y0, g) {
  const angleRad = (angleDeg * Math.PI) / 180;
  const vx = v0 * Math.cos(angleRad);
  const vy0 = v0 * Math.sin(angleRad);

  let flightTime = 0;
  if (g > 0) {
    const discriminant = vy0 * vy0 + 2 * g * y0;
    flightTime = (vy0 + Math.sqrt(Math.max(0, discriminant))) / g;
  } else {
    flightTime = 10;
  }

  const range = vx * flightTime;
  const maxHeight = y0 + (vy0 > 0 && g > 0 ? (vy0 * vy0) / (2 * g) : 0);

  const points = [];
  const numSteps = 200;
  for (let i = 0; i <= numSteps; i++) {
    const t = (i / numSteps) * flightTime;
    const x = vx * t;
    const y = Math.max(0, y0 + vy0 * t - 0.5 * g * t * t);
    points.push({ t, x, y });
  }

  return {
    points,
    range,
    maxHeight,
    flightTime
  };
}

// --- Optics & Line of Sight (LOS) Solver ---
function computeLineOfSight() {
  const hScopeM = state.sightHeightCm / 100;
  const dZeroM = state.zeroDistanceM;

  state.losIntercept = state.y0 + hScopeM;

  // Find projectile height at zero distance
  const stateAtZero = getInterpolatedStateAtX(dZeroM);
  const yAtZero = stateAtZero ? stateAtZero.y : state.y0;

  // LOS slope: (y(dZero) - yScope(0)) / dZero
  state.losSlope = (yAtZero - state.losIntercept) / dZeroM;

  // Enrich trajectory points with LOS drop, MRAD, MOA
  for (let p of state.trajectory) {
    const yLos = state.losIntercept + state.losSlope * p.x;
    const dropM = p.y - yLos; // >0 is high (impact above aim), <0 is low (drop)
    p.yLos = yLos;
    p.dropCm = dropM * 100;

    if (p.x > 0.1) {
      // Angular drop in Milliradians: -Drop / Dist * 1000
      p.mrad = -(dropM / p.x) * 1000;
      p.moa = p.mrad * 3.4377468;
    } else {
      p.mrad = 0;
      p.moa = 0;
    }
  }
}

function getInterpolatedStateAtX(targetX) {
  const traj = state.trajectory;
  if (!traj || traj.length === 0) return null;
  if (targetX <= 0) return traj[0];
  if (targetX >= traj[traj.length - 1].x) return traj[traj.length - 1];

  let low = 0;
  let high = traj.length - 1;

  while (low <= high) {
    const mid = (low + high) >> 1;
    if (traj[mid].x === targetX) return traj[mid];
    if (traj[mid].x < targetX) low = mid + 1;
    else high = mid - 1;
  }

  const i0 = Math.max(0, high);
  const i1 = Math.min(traj.length - 1, low);
  const p0 = traj[i0];
  const p1 = traj[i1];

  if (p0.x === p1.x) return p0;
  const factor = (targetX - p0.x) / (p1.x - p0.x);

  return {
    t: p0.t + factor * (p1.t - p0.t),
    x: targetX,
    y: p0.y + factor * (p1.y - p0.y),
    vx: p0.vx + factor * (p1.vx - p0.vx),
    vy: p0.vy + factor * (p1.vy - p0.vy),
    v: p0.v + factor * (p1.v - p0.v),
    mach: p0.mach + factor * (p1.mach - p0.mach),
    fd: p0.fd + factor * (p1.fd - p0.fd),
    ek: p0.ek + factor * (p1.ek - p0.ek)
  };
}

// --- Main Compute & Update Function ---
function computeSimulation() {
  const massKg = state.massGrains * GRAIN_TO_KG;
  const diameterM = state.diameterMm / 1000;

  const simResult = simulateTrajectory({
    v0: state.v0,
    angleDeg: state.angleDeg,
    y0: state.y0,
    massKg,
    diameterM,
    cd: state.cd,
    gravity: state.gravity,
    airDensity: state.airDensity,
    wind: state.wind
  });

  state.trajectory = simResult.trajectory;
  state.transonicPoint = simResult.transonicPoint;

  state.metrics = {
    range: simResult.range,
    maxHeight: simResult.maxHeight,
    apexX: simResult.apexX,
    apexTime: simResult.apexTime,
    flightTime: simResult.flightTime,
    impactVelocity: simResult.impactVelocity,
    impactMach: simResult.impactMach,
    impactAngle: simResult.impactAngle,
    impactEnergy: simResult.impactEnergy,
    momentum: simResult.momentum,
    transonicDist: simResult.transonicPoint ? simResult.transonicPoint.x : null,
    transonicTime: simResult.transonicPoint ? simResult.transonicPoint.t : null
  };

  // Compute Optics Line of Sight (LOS) and Angular Corrections
  computeLineOfSight();

  // Ideal Vacuum
  const idealResult = simulateIdealTrajectory(state.v0, state.angleDeg, state.y0, state.gravity);
  state.idealTrajectory = idealResult.points;
  state.metrics.idealRange = idealResult.range;
  state.metrics.idealMaxHeight = idealResult.maxHeight;
  state.metrics.idealFlightTime = idealResult.flightTime;

  // Sweep (15, 30, 45, 60, 75)
  if (state.showSweep) {
    state.sweepTrajectories = [15, 30, 45, 60, 75].map(sweepAngle => {
      return {
        angle: sweepAngle,
        data: simulateTrajectory({
          v0: state.v0,
          angleDeg: sweepAngle,
          y0: state.y0,
          massKg,
          diameterM,
          cd: state.cd,
          gravity: state.gravity,
          airDensity: state.airDensity,
          wind: state.wind
        })
      };
    });
  } else {
    state.sweepTrajectories = [];
  }

  // Update Scrubber Max
  dom.timeScrubber.max = state.metrics.flightTime.toFixed(2);
  dom.scrubMaxTime.textContent = state.metrics.flightTime.toFixed(2) + 's';

  if (state.currentTime > state.metrics.flightTime) {
    state.currentTime = state.metrics.flightTime;
  }

  updateMetricsUI();
  updateTargetHitAnalysis();
  renderAnalyticalCharts();
  renderDopeTable();
}

/**
 * Update UI Metric Cards & Telemetry
 */
function updateMetricsUI() {
  // Range
  if (state.metrics.range >= 1000) {
    dom.resRange.textContent = (state.metrics.range / 1000).toFixed(2);
    if (dom.resRange.nextElementSibling) dom.resRange.nextElementSibling.textContent = 'km';
  } else {
    dom.resRange.textContent = state.metrics.range.toFixed(2);
    if (dom.resRange.nextElementSibling) dom.resRange.nextElementSibling.textContent = 'm';
  }

  // Max Height
  if (state.metrics.maxHeight >= 1000) {
    dom.resMaxHeight.textContent = (state.metrics.maxHeight / 1000).toFixed(2);
    if (dom.resMaxHeight.nextElementSibling) dom.resMaxHeight.nextElementSibling.textContent = 'km';
  } else {
    dom.resMaxHeight.textContent = state.metrics.maxHeight.toFixed(2);
    if (dom.resMaxHeight.nextElementSibling) dom.resMaxHeight.nextElementSibling.textContent = 'm';
  }

  dom.resFlightTime.textContent = state.metrics.flightTime.toFixed(2);
  dom.resImpactVelocity.textContent = state.metrics.impactVelocity.toFixed(1);
  dom.resImpactAngle.textContent = `Angolo: ${state.metrics.impactAngle.toFixed(1)}° (M ${state.metrics.impactMach.toFixed(2)})`;
  
  // Impact Energy
  if (state.metrics.impactEnergy >= 10000) {
    dom.resImpactEnergy.textContent = (state.metrics.impactEnergy / 1000).toFixed(1);
    if (dom.resImpactEnergy.nextElementSibling) dom.resImpactEnergy.nextElementSibling.textContent = 'kJ';
  } else {
    dom.resImpactEnergy.textContent = state.metrics.impactEnergy.toFixed(1);
    if (dom.resImpactEnergy.nextElementSibling) dom.resImpactEnergy.nextElementSibling.textContent = 'J';
  }
  dom.resMomentum.textContent = `Impulso: ${state.metrics.momentum.toFixed(2)} N·s`;

  dom.subRangeIdeal.textContent = `Ideale: ${state.metrics.idealRange.toFixed(1)} m`;
  dom.subHeightIdeal.textContent = `Ideale: ${state.metrics.idealMaxHeight.toFixed(1)} m`;
  dom.subTimeIdeal.textContent = `Ideale: ${state.metrics.idealFlightTime.toFixed(2)} s`;

  // Update DOPE Badges
  dom.dopeZeroLabel.textContent = `${state.zeroDistanceM} m`;
  dom.dopeSightLabel.textContent = `${state.sightHeightCm.toFixed(1)} cm`;
  dom.dopeTransonicLabel.textContent = state.metrics.transonicDist 
    ? `${state.metrics.transonicDist.toFixed(0)} m (t=${state.metrics.transonicTime.toFixed(2)}s)`
    : (state.v0 > SPEED_OF_SOUND ? 'Oltre gittata' : 'Sottosonico alla volata');
}

/**
 * Render DOPE Drop Chart Table
 */
function renderDopeTable() {
  if (!dom.dopeTableBody || !state.trajectory || state.trajectory.length === 0) return;

  const step = parseInt(dom.selectDopeStep.value) || state.dopeStep;
  const maxDist = Math.min(Math.ceil(state.metrics.range / step) * step, state.metrics.range);
  const rows = [];

  for (let dist = 0; dist <= maxDist; dist += step) {
    const point = getInterpolatedStateAtX(dist);
    if (!point) continue;

    const yLos = state.losIntercept + state.losSlope * dist;
    const dropM = point.y - yLos;
    const dropCm = dropM * 100;

    let mrad = 0;
    let moa = 0;
    let clicksMil = 0;
    let clicksMoa = 0;

    if (dist > 0.1) {
      mrad = -(dropM / dist) * 1000;
      moa = mrad * 3.4377468;
      clicksMil = Math.round(mrad * 10);
      clicksMoa = Math.round(moa * 4);
    }

    const mach = point.v / SPEED_OF_SOUND;
    let regimeClass = 'speed-supersonic';
    let regimeLabel = 'Supersonico';

    if (mach >= 0.8 && mach <= 1.2) {
      regimeClass = 'speed-transonic';
      regimeLabel = 'Transonico ⚠️';
    } else if (mach < 0.8) {
      regimeClass = 'speed-subsonic';
      regimeLabel = 'Subsonico';
    }

    const isZeroRow = Math.abs(dist - state.zeroDistanceM) < step * 0.5;
    const isTransonicRow = state.transonicPoint && Math.abs(dist - state.transonicPoint.x) < step * 0.5;

    let rowClass = '';
    if (isZeroRow) rowClass = 'zero-row';
    else if (isTransonicRow) rowClass = 'transonic-row';

    const dropSign = dropCm >= 0 ? '+' : '';
    const mradSign = mrad >= 0 ? '+' : '';
    const moaSign = moa >= 0 ? '+' : '';

    rows.push(`
      <tr class="${rowClass}">
        <td><strong>${dist} m</strong> ${isZeroRow ? '<span class="badge" style="font-size:0.6rem;padding:0.1rem 0.3rem;">ZERO</span>' : ''}</td>
        <td>${point.y.toFixed(2)} m</td>
        <td style="color: ${dropCm >= 0 ? '#38bdf8' : '#f87171'}; font-weight:600;">${dropSign}${dropCm.toFixed(1)} cm</td>
        <td class="mrad-val">${mradSign}${mrad.toFixed(2)} Mil</td>
        <td><strong>${mradSign}${clicksMil}</strong> clk</td>
        <td class="moa-val">${moaSign}${moa.toFixed(2)} MOA</td>
        <td><strong>${moaSign}${clicksMoa}</strong> clk</td>
        <td class="${regimeClass}">M ${mach.toFixed(2)} (${point.v.toFixed(0)} m/s)</td>
        <td>${point.ek >= 1000 ? (point.ek / 1000).toFixed(1) + ' kJ' : point.ek.toFixed(0) + ' J'}</td>
        <td>${point.t.toFixed(3)} s</td>
      </tr>
    `);
  }

  dom.dopeTableBody.innerHTML = rows.join('');
}

/**
 * Get interpolated state at a specific time t
 */
function getStateAtTime(t) {
  const traj = state.trajectory;
  if (!traj || traj.length === 0) return null;
  if (t <= 0) return traj[0];
  if (t >= traj[traj.length - 1].t) return traj[traj.length - 1];

  let low = 0;
  let high = traj.length - 1;

  while (low <= high) {
    const mid = (low + high) >> 1;
    if (traj[mid].t === t) return traj[mid];
    if (traj[mid].t < t) low = mid + 1;
    else high = mid - 1;
  }

  const i0 = Math.max(0, high);
  const i1 = Math.min(traj.length - 1, low);
  const p0 = traj[i0];
  const p1 = traj[i1];

  if (p0.t === p1.t) return p0;
  const factor = (t - p0.t) / (p1.t - p0.t);

  const x = p0.x + factor * (p1.x - p0.x);
  const y = p0.y + factor * (p1.y - p0.y);
  const v = p0.v + factor * (p1.v - p0.v);
  const yLos = state.losIntercept + state.losSlope * x;
  const dropCm = (y - yLos) * 100;

  return {
    t,
    x,
    y,
    vx: p0.vx + factor * (p1.vx - p0.vx),
    vy: p0.vy + factor * (p1.vy - p0.vy),
    v,
    mach: v / SPEED_OF_SOUND,
    ax: p0.ax + factor * (p1.ax - p0.ax),
    ay: p0.ay + factor * (p1.ay - p0.ay),
    fd: p0.fd + factor * (p1.fd - p0.fd),
    ek: p0.ek + factor * (p1.ek - p0.ek),
    yLos,
    dropCm
  };
}

/**
 * Analyzes whether projectile intersects target
 */
function updateTargetHitAnalysis() {
  if (!state.showTarget) {
    dom.targetBanner.style.display = 'none';
    return;
  }
  dom.targetBanner.style.display = 'block';
  dom.targetPosLabel.textContent = `X: ${state.target.x.toFixed(1)}m | Y: ${state.target.y.toFixed(1)}m`;

  let minDistance = Infinity;
  let closestPoint = null;

  for (let p of state.trajectory) {
    const d = Math.hypot(p.x - state.target.x, p.y - state.target.y);
    if (d < minDistance) {
      minDistance = d;
      closestPoint = p;
    }
  }

  if (minDistance <= state.target.radius) {
    dom.targetStatusLabel.innerHTML = `<span style="color:#10b981;font-weight:bold;">🎯 BERSAGLIO COLPITO!</span> a t=${closestPoint.t.toFixed(2)}s (V=${closestPoint.v.toFixed(1)} m/s)`;
  } else {
    dom.targetStatusLabel.innerHTML = `Mancato per <strong>${minDistance.toFixed(1)}m</strong> (Trascina bersaglio per allineare)`;
  }
}

// --- Canvas Coordinate Transformations ---
function worldToCanvas(wx, wy) {
  const px = state.camera.panX + wx * state.camera.scale;
  const py = dom.canvas.height - (state.camera.panY + wy * state.camera.scale);
  return { x: px, y: py };
}

function canvasToWorld(cx, cy) {
  const wx = (cx - state.camera.panX) / state.camera.scale;
  const wy = (dom.canvas.height - cy - state.camera.panY) / state.camera.scale;
  return { x: Math.max(0, wx), y: Math.max(0, wy) };
}

/**
 * Automatically fits the entire trajectory inside the canvas
 */
function autoFitCamera() {
  const width = dom.canvas.width;
  const height = dom.canvas.height;

  const maxX = Math.max(
    state.metrics.range,
    state.compareVacuum ? state.metrics.idealRange : 0,
    state.showTarget ? state.target.x + 20 : 0,
    state.zeroDistanceM + 50,
    50
  );
  const maxY = Math.max(
    state.metrics.maxHeight,
    state.compareVacuum ? state.metrics.idealMaxHeight : 0,
    state.showTarget ? state.target.y + 10 : 0,
    state.y0 + (state.sightHeightCm / 100) + 5,
    20
  );

  const paddingX = 80;
  const paddingY = 80;

  const scaleX = (width - paddingX * 2) / maxX;
  const scaleY = (height - paddingY * 2) / maxY;

  state.camera.scale = Math.min(scaleX, scaleY);
  state.camera.panX = paddingX;
  state.camera.panY = paddingY;
}

// --- Particle Effects on Impact / Target Hit ---
function triggerImpactParticles(worldX, worldY, count = 25, color = '#38bdf8') {
  for (let i = 0; i < count; i++) {
    const angle = Math.random() * Math.PI;
    const speed = 20 + Math.random() * 80;
    state.particles.push({
      x: worldX,
      y: worldY,
      vx: Math.cos(angle) * speed,
      vy: Math.sin(angle) * speed,
      life: 1.0,
      decay: 0.02 + Math.random() * 0.03,
      color,
      size: 2 + Math.random() * 3
    });
  }
}

function updateParticles(dt) {
  for (let i = state.particles.length - 1; i >= 0; i--) {
    const p = state.particles[i];
    p.x += p.vx * dt;
    p.y += p.vy * dt;
    p.vy -= 9.8 * 4 * dt;
    p.life -= p.decay;

    if (p.life <= 0 || p.y < 0) {
      state.particles.splice(i, 1);
    }
  }
}

// --- Main Canvas Drawing ---
function drawCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = dom.canvas.getBoundingClientRect();

  if (dom.canvas.width !== rect.width * dpr || dom.canvas.height !== rect.height * dpr) {
    dom.canvas.width = rect.width * dpr;
    dom.canvas.height = rect.height * dpr;
    autoFitCamera();
  }

  ctx.save();
  ctx.scale(dpr, dpr);

  const cw = rect.width;
  const ch = rect.height;

  // Clear Canvas Background
  ctx.clearRect(0, 0, cw, ch);

  // Draw Grid Lines & Metric Units
  drawGrid(cw, ch);

  // Draw Historical Trajectory Trails
  for (let hist of state.historyTrajectories) {
    drawTrajectoryCurve(hist.points, 'rgba(148, 163, 184, 0.25)', 1.5, [4, 4]);
  }

  // Draw Multi-Angle Sweep
  if (state.showSweep && state.sweepTrajectories.length > 0) {
    state.sweepTrajectories.forEach(sweep => {
      const isCurrentAngle = Math.abs(sweep.angle - state.angleDeg) < 0.1;
      if (!isCurrentAngle) {
        drawTrajectoryCurve(sweep.data.trajectory, 'rgba(168, 85, 247, 0.4)', 1.5, [2, 2]);
        const endP = sweep.data.trajectory[sweep.data.trajectory.length - 1];
        const cp = worldToCanvas(endP.x, endP.y);
        ctx.fillStyle = 'rgba(168, 85, 247, 0.7)';
        ctx.font = '10px Inter, sans-serif';
        ctx.fillText(`${sweep.angle}° (${endP.x.toFixed(0)}m)`, cp.x + 4, cp.y - 4);
      }
    });
  }

  // Draw Line of Sight (LOS) & Zero Point
  if (state.showLos) {
    drawLineOfSight(cw);
  }

  // Draw Vacuum Ideal Trajectory
  if (state.compareVacuum && state.idealTrajectory.length > 0) {
    drawTrajectoryCurve(state.idealTrajectory, 'rgba(6, 182, 212, 0.65)', 2, [6, 4]);
    const idealEnd = state.idealTrajectory[state.idealTrajectory.length - 1];
    const cIdealEnd = worldToCanvas(idealEnd.x, idealEnd.y);
    ctx.fillStyle = '#06b6d4';
    ctx.beginPath();
    ctx.arc(cIdealEnd.x, cIdealEnd.y, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.font = '10px JetBrains Mono, monospace';
    ctx.fillText(`Vuoto: ${idealEnd.x.toFixed(1)}m`, cIdealEnd.x - 20, cIdealEnd.y - 10);
  }

  // Draw Main Realistic Trajectory Curve (with optional Mach segmented colors)
  if (state.trajectory.length > 0) {
    if (state.showMach) {
      drawMachSegmentedTrajectory();
    } else {
      drawTrajectoryCurve(state.trajectory, '#38bdf8', 3);
    }
  }

  // Draw Apex Marker & Ground Impact Marker
  drawKeyPoints();

  // Draw Transonic Crossing Marker (Mach 1.0)
  if (state.showMach && state.transonicPoint) {
    drawTransonicMarker();
  }

  // Draw Interactive Target
  if (state.showTarget) {
    drawTarget();
  }

  // Draw Ground Line & Base
  drawGround(cw);

  // Draw Projectile at Current Time & Vectors
  const curState = getStateAtTime(state.currentTime);
  if (curState) {
    drawProjectileAndVectors(curState);
    updateHUD(curState);
  }

  // Draw Particles
  drawParticles();

  ctx.restore();
}

/**
 * Draws coordinate grid with dynamic spacing
 */
function drawGrid(cw, ch) {
  const visibleWorldXMax = (cw - state.camera.panX) / state.camera.scale;
  const visibleWorldYMax = (ch - state.camera.panY) / state.camera.scale;

  const targetPixelSpacing = 80;
  const rawStep = targetPixelSpacing / state.camera.scale;
  const step = getNiceStep(rawStep);

  ctx.lineWidth = 1;
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
  ctx.fillStyle = 'rgba(156, 163, 175, 0.45)';
  ctx.font = '10px JetBrains Mono, monospace';

  // Vertical Grid Lines (X distance)
  const startX = 0;
  const endX = Math.ceil(visibleWorldXMax / step) * step;

  for (let x = startX; x <= endX; x += step) {
    const cp = worldToCanvas(x, 0);
    ctx.beginPath();
    ctx.moveTo(cp.x, 0);
    ctx.lineTo(cp.x, ch);
    ctx.stroke();

    if (x > 0) {
      const label = x >= 1000 ? `${x / 1000}km` : `${x}m`;
      ctx.fillText(label, cp.x + 3, ch - 8);
    }
  }

  // Horizontal Grid Lines (Y height)
  const endY = Math.ceil(visibleWorldYMax / step) * step;
  for (let y = step; y <= endY; y += step) {
    const cp = worldToCanvas(0, y);
    ctx.beginPath();
    ctx.moveTo(0, cp.y);
    ctx.lineTo(cw, cp.y);
    ctx.stroke();

    const label = y >= 1000 ? `${y / 1000}km` : `${y}m`;
    ctx.fillText(label, 8, cp.y - 3);
  }
}

function getNiceStep(val) {
  const exp = Math.floor(Math.log10(val));
  const frac = val / Math.pow(10, exp);
  let niceFrac = 1;
  if (frac > 5) niceFrac = 10;
  else if (frac > 2) niceFrac = 5;
  else if (frac > 1) niceFrac = 2;
  return niceFrac * Math.pow(10, exp);
}

/**
 * Draws a trajectory curve
 */
function drawTrajectoryCurve(points, color, width = 2, dash = []) {
  if (!points || points.length < 2) return;

  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash);
  ctx.beginPath();

  const start = worldToCanvas(points[0].x, points[0].y);
  ctx.moveTo(start.x, start.y);

  for (let i = 1; i < points.length; i++) {
    const p = worldToCanvas(points[i].x, points[i].y);
    ctx.lineTo(p.x, p.y);
  }

  ctx.stroke();
  ctx.restore();
}

/**
 * Draws multi-segment Mach color-coded trajectory
 */
function drawMachSegmentedTrajectory() {
  const traj = state.trajectory;
  if (!traj || traj.length < 2) return;

  ctx.save();
  ctx.lineWidth = 3.5;

  for (let i = 0; i < traj.length - 1; i++) {
    const p1 = traj[i];
    const p2 = traj[i + 1];
    const c1 = worldToCanvas(p1.x, p1.y);
    const c2 = worldToCanvas(p2.x, p2.y);

    const avgMach = (p1.mach + p2.mach) / 2;

    if (avgMach > 1.2) {
      ctx.strokeStyle = '#38bdf8'; // Supersonico (Ciano brillante)
    } else if (avgMach >= 0.8) {
      ctx.strokeStyle = '#f59e0b'; // Transonico (Ambra/Giallo)
    } else {
      ctx.strokeStyle = '#10b981'; // Subsonico (Smeraldo)
    }

    ctx.beginPath();
    ctx.moveTo(c1.x, c1.y);
    ctx.lineTo(c2.x, c2.y);
    ctx.stroke();
  }

  ctx.restore();
}

/**
 * Draws Line of Sight (LOS) from scope through zero point
 */
function drawLineOfSight(cw) {
  const maxX = Math.max(state.metrics.range * 1.2, state.zeroDistanceM * 1.5, 200);
  const startP = worldToCanvas(0, state.losIntercept);
  const endP = worldToCanvas(maxX, state.losIntercept + state.losSlope * maxX);

  ctx.save();
  ctx.strokeStyle = 'rgba(239, 68, 68, 0.75)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([5, 4]);

  ctx.beginPath();
  ctx.moveTo(startP.x, startP.y);
  ctx.lineTo(endP.x, endP.y);
  ctx.stroke();

  // Zero Point Reticle Marker
  const zeroY = state.losIntercept + state.losSlope * state.zeroDistanceM;
  const zeroP = worldToCanvas(state.zeroDistanceM, zeroY);

  ctx.setLineDash([]);
  ctx.strokeStyle = '#10b981';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(zeroP.x, zeroP.y, 6, 0, Math.PI * 2);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(zeroP.x - 9, zeroP.y);
  ctx.lineTo(zeroP.x + 9, zeroP.y);
  ctx.moveTo(zeroP.x, zeroP.y - 9);
  ctx.lineTo(zeroP.x, zeroP.y + 9);
  ctx.stroke();

  ctx.fillStyle = '#10b981';
  ctx.font = '10px JetBrains Mono, monospace';
  ctx.fillText(`Zero (${state.zeroDistanceM}m)`, zeroP.x - 30, zeroP.y - 14);

  // Scope origin dot
  ctx.fillStyle = '#ef4444';
  ctx.beginPath();
  ctx.arc(startP.x, startP.y, 4, 0, Math.PI * 2);
  ctx.fill();

  ctx.restore();
}

/**
 * Draws Transonic Crossing Marker (Mach 1.0)
 */
function drawTransonicMarker() {
  const tp = state.transonicPoint;
  if (!tp) return;

  const cp = worldToCanvas(tp.x, tp.y);

  ctx.save();
  ctx.fillStyle = '#f59e0b';
  ctx.strokeStyle = '#fef08a';
  ctx.lineWidth = 2;

  // Diamond shape
  ctx.beginPath();
  ctx.moveTo(cp.x, cp.y - 7);
  ctx.lineTo(cp.x + 7, cp.y);
  ctx.lineTo(cp.x, cp.y + 7);
  ctx.lineTo(cp.x - 7, cp.y);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();

  ctx.font = '10px JetBrains Mono, monospace';
  ctx.fillStyle = '#fbbf24';
  ctx.fillText(`Transonico Mach 1.0 (${tp.x.toFixed(0)}m)`, cp.x + 10, cp.y - 8);

  ctx.restore();
}

/**
 * Draws Ground & Terrain
 */
function drawGround(cw) {
  const groundY = dom.canvas.height - state.camera.panY;

  ctx.save();
  ctx.strokeStyle = '#10b981';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, groundY);
  ctx.lineTo(cw, groundY);
  ctx.stroke();

  // Launch Cannon / Base representation
  const launchP = worldToCanvas(0, state.y0);
  ctx.fillStyle = '#3b82f6';
  ctx.beginPath();
  ctx.arc(launchP.x, launchP.y, 5, 0, Math.PI * 2);
  ctx.fill();

  // Scope mount representation
  if (state.sightHeightCm > 0) {
    const scopeP = worldToCanvas(0, state.losIntercept);
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(launchP.x, launchP.y);
    ctx.lineTo(scopeP.x, scopeP.y);
    ctx.stroke();
  }

  // Launch Angle Direction Barrel
  const angleRad = (state.angleDeg * Math.PI) / 180;
  const barrelLen = 22;
  ctx.strokeStyle = '#60a5fa';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(launchP.x, launchP.y);
  ctx.lineTo(launchP.x + Math.cos(angleRad) * barrelLen, launchP.y - Math.sin(angleRad) * barrelLen);
  ctx.stroke();

  ctx.restore();
}

/**
 * Draws Apex & Impact Keypoints
 */
function drawKeyPoints() {
  if (!state.trajectory || state.trajectory.length === 0) return;

  // Apex (Vertex)
  const apexP = worldToCanvas(state.metrics.apexX, state.metrics.maxHeight);
  ctx.fillStyle = '#f59e0b';
  ctx.beginPath();
  ctx.arc(apexP.x, apexP.y, 4, 0, Math.PI * 2);
  ctx.fill();

  ctx.font = '10px JetBrains Mono, monospace';
  ctx.fillStyle = '#fbbf24';
  ctx.fillText(`Apice: ${state.metrics.maxHeight.toFixed(1)}m`, apexP.x - 25, apexP.y - 12);

  // Impact
  const impactP = worldToCanvas(state.metrics.range, 0);
  ctx.fillStyle = '#ef4444';
  ctx.beginPath();
  ctx.arc(impactP.x, impactP.y, 4, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = '#f87171';
  ctx.fillText(`Impatto: ${state.metrics.range.toFixed(1)}m`, impactP.x - 25, impactP.y + 16);
}

/**
 * Draws Interactive Target
 */
function drawTarget() {
  const cp = worldToCanvas(state.target.x, state.target.y);
  const rPx = Math.max(8, state.target.radius * state.camera.scale);

  ctx.save();
  // Outer Ring
  ctx.fillStyle = 'rgba(239, 68, 68, 0.3)';
  ctx.strokeStyle = '#ef4444';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(cp.x, cp.y, rPx, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  // Inner Ring
  ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
  ctx.beginPath();
  ctx.arc(cp.x, cp.y, rPx * 0.6, 0, Math.PI * 2);
  ctx.fill();

  // Bullseye Center
  ctx.fillStyle = '#ef4444';
  ctx.beginPath();
  ctx.arc(cp.x, cp.y, rPx * 0.25, 0, Math.PI * 2);
  ctx.fill();

  // Target Flag Pole
  ctx.strokeStyle = '#f87171';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(cp.x, cp.y);
  ctx.lineTo(cp.x, cp.y - 25);
  ctx.lineTo(cp.x + 12, cp.y - 19);
  ctx.lineTo(cp.x, cp.y - 13);
  ctx.stroke();

  ctx.fillStyle = '#f87171';
  ctx.fill();

  ctx.restore();
}

/**
 * Draws projectile sphere and force/velocity vector arrows
 */
function drawProjectileAndVectors(p) {
  const cp = worldToCanvas(p.x, p.y);

  ctx.save();

  // Draw Vectors
  if (state.showVectors) {
    const vectorScale = 0.25;

    // Velocity Vector (Cyan)
    drawVectorArrow(cp.x, cp.y, p.vx * vectorScale, -p.vy * vectorScale, '#38bdf8', 'v');

    // Drag Force Vector (Rose/Red)
    if (p.fd > 0.001) {
      const massKg = state.massGrains * GRAIN_TO_KG;
      const fdx = p.ax * massKg;
      const fdy = (p.ay + state.gravity) * massKg;
      const fScale = 15.0;
      drawVectorArrow(cp.x, cp.y, fdx * fScale, -fdy * fScale, '#f43f5e', 'Fd');
    }

    // Gravity Vector (Amber)
    const gScale = 3.5;
    drawVectorArrow(cp.x, cp.y, 0, state.gravity * gScale, '#f59e0b', 'g');
  }

  // Projectile Mach Glow
  let glowColor = 'rgba(56, 189, 248, 0.9)';
  if (p.mach >= 0.8 && p.mach <= 1.2) {
    glowColor = 'rgba(245, 158, 11, 0.9)';
  } else if (p.mach < 0.8) {
    glowColor = 'rgba(16, 185, 129, 0.9)';
  }

  const glow = ctx.createRadialGradient(cp.x, cp.y, 2, cp.x, cp.y, 14);
  glow.addColorStop(0, glowColor);
  glow.addColorStop(1, 'rgba(0, 0, 0, 0)');
  ctx.fillStyle = glow;
  ctx.beginPath();
  ctx.arc(cp.x, cp.y, 14, 0, Math.PI * 2);
  ctx.fill();

  // Projectile Body
  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.arc(cp.x, cp.y, 5, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#0284c7';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  ctx.restore();
}

function drawVectorArrow(fromX, fromY, dx, dy, color, label) {
  const len = Math.hypot(dx, dy);
  if (len < 4) return;

  const toX = fromX + dx;
  const toY = fromY + dy;
  const headLen = Math.min(8, len * 0.4);
  const angle = Math.atan2(dy, dx);

  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2;

  // Arrow line
  ctx.beginPath();
  ctx.moveTo(fromX, fromY);
  ctx.lineTo(toX, toY);
  ctx.stroke();

  // Arrow head
  ctx.beginPath();
  ctx.moveTo(toX, toY);
  ctx.lineTo(toX - headLen * Math.cos(angle - Math.PI / 6), toY - headLen * Math.sin(angle - Math.PI / 6));
  ctx.lineTo(toX - headLen * Math.cos(angle + Math.PI / 6), toY - headLen * Math.sin(angle + Math.PI / 6));
  ctx.closePath();
  ctx.fill();

  // Label
  ctx.font = '9px JetBrains Mono, monospace';
  ctx.fillText(label, toX + 4, toY - 4);
}

function drawParticles() {
  for (let p of state.particles) {
    const cp = worldToCanvas(p.x, p.y);
    ctx.fillStyle = p.color;
    ctx.globalAlpha = Math.max(0, p.life);
    ctx.beginPath();
    ctx.arc(cp.x, cp.y, p.size, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.globalAlpha = 1.0;
}

/**
 * Updates Floating HUD values
 */
function updateHUD(p) {
  dom.hudTime.textContent = p.t.toFixed(3) + ' s';
  dom.hudX.textContent = p.x.toFixed(1) + ' m';
  dom.hudY.textContent = p.y.toFixed(1) + ' m';
  dom.hudV.textContent = p.v.toFixed(1) + ' m/s';

  const mach = p.mach;
  let regimeText = 'Sup.';
  if (mach >= 0.8 && mach <= 1.2) regimeText = 'Trans. ⚠️';
  else if (mach < 0.8) regimeText = 'Sub.';
  dom.hudMach.textContent = `M ${mach.toFixed(2)} [${regimeText}]`;

  const dropSign = p.dropCm >= 0 ? '+' : '';
  dom.hudDrop.textContent = `${dropSign}${p.dropCm.toFixed(1)} cm`;

  dom.hudFd.textContent = p.fd.toFixed(2) + ' N';
  dom.hudEk.textContent = (p.ek >= 1000 ? (p.ek / 1000).toFixed(1) + ' kJ' : p.ek.toFixed(1) + ' J');

  dom.scrubCurrentTime.textContent = p.t.toFixed(2) + 's';
  dom.timeScrubber.value = p.t.toFixed(2);
}

// --- Analytical Charts Rendering ---
function renderAnalyticalCharts() {
  renderLineChart(dom.chartVelocity, ctxVel, state.trajectory, 't', 'v', 'Velocità (m/s)', '#38bdf8');
  renderLineChart(dom.chartHeight, ctxHeight, state.trajectory, 't', 'y', 'Quota (m)', '#10b981');
  renderLineChart(dom.chartEnergy, ctxEnergy, state.trajectory, 'x', 'ek', 'Energia (J)', '#f59e0b');
}

function renderLineChart(canvas, cCtx, data, xKey, yKey, label, strokeColor) {
  if (!data || data.length < 2) return;

  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  if (canvas.width !== rect.width * dpr || canvas.height !== rect.height * dpr) {
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
  }

  cCtx.save();
  cCtx.scale(dpr, dpr);

  const w = rect.width;
  const h = rect.height;
  cCtx.clearRect(0, 0, w, h);

  const padLeft = 45;
  const padBottom = 25;
  const padTop = 15;
  const padRight = 20;

  const plotW = w - padLeft - padRight;
  const plotH = h - padTop - padBottom;

  let maxX = data[data.length - 1][xKey] || 1;
  let maxY = 0;
  for (let d of data) {
    if (d[yKey] > maxY) maxY = d[yKey];
  }
  if (maxY === 0) maxY = 1;

  // Grid
  cCtx.strokeStyle = 'rgba(255, 255, 255, 0.06)';
  cCtx.lineWidth = 1;
  cCtx.beginPath();
  for (let i = 0; i <= 4; i++) {
    const yVal = padTop + (plotH / 4) * i;
    cCtx.moveTo(padLeft, yVal);
    cCtx.lineTo(w - padRight, yVal);
  }
  cCtx.stroke();

  // Labels
  cCtx.fillStyle = 'rgba(156, 163, 175, 0.7)';
  cCtx.font = '9px JetBrains Mono, monospace';
  cCtx.fillText(maxY.toFixed(1), 5, padTop + 8);
  cCtx.fillText((maxY / 2).toFixed(1), 5, padTop + plotH / 2);
  cCtx.fillText('0', 15, h - padBottom);

  cCtx.fillText(`0`, padLeft, h - 8);
  cCtx.fillText(`${maxX.toFixed(1)} ${xKey === 't' ? 's' : 'm'}`, w - padRight - 35, h - 8);

  // Line Plot
  cCtx.strokeStyle = strokeColor;
  cCtx.lineWidth = 2;
  cCtx.beginPath();

  for (let i = 0; i < data.length; i++) {
    const d = data[i];
    const px = padLeft + (d[xKey] / maxX) * plotW;
    const py = padTop + plotH - (d[yKey] / maxY) * plotH;
    if (i === 0) cCtx.moveTo(px, py);
    else cCtx.lineTo(px, py);
  }
  cCtx.stroke();

  // Cursor indicator for current time
  const curState = getStateAtTime(state.currentTime);
  if (curState) {
    const curXVal = curState[xKey];
    const curPx = padLeft + (curXVal / maxX) * plotW;
    cCtx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
    cCtx.setLineDash([3, 3]);
    cCtx.beginPath();
    cCtx.moveTo(curPx, padTop);
    cCtx.lineTo(curPx, h - padBottom);
    cCtx.stroke();
    cCtx.setLineDash([]);
  }

  // Chart Title
  cCtx.fillStyle = strokeColor;
  cCtx.font = '10px Inter, sans-serif';
  cCtx.fillText(label, padLeft + 5, padTop + 12);

  cCtx.restore();
}

// --- Main Animation Loop ---
let lastFrameTime = performance.now();

function animationLoop(timestamp) {
  const dt = Math.min((timestamp - lastFrameTime) / 1000, 0.1);
  lastFrameTime = timestamp;

  if (state.isPlaying) {
    state.currentTime += dt * state.playbackSpeed;

    // Check end of flight
    if (state.currentTime >= state.metrics.flightTime) {
      state.currentTime = state.metrics.flightTime;
      state.isPlaying = false;
      updatePlayButtonUI();

      // Trigger impact explosion particles
      const last = state.trajectory[state.trajectory.length - 1];
      triggerImpactParticles(last.x, last.y, 35, '#38bdf8');
    }
  }

  updateParticles(dt);
  drawCanvas();

  requestAnimationFrame(animationLoop);
}

function updatePlayButtonUI() {
  if (state.isPlaying) {
    dom.iconPlay.style.display = 'none';
    dom.iconPause.style.display = 'inline';
    dom.labelPlay.textContent = 'Pausa';
  } else {
    dom.iconPlay.style.display = 'inline';
    dom.iconPause.style.display = 'none';
    if (state.currentTime <= 0.001) {
      dom.labelPlay.textContent = 'Lancia / Avvia';
    } else if (state.currentTime >= state.metrics.flightTime) {
      dom.labelPlay.textContent = 'Rilancia';
    } else {
      dom.labelPlay.textContent = 'Riprendi';
    }
  }
}

// --- Event Listeners & Interactions ---

function setupEventListeners() {
  // Velocity Input
  dom.velocity.addEventListener('input', (e) => {
    state.v0 = parseFloat(e.target.value);
    dom.valVelocity.textContent = `${state.v0} m/s`;
    dom.valVelocityKmh.textContent = `${Math.round(state.v0 * 3.6)} km/h`;
    computeSimulation();
  });

  // Angle Input
  dom.angle.addEventListener('input', (e) => {
    state.angleDeg = parseFloat(e.target.value);
    dom.valAngle.textContent = `${state.angleDeg.toFixed(1)}°`;
    computeSimulation();
  });

  // Height Input
  dom.height.addEventListener('input', (e) => {
    state.y0 = parseFloat(e.target.value);
    dom.valHeight.textContent = `${state.y0.toFixed(1)} m`;
    computeSimulation();
  });

  // Mass Input
  dom.mass.addEventListener('input', (e) => {
    state.massGrains = parseFloat(e.target.value);
    dom.valMass.textContent = `${Math.round(state.massGrains)} gr`;
    const grams = state.massGrains * GRAIN_TO_G;
    dom.valMassGrams.textContent = grams >= 1000 
      ? `${(grams / 1000).toFixed(2)} kg` 
      : `${grams.toFixed(2)} g`;
    computeSimulation();
  });

  // Diameter Input
  dom.diameter.addEventListener('input', (e) => {
    state.diameterMm = parseFloat(e.target.value);
    dom.valDiameter.textContent = `${state.diameterMm.toFixed(1)} mm`;
    computeSimulation();
  });

  // Cd Input
  dom.cd.addEventListener('input', (e) => {
    state.cd = parseFloat(e.target.value);
    dom.valCd.textContent = state.cd.toFixed(3);
    updateBcDisplay();
    computeSimulation();
  });

  // Gravity Select
  dom.gravity.addEventListener('change', (e) => {
    state.gravity = parseFloat(e.target.value);
    computeSimulation();
    autoFitCamera();
  });

  // Air Density Input
  dom.airDensity.addEventListener('input', (e) => {
    state.airDensity = parseFloat(e.target.value);
    dom.valAirDensity.textContent = `${state.airDensity.toFixed(3)} kg/m³`;
    computeSimulation();
  });

  // Wind Input
  dom.wind.addEventListener('input', (e) => {
    state.wind = parseFloat(e.target.value);
    dom.valWind.textContent = `${state.wind > 0 ? '+' : ''}${state.wind.toFixed(1)} m/s`;
    computeSimulation();
  });

  // Optics Inputs
  dom.sightHeight.addEventListener('input', (e) => {
    state.sightHeightCm = parseFloat(e.target.value);
    dom.valSightHeight.textContent = `${state.sightHeightCm.toFixed(1)} cm`;
    computeSimulation();
  });

  dom.zeroDistance.addEventListener('input', (e) => {
    state.zeroDistanceM = parseFloat(e.target.value);
    dom.valZeroDistance.textContent = `${state.zeroDistanceM} m`;
    computeSimulation();
  });

  dom.dragModel.addEventListener('change', (e) => {
    state.dragModel = e.target.value;
    updateBcDisplay();
  });

  // Toggles
  dom.toggleVacuum.addEventListener('change', (e) => {
    state.compareVacuum = e.target.checked;
    drawCanvas();
  });

  dom.toggleLos.addEventListener('change', (e) => {
    state.showLos = e.target.checked;
    drawCanvas();
  });

  dom.toggleMach.addEventListener('change', (e) => {
    state.showMach = e.target.checked;
    drawCanvas();
  });

  dom.toggleVectors.addEventListener('change', (e) => {
    state.showVectors = e.target.checked;
    drawCanvas();
  });

  dom.toggleTarget.addEventListener('change', (e) => {
    state.showTarget = e.target.checked;
    updateTargetHitAnalysis();
    drawCanvas();
  });

  dom.toggleSweep.addEventListener('change', (e) => {
    state.showSweep = e.target.checked;
    computeSimulation();
  });

  // DOPE Step select
  dom.selectDopeStep.addEventListener('change', (e) => {
    state.dopeStep = parseInt(e.target.value);
    renderDopeTable();
  });

  // Presets Click
  dom.presetBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      const presetKey = btn.dataset.preset;
      applyPreset(presetKey);

      dom.presetBtns.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
    });
  });

  // Play / Pause
  dom.btnPlayPause.addEventListener('click', () => {
    if (state.currentTime >= state.metrics.flightTime) {
      state.currentTime = 0;
    }
    state.isPlaying = !state.isPlaying;
    updatePlayButtonUI();
  });

  // Step Forward
  dom.btnStep.addEventListener('click', () => {
    state.isPlaying = false;
    updatePlayButtonUI();
    state.currentTime = Math.min(state.metrics.flightTime, state.currentTime + 0.05);
  });

  // Restart Playback (No auto-play)
  dom.btnRestart.addEventListener('click', () => {
    state.currentTime = 0;
    state.isPlaying = false;
    updatePlayButtonUI();
  });

  // Time Scrubber
  dom.timeScrubber.addEventListener('input', (e) => {
    state.isPlaying = false;
    updatePlayButtonUI();
    state.currentTime = parseFloat(e.target.value);
  });

  // Speed Selector
  dom.speedPills.forEach(pill => {
    pill.addEventListener('click', () => {
      dom.speedPills.forEach(p => p.classList.remove('active'));
      pill.classList.add('active');
      state.playbackSpeed = parseFloat(pill.dataset.speed);
    });
  });

  // Zoom / View Toolbar
  dom.btnZoomIn.addEventListener('click', () => {
    state.camera.scale *= 1.25;
  });
  dom.btnZoomOut.addEventListener('click', () => {
    state.camera.scale /= 1.25;
  });
  dom.btnZoomFit.addEventListener('click', () => {
    autoFitCamera();
  });
  dom.btnClearTrails.addEventListener('click', () => {
    state.historyTrajectories = [];
    showToast('Traiettorie storiche eliminate');
  });

  // Canvas Mouse Pan & Zoom
  dom.canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
    const rect = dom.canvas.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;

    const before = canvasToWorld(mouseX, mouseY);
    state.camera.scale *= zoomFactor;
    const after = canvasToWorld(mouseX, mouseY);

    state.camera.panX += (after.x - before.x) * state.camera.scale;
    state.camera.panY += (after.y - before.y) * state.camera.scale;
  }, { passive: false });

  dom.canvas.addEventListener('mousedown', (e) => {
    const rect = dom.canvas.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;
    const worldP = canvasToWorld(mouseX, mouseY);

    if (state.showTarget) {
      const distToTarget = Math.hypot(worldP.x - state.target.x, worldP.y - state.target.y);
      if (distToTarget <= Math.max(state.target.radius * 2, 20 / state.camera.scale)) {
        state.target.isDragging = true;
        return;
      }
    }

    state.camera.isPanning = true;
    state.camera.startX = e.clientX - state.camera.panX;
    state.camera.startY = e.clientY + state.camera.panY;
  });

  window.addEventListener('mousemove', (e) => {
    if (state.target.isDragging) {
      const rect = dom.canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;
      const worldP = canvasToWorld(mouseX, mouseY);

      state.target.x = Math.max(5, worldP.x);
      state.target.y = Math.max(0, worldP.y);
      updateTargetHitAnalysis();
      return;
    }

    if (state.camera.isPanning) {
      state.camera.panX = e.clientX - state.camera.startX;
      state.camera.panY = -(e.clientY - state.camera.startY);
    }
  });

  window.addEventListener('mouseup', () => {
    state.target.isDragging = false;
    state.camera.isPanning = false;
  });

  // Touch Support
  let touchStartDist = 0;
  dom.canvas.addEventListener('touchstart', (e) => {
    if (e.touches.length === 1) {
      const rect = dom.canvas.getBoundingClientRect();
      const touchX = e.touches[0].clientX - rect.left;
      const touchY = e.touches[0].clientY - rect.top;
      const worldP = canvasToWorld(touchX, touchY);

      if (state.showTarget && Math.hypot(worldP.x - state.target.x, worldP.y - state.target.y) <= 30 / state.camera.scale) {
        state.target.isDragging = true;
      } else {
        state.camera.isPanning = true;
        state.camera.startX = e.touches[0].clientX - state.camera.panX;
        state.camera.startY = e.touches[0].clientY + state.camera.panY;
      }
    } else if (e.touches.length === 2) {
      touchStartDist = Math.hypot(
        e.touches[0].clientX - e.touches[1].clientX,
        e.touches[0].clientY - e.touches[1].clientY
      );
    }
  }, { passive: true });

  dom.canvas.addEventListener('touchmove', (e) => {
    if (e.touches.length === 1) {
      if (state.target.isDragging) {
        const rect = dom.canvas.getBoundingClientRect();
        const worldP = canvasToWorld(e.touches[0].clientX - rect.left, e.touches[0].clientY - rect.top);
        state.target.x = Math.max(5, worldP.x);
        state.target.y = Math.max(0, worldP.y);
        updateTargetHitAnalysis();
      } else if (state.camera.isPanning) {
        state.camera.panX = e.touches[0].clientX - state.camera.startX;
        state.camera.panY = -(e.touches[0].clientY - state.camera.startY);
      }
    } else if (e.touches.length === 2) {
      const dist = Math.hypot(
        e.touches[0].clientX - e.touches[1].clientX,
        e.touches[0].clientY - e.touches[1].clientY
      );
      if (touchStartDist > 0) {
        state.camera.scale *= dist / touchStartDist;
        touchStartDist = dist;
      }
    }
  }, { passive: true });

  dom.canvas.addEventListener('touchend', () => {
    state.target.isDragging = false;
    state.camera.isPanning = false;
    touchStartDist = 0;
  });

  // Tab switching
  dom.tabBtns.forEach(tab => {
    tab.addEventListener('click', () => {
      dom.tabBtns.forEach(t => t.classList.remove('active'));
      dom.tabPanes.forEach(p => p.classList.remove('active'));

      tab.classList.add('active');
      const targetPane = document.getElementById(tab.dataset.tab);
      if (targetPane) {
        targetPane.classList.add('active');
        renderAnalyticalCharts();
        if (tab.dataset.tab === 'tab-dope') {
          renderDopeTable();
        }
      }
    });
  });

  // Export CSV
  dom.btnExportCsv.addEventListener('click', exportTrajectoryCsv);

  // Reset Defaults
  dom.btnResetDefaults.addEventListener('click', () => {
    applyPreset('9mm');
    dom.presetBtns.forEach(b => b.classList.toggle('active', b.dataset.preset === '9mm'));
    showToast('Parametri ripristinati');
  });

  // Window resize handler
  window.addEventListener('resize', () => {
    autoFitCamera();
    renderAnalyticalCharts();
    renderDopeTable();
  });
}

function updateBcDisplay() {
  const sectionalDensity = (state.massGrains / 7000) / Math.pow(state.diameterMm / 25.4, 2);
  const estBc = (sectionalDensity / (state.cd / 0.47)).toFixed(3);
  dom.valBcDisplay.textContent = `BC G1 stimato: ${estBc}`;
}

/**
 * Apply Preset Configuration
 */
function applyPreset(key) {
  const p = PRESETS[key];
  if (!p) return;

  if (state.trajectory && state.trajectory.length > 0) {
    state.historyTrajectories.push({
      name: p.name,
      points: [...state.trajectory]
    });
    if (state.historyTrajectories.length > 4) {
      state.historyTrajectories.shift();
    }
  }

  state.v0 = p.v0;
  state.angleDeg = p.angle;
  state.y0 = p.y0;
  state.massGrains = p.massGrains;
  state.diameterMm = p.diameter;
  state.cd = p.cd;
  state.wind = p.wind;
  state.airDensity = p.airDensity;
  state.sightHeightCm = p.sightHeight || 4.5;
  state.zeroDistanceM = p.zeroDist || 100;

  // Update DOM
  dom.velocity.value = state.v0;
  dom.valVelocity.textContent = `${state.v0} m/s`;
  dom.valVelocityKmh.textContent = `${Math.round(state.v0 * 3.6)} km/h`;

  dom.angle.value = state.angleDeg;
  dom.valAngle.textContent = `${state.angleDeg.toFixed(1)}°`;

  dom.height.value = state.y0;
  dom.valHeight.textContent = `${state.y0.toFixed(1)} m`;

  dom.mass.value = state.massGrains;
  dom.valMass.textContent = `${Math.round(state.massGrains)} gr`;
  const pGrams = state.massGrains * GRAIN_TO_G;
  dom.valMassGrams.textContent = pGrams >= 1000 
    ? `${(pGrams / 1000).toFixed(2)} kg` 
    : `${pGrams.toFixed(2)} g`;

  dom.diameter.value = state.diameterMm;
  dom.valDiameter.textContent = `${state.diameterMm.toFixed(1)} mm`;

  dom.cd.value = state.cd;
  dom.valCd.textContent = state.cd.toFixed(3);

  dom.sightHeight.value = state.sightHeightCm;
  dom.valSightHeight.textContent = `${state.sightHeightCm.toFixed(1)} cm`;

  dom.zeroDistance.value = state.zeroDistanceM;
  dom.valZeroDistance.textContent = `${state.zeroDistanceM} m`;

  dom.airDensity.value = state.airDensity;
  dom.valAirDensity.textContent = `${state.airDensity.toFixed(3)} kg/m³`;

  dom.wind.value = state.wind;
  dom.valWind.textContent = `${state.wind.toFixed(1)} m/s`;

  updateBcDisplay();
  computeSimulation();

  state.target.x = Math.round(state.metrics.range * 0.85);
  state.target.y = 0;

  autoFitCamera();
  state.currentTime = 0;
  state.isPlaying = false; // No auto-play
  updatePlayButtonUI();
}

/**
 * Export Trajectory Data as CSV
 */
function exportTrajectoryCsv() {
  if (!state.trajectory || state.trajectory.length === 0) {
    showToast('Nessun dato da esportare');
    return;
  }

  let csvContent = 'data:text/csv;charset=utf-8,';
  csvContent += 'Tempo_s,Posizione_X_m,Posizione_Y_m,Velocita_X_ms,Velocita_Y_ms,Velocita_Totale_ms,Mach,Caduta_vs_Mira_cm,Correzione_MRAD,Correzione_MOA,Resistenza_Aria_N,Energia_Cinetica_J\n';

  state.trajectory.forEach(p => {
    csvContent += `${p.t.toFixed(4)},${p.x.toFixed(3)},${p.y.toFixed(3)},${p.vx.toFixed(2)},${p.vy.toFixed(2)},${p.v.toFixed(2)},${p.mach.toFixed(3)},${(p.dropCm || 0).toFixed(2)},${(p.mrad || 0).toFixed(2)},${(p.moa || 0).toFixed(2)},${p.fd.toFixed(3)},${p.ek.toFixed(2)}\n`;
  });

  const encodedUri = encodeURI(csvContent);
  const link = document.createElement('a');
  link.setAttribute('href', encodedUri);
  link.setAttribute('download', `balistica_traiettoria_${Date.now()}.csv`);
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);

  showToast('File CSV scaricato con successo');
}

/**
 * Toast feedback helper
 */
function showToast(message) {
  const existing = document.querySelector('.toast');
  if (existing) existing.remove();

  const toast = document.createElement('div');
  toast.className = 'toast';
  toast.textContent = message;
  document.body.appendChild(toast);

  setTimeout(() => {
    toast.remove();
  }, 2800);
}

// --- Initialization ---
window.addEventListener('DOMContentLoaded', () => {
  setupEventListeners();
  updateBcDisplay();
  computeSimulation();
  autoFitCamera();
  requestAnimationFrame(animationLoop);
});
