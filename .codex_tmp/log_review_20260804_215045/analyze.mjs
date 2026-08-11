import fs from "node:fs/promises";
import { Workbook } from "@oai/artifact-tool";

const csvPath = "C:/Users/watar/OneDrive/デスクトップ/Twelite_pc_tool/converted/20260804_215045_flash.csv";
const csvText = await fs.readFile(csvPath, "utf8");
const workbook = await Workbook.fromCSV(csvText, { sheetName: "Log" });

const overview = await workbook.inspect({
  kind: "workbook,sheet,table",
  maxChars: 3000,
  tableMaxRows: 4,
  tableMaxCols: 12,
  tableMaxCellChars: 60,
});

const sheet = workbook.worksheets.getItem("Log");
const values = sheet.getUsedRange(true).values;
const headers = values[0].map(String);
const rows = values.slice(1).map((cells, index) => {
  const row = { _row: index + 2 };
  headers.forEach((header, column) => { row[header] = cells[column]; });
  return row;
});

const n = (value) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
};
const s = (value) => value == null ? "" : String(value);
const ts = (row) => n(row.timestamp_ms) ?? 0;
const duration = (start, end) => Math.max(0, ts(end) - ts(start));
const displacement = (start, end, xKey, yKey) => {
  const x0 = n(start[xKey]);
  const y0 = n(start[yKey]);
  const x1 = n(end[xKey]);
  const y1 = n(end[yKey]);
  if ([x0, y0, x1, y1].some((v) => v == null)) return null;
  return Math.hypot(x1 - x0, y1 - y0);
};
const changed = (a, b, key) => s(a?.[key]) !== s(b?.[key]);
const compact = (row) => ({
  row: row._row,
  t_ms: ts(row),
  state: s(row.mission_state_name),
  jog_v: n(row.jog_velocity_mm_s),
  jog_w: n(row.jog_omega_rad_s),
  camera_valid: n(row.camera_valid),
  hash: s(row.camera_scene_hash_hex),
  scores: {
    blocked: n(row.stuck_score_wheel_blocked),
    slip: n(row.stuck_score_wheel_slip),
    rotation: n(row.stuck_score_rotation_blocked),
    trapped: n(row.stuck_score_body_trapped),
  },
  phase: s(row.stuck_verification_phase_name),
  trigger: s(row.stuck_trigger_reason_name),
  result: s(row.stuck_verification_result_name),
  hash_bits: n(row.stuck_hash_distance_bits),
  recurrence: n(row.stuck_recurrence_count),
  confirmed: s(row.stuck_reason_name),
  enc_v_l: n(row.encoder_left_velocity_mm_s),
  enc_v_r: n(row.encoder_right_velocity_mm_s),
  gps_samples: n(row.stuck_gps_sample_count),
  gps_radius_mm: n(row.stuck_gps_max_radius_mm),
  gps_enc_mm: n(row.stuck_gps_encoder_distance_mm),
  tilt_deg: (n(row.stuck_tilt_deg_x10) ?? 0) / 10,
});

const stateCounts = {};
for (const row of rows) {
  const key = s(row.mission_state_name);
  stateCounts[key] = (stateCounts[key] ?? 0) + 1;
}

const transitions = [];
for (let i = 1; i < rows.length; i++) {
  if (changed(rows[i - 1], rows[i], "mission_state_name")) {
    transitions.push({
      row: rows[i]._row,
      t_ms: ts(rows[i]),
      from: s(rows[i - 1].mission_state_name),
      to: s(rows[i].mission_state_name),
    });
  }
}

const scoreKeys = [
  "stuck_score_wheel_blocked",
  "stuck_score_wheel_slip",
  "stuck_score_rotation_blocked",
  "stuck_score_body_trapped",
];
const scoreMaxima = {};
for (const key of scoreKeys) {
  let best = null;
  for (const row of rows) {
    const value = n(row[key]);
    if (value != null && (best == null || value > best.value)) {
      best = { value, ...compact(row) };
    }
  }
  scoreMaxima[key] = best;
}

const diagnosticEvents = [];
for (let i = 0; i < rows.length; i++) {
  const row = rows[i];
  const prev = rows[i - 1];
  const notable = !prev ||
    changed(prev, row, "stuck_verification_phase_name") ||
    changed(prev, row, "stuck_trigger_reason_name") ||
    changed(prev, row, "stuck_verification_result_name") ||
    changed(prev, row, "stuck_reason_name") ||
    changed(prev, row, "camera_valid");
  if (notable && (
      s(row.stuck_verification_phase_name) !== "IDLE" ||
      s(row.stuck_trigger_reason_name) !== "NONE" ||
      s(row.stuck_verification_result_name) !== "NONE" ||
      s(row.stuck_reason_name) !== "NONE" ||
      n(row.camera_valid) === 1 ||
      (prev && n(prev.camera_valid) === 1))) {
    diagnosticEvents.push(compact(row));
  }
}

const cameraRows = rows.filter((row) => n(row.camera_valid) === 1);
const hashes = cameraRows.map((row) => s(row.camera_scene_hash_hex));
const uniqueHashes = [...new Set(hashes)];
const popcount64 = (hex) => {
  let value = BigInt(hex);
  let count = 0;
  while (value) { count += Number(value & 1n); value >>= 1n; }
  return count;
};
const cameraHashTransitions = [];
for (let i = 1; i < cameraRows.length; i++) {
  const before = s(cameraRows[i - 1].camera_scene_hash_hex);
  const after = s(cameraRows[i].camera_scene_hash_hex);
  if (before !== after) {
    cameraHashTransitions.push({
      row: cameraRows[i]._row,
      t_ms: ts(cameraRows[i]),
      bits: popcount64(before) + popcount64(after) -
        2 * popcount64(`0x${(BigInt(before) & BigInt(after)).toString(16)}`),
      before,
      after,
      state: s(cameraRows[i].mission_state_name),
    });
  }
}

const drivingWindows = [];
let windowStart = null;
for (let i = 0; i <= rows.length; i++) {
  const active = i < rows.length &&
    ((n(rows[i].jog_velocity_mm_s) ?? 0) !== 0 ||
     Math.abs(n(rows[i].jog_omega_rad_s) ?? 0) > 0.001);
  if (active && windowStart == null) windowStart = i;
  if (!active && windowStart != null) {
    const start = rows[windowStart];
    const end = rows[i - 1];
    const slice = rows.slice(windowStart, i);
    drivingWindows.push({
      start: compact(start),
      end: compact(end),
      duration_ms: duration(start, end),
      row_count: slice.length,
      fusion_displacement_mm: displacement(start, end, "x_mm", "y_mm"),
      gps_displacement_mm: displacement(start, end, "gps_x_mm", "gps_y_mm"),
      encoder_left_delta_mm: (n(end.encoder_left_mm) ?? 0) - (n(start.encoder_left_mm) ?? 0),
      encoder_right_delta_mm: (n(end.encoder_right_mm) ?? 0) - (n(start.encoder_right_mm) ?? 0),
      max_scores: Object.fromEntries(scoreKeys.map((key) => [
        key,
        Math.max(...slice.map((row) => n(row[key]) ?? 0)),
      ])),
      max_abs_tilt_deg: Math.max(...slice.map((row) => Math.abs((n(row.stuck_tilt_deg_x10) ?? 0) / 10))),
    });
    windowStart = null;
  }
}

const gaps = rows.slice(1).map((row, i) => ({
  row: row._row,
  dt: ts(row) - ts(rows[i]),
  dm: (n(row.message_number) ?? 0) - (n(rows[i].message_number) ?? 0),
}));

const suspendRows = rows.filter((row) => s(row.mission_state_name) === "STUCK_SUSPEND");
const output = {
  artifact_overview: overview.ndjson,
  row_count: rows.length,
  columns: headers.length,
  time: {
    first_ms: ts(rows[0]),
    last_ms: ts(rows.at(-1)),
    duration_ms: duration(rows[0], rows.at(-1)),
  },
  state_counts: stateCounts,
  state_transitions: transitions,
  score_maxima: scoreMaxima,
  diagnostic_events: diagnosticEvents.slice(0, 100),
  diagnostic_event_count: diagnosticEvents.length,
  camera: {
    valid_rows: cameraRows.length,
    first: cameraRows.length ? compact(cameraRows[0]) : null,
    last: cameraRows.length ? compact(cameraRows.at(-1)) : null,
    unique_hash_count: uniqueHashes.length,
    hash_transition_count: cameraHashTransitions.length,
    hash_transitions: cameraHashTransitions.slice(0, 100),
  },
  stuck_suspend: {
    rows: suspendRows.length,
    first: suspendRows.length ? compact(suspendRows[0]) : null,
    last: suspendRows.length ? compact(suspendRows.at(-1)) : null,
    duration_ms: suspendRows.length ? duration(suspendRows[0], suspendRows.at(-1)) : 0,
    camera_valid_rows: suspendRows.filter((row) => n(row.camera_valid) === 1).length,
  },
  driving_windows: drivingWindows,
  log_integrity: {
    max_timestamp_gap_ms: Math.max(...gaps.map((gap) => gap.dt)),
    gaps_over_150ms: gaps.filter((gap) => gap.dt > 150).slice(0, 30),
    message_number_nonunit: gaps.filter((gap) => gap.dm !== 1).slice(0, 30),
  },
};

console.log(JSON.stringify(output, null, 2));
