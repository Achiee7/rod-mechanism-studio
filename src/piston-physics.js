const FITTING_FRACTION = 0.1;
const BARREL_FRACTION = 1 - 2 * FITTING_FRACTION;
const MOUNT_TOLERANCE = 1e-6;

// Nominal pin-to-pin limits for the installed pose, in metres.
export function pistonLengthRanges(compressed, extended, mount) {
  const range = (low, high) => Number.isFinite(low) && Number.isFinite(high) && low <= high ? [low, high] : null;
  const ratio = 1 + BARREL_FRACTION;
  return {
    compressed: extended >= mount - MOUNT_TOLERANCE
      ? range(Math.max(0.001, extended / ratio), Math.min(extended, mount + MOUNT_TOLERANCE)) : null,
    extended: compressed > 0 && compressed <= mount + MOUNT_TOLERANCE
      ? range(Math.max(0.001, compressed, mount - MOUNT_TOLERANCE), compressed * ratio) : null,
  };
}

export function pistonGeometry(element, centerLength) {
  const compressed = element.minLength ?? element.length * 0.65;
  const extended = element.maxLength ?? element.length;
  const validCompressed = Number.isFinite(compressed) && compressed > 0;
  const buildLength = validCompressed ? compressed : Math.max(0.001, Number(element.length) || 0.001);
  const fittingLength = buildLength * FITTING_FRACTION;
  const barrelLength = buildLength * BARREL_FRACTION;
  const tolerance = Math.max(1e-6, buildLength * 1e-4);
  const issues = [];
  if (!validCompressed || !Number.isFinite(extended) || extended < compressed) {
    issues.push({ code: "piston-config", message: "Invalid compressed or extended length" });
  } else if (extended - compressed > barrelLength + tolerance) {
    issues.push({ code: "piston-config", message: "Stroke exceeds the available barrel length" });
  }
  const configurationValid = !issues.length;
  if (Number.isFinite(centerLength) && validCompressed && Number.isFinite(extended)) {
    if (centerLength < compressed - MOUNT_TOLERANCE) issues.push({ code: "stroke", message: "Below compressed length" });
    else if (centerLength > extended + MOUNT_TOLERANCE) issues.push({ code: "stroke", message: "Beyond extended length" });
  }
  return {
    fittingLength, barrelLength, barrelRadius: buildLength * 0.027,
    minLength: compressed, maxLength: extended, maxPhysicalStroke: barrelLength,
    configurationValid, valid: !issues.length, issues, code: issues[0]?.code,
    problem: issues.map(issue => issue.message).join(" · ") || undefined,
  };
}
