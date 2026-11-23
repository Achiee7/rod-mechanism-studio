const FITTING_FRACTION = 0.1;

export function pistonGeometry(element, centerLength) {
  const compressed = element.minLength ?? element.length * 0.65;
  const extended = element.maxLength ?? element.length;
  const validCompressed = Number.isFinite(compressed) && compressed > 0;
  const buildLength = validCompressed ? compressed : Math.max(0.001, Number(element.length) || 0.001);
  const fittingLength = buildLength * FITTING_FRACTION;
  const barrelLength = buildLength - 2 * fittingLength;
  const tolerance = Math.max(1e-6, buildLength * 1e-4);
  const issues = [];
  if (!validCompressed || !Number.isFinite(extended) || extended < compressed) {
    issues.push({ code: "piston-config", message: "Invalid compressed or extended length" });
  } else if (extended - compressed > barrelLength + tolerance) {
    issues.push({ code: "piston-config", message: "Stroke exceeds the available barrel length" });
  }
  const configurationValid = !issues.length;
  if (Number.isFinite(centerLength) && validCompressed && Number.isFinite(extended)) {
    if (centerLength < compressed - 1e-6) issues.push({ code: "stroke", message: "Below compressed length" });
    else if (centerLength > extended + 1e-6) issues.push({ code: "stroke", message: "Beyond extended length" });
  }
  return {
    fittingLength, barrelLength, barrelRadius: buildLength * 0.027,
    minLength: compressed, maxLength: extended, maxPhysicalStroke: barrelLength,
    configurationValid, valid: !issues.length, issues, code: issues[0]?.code,
    problem: issues.map(issue => issue.message).join(" · ") || undefined,
  };
}
