function drawMotorVisualization(motor_id, position, speed) {
  const canvas = document.getElementById(`${motor_id}-vis`);
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const centerX = canvas.width / 2;
  const centerY = canvas.height / 2;
  const radius = 40;

  // Clear canvas and draw outer ring
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.beginPath();
  ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
  ctx.strokeStyle = "#ccc";
  ctx.lineWidth = 4;
  ctx.stroke();

  // Calculate angle: map 0–10000 to 0–2π (CCW), with 0 at top (−π/2)
  const norm_pos = position % 10000;
  const angle = ((norm_pos / 10000) * 2 * Math.PI) - Math.PI / 2;

  // Compute indicator endpoint
  const barX = centerX + radius * Math.cos(angle);
  const barY = centerY + radius * Math.sin(angle);

  // Draw the indicator line
  ctx.beginPath();
  ctx.moveTo(centerX, centerY);
  ctx.lineTo(barX, barY);
  ctx.strokeStyle = speed !== 0 ? "#00f" : "#333";
  ctx.lineWidth = 3;
  ctx.stroke();
}
