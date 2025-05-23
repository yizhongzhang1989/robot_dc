// motor_visualization.js
function drawMotorVisualization(motor_id, position, speed) {
  const canvas = document.getElementById(`${motor_id}-vis`);
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const centerX = canvas.width / 2;
  const centerY = canvas.height / 2;
  const radius = 40;

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.beginPath();
  ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
  ctx.strokeStyle = "#ccc";
  ctx.lineWidth = 4;
  ctx.stroke();

  const angle = (position % 360) * (Math.PI / 180);
  const barX = centerX + radius * Math.cos(angle);
  const barY = centerY + radius * Math.sin(angle);

  ctx.beginPath();
  ctx.moveTo(centerX, centerY);
  ctx.lineTo(barX, barY);
  ctx.strokeStyle = speed !== 0 ? "#00f" : "#333";
  ctx.lineWidth = 3;
  ctx.stroke();
}
