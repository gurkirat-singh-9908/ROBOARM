(() => {
  const channel = 'manual-control';
  const socket = io();

  socket.on('connect', () => {
    socket.emit('join_channel', { channel });
  });

  function emitValue(param, value) {
    socket.emit('update_value', {
      channel,
      param,
      value,
    });
  }

  document.querySelectorAll('.form-range').forEach((slider) => {
    slider.addEventListener('input', function onInput() {
      const suffix = this.id.split('_')[1];
      emitValue(this.id, this.value);
      document.getElementById(`value_${suffix}`).textContent = this.value;
      document.getElementById(`display_${suffix}`).textContent = this.value;
    });
  });

  socket.on('value_updated', (data) => {
    if (!data || data.channel !== channel) return;
    console.log('Value updated:', data);
  });

  socket.on('control_error', (data) => {
    console.warn(data?.message || 'Control channel error');
  });

  function applyPose(pose) {
    Object.keys(pose).forEach((param) => {
      const value = pose[param];
      emitValue(param, value);
      const displayId = param.replace('slider_', '');
      document.getElementById(`value_${displayId}`).textContent = value;
      document.getElementById(`display_${displayId}`).textContent = value;

      if (param.startsWith('slider_')) {
        document.getElementById(param).value = value;
      }

      if (['roll', 'pitch', 'yaw'].includes(param)) {
        const knob = document.querySelector(`.knob-container[data-param="${param}"]`);
        if (knob) {
          knob.currentAngle = value;
          knob.querySelector('.knob-indicator-container').style.transform = `rotate(${value}deg)`;
        }
      }
    });
  }

  document.getElementById('home_position').addEventListener('click', () => {
    applyPose({
      slider_x: 0,
      slider_y: 0,
      slider_z: 0.5,
      roll: 90,
      pitch: 90,
      yaw: 90,
      slider_gripper: 0,
    });
  });

  document.getElementById('rest_position').addEventListener('click', () => {
    applyPose({
      slider_x: 0,
      slider_y: 0,
      slider_z: -0.5,
      roll: 0,
      pitch: 0,
      yaw: 0,
      slider_gripper: 100,
    });
  });

  document.querySelectorAll('.knob-container').forEach((knob) => {
    let dragging = false;
    let lastAngle = 0;

    function getClientCoords(event) {
      if (event.touches && event.touches.length > 0) {
        return {
          clientX: event.touches[0].clientX,
          clientY: event.touches[0].clientY,
        };
      }
      return { clientX: event.clientX, clientY: event.clientY };
    }

    function updateKnob(event) {
      const coords = getClientCoords(event);
      const rect = knob.getBoundingClientRect();
      const centerX = rect.left + rect.width / 2;
      const centerY = rect.top + rect.height / 2;
      const dx = coords.clientX - centerX;
      const dy = coords.clientY - centerY;

      let target = (Math.atan2(dy, dx) * 180) / Math.PI + 90;
      target = Math.max(0, Math.min(180, target));

      if (typeof knob.currentAngle === 'undefined') {
        knob.currentAngle = target;
      }

      const factor = 0.1;
      const newAngle = knob.currentAngle + factor * (target - knob.currentAngle);
      knob.currentAngle = newAngle;
      lastAngle = Math.round(newAngle);

      const param = knob.dataset.param;
      document.getElementById(`value_${param}`).textContent = lastAngle;
      document.getElementById(`display_${param}`).textContent = lastAngle;
      emitValue(param, lastAngle);
      knob.querySelector('.knob-indicator-container').style.transform = `rotate(${newAngle}deg)`;
    }

    knob.addEventListener('mousedown', (event) => {
      dragging = true;
      updateKnob(event);
    });
    knob.addEventListener('touchstart', (event) => {
      dragging = true;
      updateKnob(event);
    });

    document.addEventListener('mousemove', (event) => {
      if (dragging) updateKnob(event);
    });
    document.addEventListener('touchmove', (event) => {
      if (dragging) updateKnob(event);
    });

    document.addEventListener('mouseup', () => {
      if (!dragging) return;
      dragging = false;
      emitValue(knob.dataset.param, lastAngle);
    });
    document.addEventListener('touchend', () => {
      if (!dragging) return;
      dragging = false;
      emitValue(knob.dataset.param, lastAngle);
    });
  });
})();
