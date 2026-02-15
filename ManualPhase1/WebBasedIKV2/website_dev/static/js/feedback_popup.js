(() => {
  const key = 'feedback_popup_last_shown_at';
  const cooldownMs = 1000 * 60 * 60 * 24;
  const delayMs = 1000 * 75;

  const modalEl = document.getElementById('feedbackModal');
  const submitBtn = document.getElementById('feedbackSubmitBtn');
  const ratingEl = document.getElementById('feedbackRating');
  const commentEl = document.getElementById('feedbackComment');
  const statusEl = document.getElementById('feedbackStatus');

  if (!modalEl || !submitBtn || !ratingEl || !commentEl || !statusEl || typeof bootstrap === 'undefined') {
    return;
  }

  const now = Date.now();
  const lastShown = Number(localStorage.getItem(key) || 0);
  if (now - lastShown < cooldownMs) return;

  const modal = new bootstrap.Modal(modalEl);
  setTimeout(() => {
    modal.show();
    localStorage.setItem(key, String(Date.now()));
  }, delayMs);

  submitBtn.addEventListener('click', async () => {
    submitBtn.disabled = true;
    statusEl.textContent = 'Submitting...';

    try {
      const response = await fetch('/api/feedback', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          rating: Number(ratingEl.value),
          comment: commentEl.value,
          page: window.location.pathname,
        }),
      });

      const payload = await response.json();
      if (!response.ok) {
        statusEl.textContent = payload.message || 'Failed to submit feedback.';
        submitBtn.disabled = false;
        return;
      }

      statusEl.textContent = 'Thank you for your feedback!';
      setTimeout(() => modal.hide(), 700);
    } catch (error) {
      console.error(error);
      statusEl.textContent = 'Network error. Please try again.';
      submitBtn.disabled = false;
    }
  });
})();
