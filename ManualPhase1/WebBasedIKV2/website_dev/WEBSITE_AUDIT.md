# Website Audit (WebBasedIKV2)

## Key problems identified

1. **Session invalidation on every restart**
   - `SECRET_KEY` is generated with `os.urandom(24)` at runtime, so all existing sessions/cookies become invalid whenever the app restarts.
   - Recommended fix: load a stable secret from environment variables.

2. **Security policy allows inline scripts/styles**
   - CSP currently includes `'unsafe-inline'` for scripts and styles, which weakens XSS protection.
   - Recommended fix: migrate inline JavaScript to static files and remove `'unsafe-inline'`.

3. **API endpoints are placeholders**
   - `/api/position` and `/api/orientation` accept JSON but do not validate or persist commands.
   - Recommended fix: validate schema, enforce limits, and connect to the robot-control pipeline.

4. **WebSocket updates are broadcast to all clients**
   - Manual control emits values and server broadcasts to everyone (`broadcast=True`), which can cause one user to overwrite another user's live controls.
   - Recommended fix: add client scoping (rooms/session ownership) and authorization.

5. **Template contains stray TODO text**
   - `MANUAL_CONTROL.html` includes a trailing `#TODO` outside HTML comments/blocks.
   - Recommended fix: remove or convert to proper HTML/Jinja comment.

6. **Missing reverse-tabnabbing protection on external links**
   - External links use `target="_blank"` without `rel="noopener noreferrer"`.
   - Recommended fix: add `rel` attribute to all external links.

7. **Potential mobile UX issue due to forced viewport-height layout**
   - Main wrapper uses `.container vh-100` plus `max-height: calc(100vh - 80px)` and overflow constraints, which may clip content on smaller screens.
   - Recommended fix: allow natural page height with responsive spacing and scrolling.

8. **Accessibility gaps on custom controls**
   - Knob controls rely on pointer events and do not expose keyboard interaction/ARIA semantics.
   - Recommended fix: implement keyboard support (arrow keys), ARIA labels, and focus states.

## Pending / unfinished areas

- Automatic control page is explicitly marked "under development" and currently only displays static trained-object examples.
- Position/orientation API routes are stubs with no operational arm-control integration.
- Error pages are wired, but there is no structured logging/observability pipeline visible in app configuration.

## Recommended change plan (priority order)

### P0 (security + correctness)
- Use environment-backed `SECRET_KEY`.
- Remove inline JS/CSS dependency from CSP.
- Add validation and auth for API/WebSocket control events.
- Add `rel="noopener noreferrer"` to outbound links.

### P1 (core product readiness)
- Implement command queue/acknowledgement flow between UI and robot controller.
- Add user/session ownership for control channels (prevent control collisions).
- Add robust error handling and user-visible error toasts for failed commands.

### P2 (UX enhancements)
- Add a delayed feedback popup (e.g., after 60-90 seconds on page):
  - Use `setTimeout` and `localStorage` to avoid re-showing too frequently.
  - Provide quick rating + optional free-text suggestion.
  - POST to `/api/feedback` with server-side validation and rate-limiting.
- Improve responsive layout for mobile and tablet.
- Add accessibility improvements for knobs/sliders.

## Suggested implementation for the feedback popup

- Trigger logic:
  - Start timer when user lands on key pages (`index`, `manual`, `automatic`).
  - Show Bootstrap modal after delay only if not shown recently.
- Data model:
  - `rating` (1-5), `comment` (0-500 chars), `page`, `timestamp`, optional anonymous session ID.
- Backend endpoint:
  - `POST /api/feedback` with payload validation and storage (SQLite/Postgres).
- Anti-spam:
  - Reuse Flask-Limiter with strict limits for feedback submission.

