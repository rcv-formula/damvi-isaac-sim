"""Freeze the failed contact state for inspection without resetting the scene."""
import live_gui
live_gui.controller.timeline.pause()
live_gui.controller.write_status(force_trace=True)
print('Physics paused for contact-state inspection. No poses or scene files changed.')
