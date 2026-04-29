# Phase 1 Physics Test Checklist

Use `Assets/Scenes/Akash_Scene.unity` for Phase 1 validation.

## Straight-Line Speed
- Start from rest and accelerate with full throttle.
- Confirm throttle ramps smoothly instead of snapping to full force.
- Confirm the car tracks straight with no uncommanded spin.

## High-Speed Corner
- Enter a broad corner above 150 km/h.
- Confirm steering response remains progressive.
- Confirm rear slip produces a light countersteer assist without taking control away from the player.

## Heavy Braking Zone
- Reach high speed, then hold brake input.
- Confirm braking force ramps quickly but does not instantly rotate the car.
- Watch front/rear load and long-G values in the debug panel.

## Camera Speed Perception
- Confirm FOV increases smoothly with speed.
- Confirm Cinemachine follow remains stable and does not jitter at high speed.

## Debug And Profiling
- Toggle the debug panel with Tab.
- Tune one slider at a time and watch for stable telemetry changes.
- Profile FixedUpdate with one car, then duplicate cars later for AI load testing.
