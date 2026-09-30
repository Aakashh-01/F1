using UnityEngine;

namespace F1.Gameplay
{
    /// <summary>
    /// Places a vehicle on a surface by measuring the vehicle itself, rather than by
    /// lifting it a fixed number of metres.
    ///
    /// Why a constant was wrong
    /// -----------------------
    /// A car's rigidbody origin is NOT its contact patch. On this model the origin sits
    /// 0.922 m above the bottom of the collider, so the body origin has to be about 0.92 m
    /// up for the wheels to rest on the road. Both spawn paths were using far less: the
    /// player spawner lifted 0.6 m and the race grid 0.05 m, which put the collider
    /// 0.32 m and 0.87 m BELOW the surface respectively. PhysX resolves that penetration by
    /// shoving the car out on the next step, which is the bump on the grid and the unsettled
    /// placement on the line.
    ///
    /// A constant also cannot be right for two different vehicles, which is why the player
    /// and the AI cars were wrong by different amounts. Measuring instead makes the same
    /// code correct for both, and correct again if a car's collider is re-exported.
    ///
    /// Measuring the COLLIDER is only right for a car that rests on its collider. These cars
    /// do not: they hang on a raycast-wheel suspension, so the wheels find the road and the
    /// rigidbody origin floats a ride height above it. Reading the collider put the AI car's
    /// origin 0.97 m below where its suspension balances, drove the wheel anchors half a
    /// metre into the tarmac, and the bump stop answered with roughly 3 g of lift — which
    /// was the bump on the grid. A vehicle with wheels is therefore measured with the
    /// suspension's own rest geometry; the collider is the fallback for a body without.
    ///
    /// This only sets a position and clears the velocity. It deliberately does not put the
    /// body to sleep: the start countdown parks these cars, and the parked path damps them,
    /// so a sleeping body would be a car that cannot be woken by its own drivetrain.
    /// </summary>
    public static class VehicleGroundSnap
    {
        /// <summary>
        /// Gap left under a car that is placed by its COLLIDER. Small but positive, so the
        /// body is resting rather than exactly touching — a collider spawned at zero
        /// clearance can still register a contact on its first step.
        ///
        /// Not used on the suspension path. A car on raycast wheels is placed on its
        /// equilibrium with no gap at all, because there a gap is a drop the springs have to
        /// catch rather than a margin; see TryGetSuspensionRestHeight.
        /// </summary>
        public const float DefaultClearance = 0.04f;

        /// <summary>
        /// Moves a car onto the surface directly beneath it and brings it to rest, using
        /// <paramref name="clearance"/> for a car measured by its collider.
        ///
        /// A car with raycast wheels is placed on its own suspension equilibrium and ignores
        /// the clearance; a car without them is placed so its lowest collider point sits
        /// <paramref name="clearance"/> above that surface.
        ///
        /// Leaves the car where it is if it has no rigidbody, no collider, or nothing beneath
        /// it — a snap that cannot find a surface is not a reason to move a car somewhere
        /// arbitrary.
        /// </summary>
        public static bool Snap(GameObject car, float clearance = DefaultClearance)
        {
            if (car == null) return false;

            var body = car.GetComponentInChildren<Rigidbody>();
            if (body == null) return false;

            float lowest = LowestColliderOffset(body);
            if (float.IsInfinity(lowest) || float.IsNaN(lowest)) return false;

            // RaycastAll, not Raycast, with the car's own colliders filtered out. The probe
            // has to start above the car to be useful on a slope, so the FIRST thing it can
            // hit is the car's own roof; taking that placed the car a car's own height
            // further up, which measured as a car floating 1.6 m over the road.
            //
            // And of the remaining hits, only accept a surface BELOW the body origin. The
            // circuits carry gantries and bridges, and taking the highest hit picked one of
            // those instead of the road — both the player car and the AI car came out at
            // exactly the same wrong height, which is the signature of the probe finding
            // shared overhead geometry rather than the ground. A car's origin sits roughly
            // 0.9 m above its contact patch, so the road it belongs on is always below the
            // origin and anything above it is structure, not ground.
            var hits = Physics.RaycastAll(body.position + Vector3.up * 6f, Vector3.down,
                                          40f, ~0, QueryTriggerInteraction.Ignore);
            float surfaceY = float.NegativeInfinity;
            foreach (var h in hits)
            {
                if (h.collider == null) continue;
                var t = h.collider.transform;
                if (t == car.transform || t.IsChildOf(car.transform)) continue;
                if (h.point.y >= body.position.y) continue;
                if (h.point.y > surfaceY) surfaceY = h.point.y;
            }
            if (float.IsNegativeInfinity(surfaceY)) return false;

            // Which reference decides this car's height. A vehicle that hangs on raycast
            // wheels decides it with its suspension, and measuring its collider instead
            // fights that suspension into a launch. See TryGetSuspensionRestHeight.
            float targetOriginY = TryGetSuspensionRestHeight(car, body, surfaceY,
                                                               out float suspensionY)
                ? suspensionY
                : surfaceY + clearance - lowest;

            float lift = targetOriginY - body.position.y;

            // A vehicle is never more than a couple of metres from the surface it is being
            // placed on. Anything beyond that means the probe found the wrong thing, and
            // teleporting a car is far worse than leaving it where the caller put it.
            if (Mathf.Abs(lift) > MaxLift) return false;

            // Both, together. Writing only the body is what made this look broken: the snap
            // measured correctly, lifted the car correctly, and was then silently undone.
            //
            // Physics.autoSyncTransforms is False in this project, so any later
            // Physics.SyncTransforms() pushes the *transform's* pose down into the body. The
            // grid manager calls exactly that at the end of SpawnGrid, and a transform left
            // at the un-snapped grid height therefore won: the car was put back 0.87 m into
            // the road and PhysX ejected it on the next step. That ejection is the bump.
            //
            // Writing one without the other cannot hold. PhysX caches the pose a body was
            // created at and reverts a transform-only write on the next step, so the body
            // write is needed; the transform write is needed so the two agree and no sync
            // can find a stale height to restore. Written together, the corrected height
            // survives whichever one the engine reads.
            //
            // The car root carries the rigidbody on both the player and the AI prefab, so
            // this is the same transform PositionCarOnGrid wrote the grid pose to.
            Vector3 corrected = body.position + new Vector3(0f, lift, 0f);
            body.position = corrected;
            car.transform.position = corrected;

            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            body.WakeUp();
            return true;
        }

        /// <summary>
        /// Largest vertical correction the snap will make. Beyond this it assumes the probe
        /// found the wrong surface and declines to move the car at all.
        /// </summary>
        public const float MaxLift = 2.5f;

        /// <summary>
        /// The body height at which this car's own suspension is in equilibrium over
        /// <paramref name="surfaceY"/>, or false if it has no raycast wheels to ask.
        ///
        /// Why the suspension, and not the collider
        /// -----------------------------------------
        /// <see cref="RaycastWheel.Simulate"/> builds its lift as an UNCONDITIONAL
        /// <c>mass * |gravity| / 4</c> per grounded wheel, plus a spring term. Four grounded
        /// wheels therefore cancel the car's entire weight at any ride height, and the body
        /// settles wherever the spring term crosses zero — which is
        /// <c>SuspensionTravel == restLengthRatio</c>, i.e. exactly
        /// <see cref="RaycastWheel.GetRestAnchorDistance"/> above the contact patch.
        ///
        /// That equilibrium is a property of the vehicle, and the suspension enforces it
        /// every physics step. Placing the body anywhere else is not a position, it is a
        /// stored instruction: the spring answers the difference. Placing it a little low is
        /// a gentle settle; placing it 0.97 m low — which is what reading the collider did to
        /// the AI car, whose collider is a small box that by design floats a metre above the
        /// road — buries the wheel anchors half a metre into the tarmac. The bump stop then
        /// saturates, <c>maxForcePerWheel</c> clamps at 3x static weight, and four wheels
        /// answer with about 3 g of lift. That launch is the bump on the grid, and it looked
        /// like a spawn problem because it happened during the spawn.
        ///
        /// So this asks the same question the physics asks, using the same numbers, and the
        /// car is placed where the suspension already wanted it. Nothing has to settle.
        ///
        /// The equilibrium exactly, with no clearance, and that is deliberate where the
        /// collider fallback uses one. A gap here is not a safety margin, it is a drop: at
        /// rest each wheel is already carrying a quarter of the car's weight, so lifting the
        /// body 4 cm leaves the springs carrying almost nothing and the car free-falls the
        /// 4 cm back onto them. The collider branch can afford a gap because a collider
        /// carries nothing. The wheel branch is placed on the equilibrium so the spawn and
        /// the suspension agree and the car is still on its first physics step.
        ///
        /// The maximum over all wheels, not the average or the first: a car that is level has
        /// all four equal, and one that is not — a kerb, a compression, a bad spawn — must
        /// clear its tallest requirement or that wheel starts over-compressed.
        /// </summary>
        private static bool TryGetSuspensionRestHeight(GameObject car, Rigidbody body,
                                                        float surfaceY, out float originY)
        {
            originY = 0f;

            var wheels = car.GetComponentsInChildren<RaycastWheel>(true);
            if (wheels == null || wheels.Length == 0)
                return false;

            float best = float.NegativeInfinity;
            bool any = false;

            foreach (var wheel in wheels)
            {
                if (wheel == null) continue;

                float rest = wheel.GetRestAnchorDistance();
                if (float.IsNaN(rest) || float.IsInfinity(rest)) continue;

                // How far this wheel's anchor currently hangs below the rigidbody origin.
                // Sampled from the live transform, so it survives a rotated or scaled car.
                float anchorBelowOrigin = body.position.y - wheel.transform.position.y;
                float required = surfaceY + rest + anchorBelowOrigin;

                if (required > best) best = required;
                any = true;
            }

            if (!any || float.IsNaN(best) || float.IsInfinity(best))
                return false;

            originY = best;
            return true;
        }

        /// <summary>
        /// How far the lowest enabled collider sits BELOW the rigidbody origin, in metres.
        /// Negative for a car whose origin is above its contact patch, which is the normal
        /// case. Positive if a vehicle is modelled origin-at-the-ground.
        /// </summary>
        public static float LowestColliderOffset(Rigidbody body)
        {
            float lowest = float.PositiveInfinity;
            foreach (var col in body.GetComponentsInChildren<Collider>())
            {
                if (!col.enabled || col.isTrigger) continue;
                // bounds are world-space, so subtract the body's world position to get an
                // offset that survives the car being rotated.
                lowest = Mathf.Min(lowest, col.bounds.min.y - body.position.y);
            }
            return float.IsPositiveInfinity(lowest) ? float.NegativeInfinity : lowest;
        }
    }
}
