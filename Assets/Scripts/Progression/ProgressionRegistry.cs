using UnityEngine;
using System;
using System.Collections.Generic;
using System.Linq;
using F1.GameData;
using F1.Progression;

namespace F1.Progression
{
    /// <summary>
    /// Progression-aware game data queries.
    /// Separated from GameDataRegistry to avoid circular dependency (GameData -> Progression -> GameData).
    /// </summary>
    public static class ProgressionRegistry
    {
        // --- Filtered lists for UI (require PlayerProfile) ---

        /// <summary>Cars the player permanently owns.</summary>
        public static IReadOnlyList<CarDefinition> GetOwnedCars()
        {
            var profile = PlayerProfileManager.Current;
            if (profile == null) return GameDataRegistry.FreeCars;
            return GameDataRegistry.AllCars.Where(c => profile.OwnsCar(c.CarId)).ToList();
        }

        /// <summary>
        /// Cars the player does <i>not</i> own but can still use: actively rented ones,
        /// plus (when <paramref name="includeRentals"/>) rental variants a rental can be
        /// started on.
        ///
        /// Deliberately disjoint from <see cref="GetOwnedCars"/> and
        /// <see cref="GetLockedCars"/>. The selection screen renders the three lists as
        /// three separate columns, so any overlap renders the same car more than once.
        /// Points-gated cars are excluded because they can be neither rented nor used.
        /// </summary>
        public static IReadOnlyList<CarDefinition> GetAvailableCars(bool includeRentals = true)
        {
            var profile = PlayerProfileManager.Current;
            if (profile == null) return GameDataRegistry.FreeCars;

            return GameDataRegistry.AllCars
                .Where(c => !profile.OwnsCar(c.CarId)
                            && (profile.HasActiveRental(c.CarId)
                                || (includeRentals && c.IsRental)))
                .ToList();
        }

        /// <summary>
        /// Cars gated behind points the player has not earned. Excludes rentals, which
        /// are obtainable without points.
        /// </summary>
        public static IReadOnlyList<CarDefinition> GetLockedCars()
        {
            var profile = PlayerProfileManager.Current;
            if (profile == null) return GameDataRegistry.AllCars;

            return GameDataRegistry.AllCars
                .Where(c => !profile.OwnsCar(c.CarId)
                            && !profile.HasActiveRental(c.CarId)
                            && !c.IsRental)
                .ToList();
        }

        public static IReadOnlyList<TrackDefinition> GetOwnedTracks()
        {
            var profile = PlayerProfileManager.Current;
            if (profile == null) return GameDataRegistry.FreeTracks;
            return GameDataRegistry.AllTracks.Where(t => profile.OwnsTrack(t.TrackId)).ToList();
        }

        public static IReadOnlyList<TrackDefinition> GetAvailableTracks()
        {
            var profile = PlayerProfileManager.Current;
            if (profile == null) return GameDataRegistry.FreeTracks;
            return GameDataRegistry.AllTracks.Where(t => profile.OwnsTrack(t.TrackId) || t.UnlockCostPoints == 0).ToList();
        }

        public static IReadOnlyList<TrackDefinition> GetLockedTracks()
        {
            var profile = PlayerProfileManager.Current;
            if (profile == null) return GameDataRegistry.AllTracks.Where(t => t.UnlockCostPoints > 0).ToList();
            return GameDataRegistry.AllTracks.Where(t => !profile.OwnsTrack(t.TrackId) && t.UnlockCostPoints > 0).ToList();
        }

        // --- Progression helpers ---

        public static bool CanAffordCar(string carId)
        {
            var car = GameDataRegistry.GetCar(carId);
            var profile = PlayerProfileManager.Current;
            return car != null && profile != null && profile.CanAffordCar(car.UnlockCostPoints);
        }

        public static bool TryUnlockCar(string carId)
        {
            var car = GameDataRegistry.GetCar(carId);
            var profile = PlayerProfileManager.Current;
            if (car == null || profile == null) return false;
            if (profile.OwnsCar(carId)) return true; // Already owned
            if (profile.SpendPoints(car.UnlockCostPoints))
            {
                profile.GrantCar(carId);
                return true;
            }
            return false;
        }

        public static bool CanAffordTrack(string trackId)
        {
            var track = GameDataRegistry.GetTrack(trackId);
            var profile = PlayerProfileManager.Current;
            return track != null && profile != null && profile.CanAffordTrack(track.UnlockCostPoints);
        }

        public static bool TryUnlockTrack(string trackId)
        {
            var track = GameDataRegistry.GetTrack(trackId);
            var profile = PlayerProfileManager.Current;
            if (track == null || profile == null) return false;
            if (profile.OwnsTrack(trackId)) return true;
            if (profile.SpendPoints(track.UnlockCostPoints))
            {
                profile.GrantTrack(trackId);
                return true;
            }
            return false;
        }

        public static bool TryRentCar(string carId, int hours = 24)
        {
            var car = GameDataRegistry.GetCar(carId);
            var profile = PlayerProfileManager.Current;
            if (car == null || profile == null) return false;
            if (profile.OwnsCar(carId)) return true;
            if (profile.HasActiveRental(carId)) return true;

            // Check premium currency for rental (future: real money)
            // For now, allow rental if we have the currency
            profile.StartRental(carId, hours);
            return true;
        }
    }
}