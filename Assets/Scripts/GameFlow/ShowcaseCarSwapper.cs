using System;
using UnityEngine;
using F1.GameData;
using F1.Progression;

namespace F1.GameFlow
{
    /// <summary>
    /// Swaps the garage's displayed showcase car.
    ///
    /// Entries are authored by the garage builder. The prefabs are BAKED display models —
    /// car visuals with the driving stack stripped — never the raw car prefab, which
    /// would drag cameras, physics and input into the garage. All six cars share one body
    /// today, so every entry may point at the same showcase prefab and the tint is what
    /// tells them apart until real liveries exist.
    /// </summary>
    [DisallowMultipleComponent]
    public class ShowcaseCarSwapper : MonoBehaviour
    {
        [Serializable]
        public class CarShowcaseEntry
        {
            public CarDefinition Car;
            public GameObject ShowcasePrefab;
        }

        [SerializeField] private CarShowcaseEntry[] _entries;

        private static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");

        private GameObject _instance;
        private CarDefinition _shownCar;

        /// <summary>The car currently on display, or null.</summary>
        public CarDefinition ShownCar => _shownCar;

        /// <summary>
        /// Shows <paramref name="car"/>: destroys the previous model, instantiates this
        /// entry's prefab under this transform at local identity, and tints it. A locked
        /// car is tinted towards greyscale so the panel's LOCKED state reads on the model
        /// itself.
        /// </summary>
        public void Show(CarDefinition car)
        {
            if (_instance != null) Destroy(_instance);
            _shownCar = car;
            if (car == null) return;

            var entry = FindEntry(car.CarId);
            if (entry?.ShowcasePrefab == null)
            {
                Debug.LogWarning($"[ShowcaseCarSwapper] No showcase prefab authored for '{car.CarId}'.");
                return;
            }

            _instance = Instantiate(entry.ShowcasePrefab, transform);
            _instance.transform.localPosition = Vector3.zero;
            _instance.transform.localRotation = Quaternion.identity;
            _instance.transform.localScale = Vector3.one;

            ApplyTint(_instance, TintFor(car));
        }

        private CarShowcaseEntry FindEntry(string carId)
        {
            if (_entries == null) return null;
            foreach (var entry in _entries)
                if (entry?.Car != null && entry.Car.CarId == carId)
                    return entry;
            return null;
        }

        private static Color TintFor(CarDefinition car)
        {
            var tint = car.DisplayTint;
            var profile = PlayerProfileManager.Current;
            if (profile == null || profile.IsCarAvailable(car.CarId))
                return tint;

            float grey = tint.grayscale * 0.5f;
            return new Color(grey, grey, grey, 1f);
        }

        private static void ApplyTint(GameObject root, Color tint)
        {
            var block = new MaterialPropertyBlock();
            block.SetColor(BaseColorId, tint);
            foreach (var renderer in root.GetComponentsInChildren<Renderer>())
                renderer.SetPropertyBlock(block);
        }
    }
}
