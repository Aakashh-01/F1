using UnityEngine;

[DisallowMultipleComponent]
public class F1EngineAudioController : MonoBehaviour
{
    [Header("References")]
    public VehiclePhysicsCoordinator coordinator;
    public VehiclePhysicsProfile physicsProfile;
    public AudioSource engineSource;
    public AudioSource shiftSource;
    public AudioClip engineLoopClip;
    public AudioClip shiftBlipClip;

    [Header("Runtime")]
    public bool generatePlaceholderClip = true;
    [Range(4000, 48000)] public int placeholderSampleRate = 22050;
    [Range(0.1f, 2f)] public float placeholderLoopSeconds = 0.55f;
    [Range(0f, 1f)] public float masterVolume = 1f;

    public float CurrentRpm { get; private set; }
    public int CurrentGear { get; private set; } = 1;
    public float NormalizedLoad { get; private set; }
    public bool IsShifting => _shiftTimer > 0f;

    private float _rpmVelocity;
    private float _loadVelocity;
    private float _shiftTimer;

    private EngineAudioProfile Settings => physicsProfile != null ? physicsProfile.engineAudio : _fallbackProfile;
    private readonly EngineAudioProfile _fallbackProfile = new EngineAudioProfile();

    private void Awake()
    {
        ResolveReferences();
        EnsureAudioSources();
        EnsureLoopClip();
        ApplySourceDefaults();
        RefreshTelemetry(true);
    }

    private void Update()
    {
        RefreshTelemetry(false);
        UpdateAudio();
    }

    public void RefreshTelemetry(bool snap)
    {
        ResolveReferences();

        EngineAudioProfile settings = Settings;
        float speedKmh = coordinator != null ? coordinator.SpeedKmh : 0f;
        float throttle = coordinator != null ? coordinator.ThrottleInput : 0f;
        float brake = coordinator != null ? coordinator.BrakeInput : 0f;
        int previousGear = CurrentGear;
        CurrentGear = CalculateGear(speedKmh, settings);

        if (CurrentGear != previousGear)
        {
            _shiftTimer = Mathf.Max(_shiftTimer, settings.shiftDuration);
            PlayShiftBlip();
        }

        float rpm01 = settings.speedToRpm != null
            ? Mathf.Clamp01(settings.speedToRpm.Evaluate(speedKmh))
            : Mathf.InverseLerp(0f, 330f, speedKmh);
        float throttleLift = Mathf.Lerp(-0.08f, 0.08f, throttle);
        float targetRpm = Mathf.Lerp(settings.idleRpm, settings.maxRpm, Mathf.Clamp01(rpm01 + throttleLift));
        float targetLoad = Mathf.Clamp01(throttle * 0.85f + Mathf.InverseLerp(15f, 260f, speedKmh) * 0.28f - brake * 0.12f);

        if (snap)
        {
            CurrentRpm = targetRpm;
            NormalizedLoad = targetLoad;
        }
        else
        {
            CurrentRpm = Mathf.SmoothDamp(CurrentRpm, targetRpm, ref _rpmVelocity, Mathf.Max(0.01f, settings.rpmSmoothTime));
            NormalizedLoad = Mathf.SmoothDamp(NormalizedLoad, targetLoad, ref _loadVelocity, Mathf.Max(0.01f, settings.loadSmoothTime));
        }

        _shiftTimer = Mathf.Max(0f, _shiftTimer - Time.deltaTime);
    }

    public AudioClip CreatePlaceholderEngineClip()
    {
        int sampleRate = Mathf.Max(4000, placeholderSampleRate);
        int sampleCount = Mathf.Max(256, Mathf.RoundToInt(sampleRate * placeholderLoopSeconds));
        float[] samples = new float[sampleCount];
        const float fundamental = 120f;

        for (int i = 0; i < sampleCount; i++)
        {
            float t = i / (float)sampleRate;
            float tone =
                Mathf.Sin(2f * Mathf.PI * fundamental * t) * 0.46f +
                Mathf.Sin(2f * Mathf.PI * fundamental * 2f * t) * 0.24f +
                Mathf.Sin(2f * Mathf.PI * fundamental * 3f * t) * 0.14f;
            float buzz = Mathf.Sin(2f * Mathf.PI * fundamental * 6f * t) * 0.08f;
            samples[i] = Mathf.Clamp((tone + buzz) * 0.45f, -1f, 1f);
        }

        AudioClip clip = AudioClip.Create("Generated_F1_Engine_Loop", sampleCount, 1, sampleRate, false);
        clip.SetData(samples, 0);
        return clip;
    }

    private void UpdateAudio()
    {
        if (engineSource == null)
            return;

        EngineAudioProfile settings = Settings;
        float rpm01 = Mathf.InverseLerp(settings.idleRpm, settings.maxRpm, CurrentRpm);
        float shiftDip = IsShifting ? settings.shiftPitchDip : 0f;
        engineSource.pitch = Mathf.Max(0.05f, Mathf.Lerp(settings.minPitch, settings.maxPitch, rpm01) - shiftDip);
        engineSource.volume = Mathf.Lerp(settings.idleVolume, settings.maxVolume, NormalizedLoad) * masterVolume;

        if (!engineSource.isPlaying && engineSource.clip != null)
            engineSource.Play();
    }

    private int CalculateGear(float speedKmh, EngineAudioProfile settings)
    {
        int gearCount = Mathf.Max(1, settings.gearCount);
        int gear = 1;
        float[] shifts = settings.upshiftSpeedsKmh;
        if (shifts != null)
        {
            int usable = Mathf.Min(shifts.Length, gearCount - 1);
            for (int i = 0; i < usable; i++)
            {
                if (speedKmh >= shifts[i])
                    gear = i + 2;
            }
        }

        return Mathf.Clamp(gear, 1, gearCount);
    }

    private void ResolveReferences()
    {
        if (coordinator == null)
            coordinator = GetComponent<VehiclePhysicsCoordinator>();

        if (physicsProfile == null && coordinator != null)
            physicsProfile = coordinator.physicsProfile;
    }

    private void EnsureAudioSources()
    {
        if (engineSource == null)
        {
            engineSource = GetComponent<AudioSource>();
            if (engineSource == null)
                engineSource = gameObject.AddComponent<AudioSource>();
        }

        if (shiftSource == null)
        {
            AudioSource[] sources = GetComponents<AudioSource>();
            if (sources.Length > 1)
                shiftSource = sources[1];
        }
    }

    private void EnsureLoopClip()
    {
        if (engineLoopClip == null && generatePlaceholderClip)
            engineLoopClip = CreatePlaceholderEngineClip();

        if (engineSource != null && engineSource.clip == null)
            engineSource.clip = engineLoopClip;
    }

    private void ApplySourceDefaults()
    {
        if (engineSource != null)
        {
            engineSource.loop = true;
            engineSource.playOnAwake = true;
            engineSource.spatialBlend = 0.35f;
            engineSource.dopplerLevel = 0f;
            engineSource.rolloffMode = AudioRolloffMode.Linear;
            engineSource.maxDistance = 70f;
        }

        if (shiftSource != null)
        {
            shiftSource.loop = false;
            shiftSource.playOnAwake = false;
            shiftSource.spatialBlend = 0.35f;
            shiftSource.dopplerLevel = 0f;
        }
    }

    private void PlayShiftBlip()
    {
        if (shiftSource != null && shiftBlipClip != null && isActiveAndEnabled)
            shiftSource.PlayOneShot(shiftBlipClip, 0.35f * masterVolume);
    }
}
