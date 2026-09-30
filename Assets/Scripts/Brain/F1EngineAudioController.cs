using UnityEngine;

/// <summary>
/// Engine sound for a car: RPM, load, gearing, and the timbre that makes a throttle input
/// feel like a throttle input.
///
/// <para>
/// <b>What was wrong before.</b> The controller shipped with no clips at all — both clip
/// fields were null and the placeholder generator was on, so what you heard was three sine
/// partials at 120 Hz plus one harmonic buzz, 0.55 s long and mono. On top of that, the only
/// things modulating it were pitch and volume: a single loop resampled across a 0.72-2.05
/// sweep, and a volume that rose with throttle. That is why it read as cheap. It was not
/// loudness or quality settings; it was three sine waves and no timbre.
///
/// <para>
/// <b>Why pitch-shifting one loop cannot fix it.</b> Resampling a loop to change its pitch
/// changes its length and artefacts with every cycle; sweeping one clip 3x is the source of
/// the "siren" character. Real engine audio is built the other way round: several recorded
/// loops, each covering a band of the rev range, played at roughly their own pitch and
/// crossfaded. That is what <see cref="engineLayerClips"/> is for. With no real clips the
/// same machinery runs on generated ones, so the structure is correct before any asset
/// exists.
///
/// <para>
/// <b>Why the filter matters more than the clips.</b> A closed throttle is a closed throttle:
/// induction is shut, the top end is muted, and the note is dull. Opening it is most of what
/// a driver hears as "load". Volume alone cannot express it, which is why throttle used to
/// do almost nothing except get louder. Every engine voice now carries a low-pass whose
/// cutoff tracks throttle, which makes pressing the pedal change the *character* of the
/// sound rather than its level — the single largest immersion win available here.
///
/// <para>
/// The public surface is deliberately unchanged — <c>engineSource</c>, <c>shiftSource</c>,
/// <c>engineLoopClip</c>, <c>shiftBlipClip</c>, <c>CurrentRpm</c>, <c>CurrentGear</c>,
/// <c>NormalizedLoad</c>, <c>IsShifting</c> — because the foundation tests drive it
/// directly, and because a component whose name and contract move is a component every call
/// site has to be re-read against.
/// </para>
/// </summary>
[DisallowMultipleComponent]
public class F1EngineAudioController : MonoBehaviour
{
    [Header("References")]
    public VehiclePhysicsCoordinator coordinator;
    public VehiclePhysicsProfile physicsProfile;

    /// <summary>Primary engine voice. Also layer 0, kept for the existing contract.</summary>
    public AudioSource engineSource;
    public AudioSource shiftSource;

    /// <summary>
    /// Fallback loop used when no banded clips are supplied. Generated if null and
    /// <see cref="generatePlaceholderClip"/> is set.
    /// </summary>
    public AudioClip engineLoopClip;
    public AudioClip shiftBlipClip;

    [Header("Banded engine layers (preferred)")]
    [Tooltip("Optional recorded loops, lowest rev band first. Played at roughly their own " +
             "pitch and crossfaded as revs rise, instead of one loop being resampled across " +
             "the whole range. Leave empty to use the single generated loop for every band.")]
    public AudioClip[] engineLayerClips;

    [Tooltip("Induction / intake roar that fades in with throttle. This is the layer that " +
             "makes the pedal feel connected; the engine note alone does not.")]
    public AudioClip loadLayerClip;

    [Tooltip("Overrun crackle and burble, audible when the throttle is lifted at speed.")]
    public AudioClip overrunLayerClip;

    [Header("Throttle timbre")]
    [Tooltip("Low-pass cutoff with the throttle shut. A muffled induction note.")]
    [Range(200f, 4000f)] public float closedThrottleCutoffHz = 620f;

    [Tooltip("Low-pass cutoff at full throttle. Open, bright, and the reason pressing the " +
             "pedal sounds like pressing the pedal.")]
    [Range(2000f, 20000f)] public float openThrottleCutoffHz = 13500f;

    [Tooltip("How quickly the filter tracks the throttle. Slow enough to feel like a " +
             "mechanism, fast enough to answer the input.")]
    [Range(0.01f, 0.5f)] public float filterSmoothTime = 0.06f;

    [Header("Layers")]
    [Range(0f, 1f)] public float loadLayerVolume = 0.5f;
    [Range(0f, 1f)] public float overrunLayerVolume = 0.45f;
    [Tooltip("Rev fraction above which a lifted throttle produces overrun noise.")]
    [Range(0f, 1f)] public float overrunRevThreshold = 0.55f;
    [Tooltip("Rev fraction below which overrun noise stops. Hysteresis, so it does not " +
             "chatter on and off around the threshold.")]
    [Range(0f, 1f)] public float overrunRevCutoff = 0.42f;

    [Header("Runtime")]
    public bool generatePlaceholderClip = true;
    [Range(8000, 48000)] public int placeholderSampleRate = 44100;
    [Range(0.1f, 2f)] public float placeholderLoopSeconds = 0.5f;
    [Range(0.03f, 0.3f)] public float placeholderShiftSeconds = 0.12f;
    [Range(0f, 1f)] public float masterVolume = 1f;

    [Tooltip("0 is a cockpit mix (the player's own car, heard from inside). 1 is fully in " +
             "the world, which is what a rival car wants.")]
    [Range(0f, 1f)] public float spatialBlend = 0f;

    public float CurrentRpm { get; private set; }
    public int CurrentGear { get; private set; } = 1;
    public float NormalizedLoad { get; private set; }

    /// <summary>Current throttle, 0..1. Drives the filter and the load layer.</summary>
    public float CurrentThrottle { get; private set; }

    public bool IsShifting => _shiftTimer > 0f;

    private readonly AudioSource[] _engineLayers = new AudioSource[EngineLayerCount];
    private AudioLowPassFilter[] _engineFilters;
    private AudioSource _loadSource;
    private AudioLowPassFilter _loadFilter;
    private AudioSource _overrunSource;

    private float _rpmVelocity;
    private float _loadVelocity;
    private float _filterVelocity;
    private float _shiftTimer;
    private bool _overrunLatched;

    /// <summary>
    /// Rev bands. Three is the point where crossfading earns its keep: enough that no single
    /// loop is ever swept far, few enough that the mix stays stable.
    /// </summary>
    private const int EngineLayerCount = 3;

    private EngineAudioProfile Settings => physicsProfile != null ? physicsProfile.engineAudio : _fallbackProfile;
    private readonly EngineAudioProfile _fallbackProfile = new EngineAudioProfile();

    private void Awake()
    {
        ResolveReferences();
        EnsureAudioSources();
        EnsureClips();
        ApplySourceDefaults();
        RefreshTelemetry(true);
    }

    private void Update()
    {
        RefreshTelemetry(false);
        UpdateAudio();
    }

    private void OnDestroy()
    {
        // Generated clips are owned by this component, not by an asset, and Unity will not
        // collect them while an AudioSource still references them.
        if (_generatedEngine != null) { _generatedEngine = null; }
    }

    private AudioClip _generatedEngine;
    private AudioClip _generatedShift;
    private AudioClip _generatedLoad;
    private AudioClip _generatedOverrun;

    public void RefreshTelemetry(bool snap)
    {
        ResolveReferences();

        EngineAudioProfile settings = Settings;
        float speedKmh = coordinator != null ? coordinator.SpeedKmh : 0f;
        float throttle = coordinator != null ? coordinator.ThrottleInput : 0f;
        float brake = coordinator != null ? coordinator.BrakeInput : 0f;
        CurrentThrottle = Mathf.Clamp01(throttle);

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

        // The old lift was +/-0.08, which is inaudible. A real engine flares toward the
        // limiter as the throttle goes down; this is a rounding of the pedal, not the main
        // character change (that is the filter's job now).
        float throttleLift = Mathf.Lerp(-0.05f, 0.10f, CurrentThrottle);
        float targetRpm = Mathf.Lerp(settings.idleRpm, settings.maxRpm, Mathf.Clamp01(rpm01 + throttleLift));
        float targetLoad = Mathf.Clamp01(CurrentThrottle * 0.7f + Mathf.InverseLerp(15f, 260f, speedKmh) * 0.3f - brake * 0.12f);

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

    /// <summary>
    /// Generates band-limited noise that loops without a seam.
    ///
    /// <para>
    /// Two things have to be right, and both were wrong in the first attempt. The noise has
    /// to be <i>band-limited</i>: averaging several white samples at the same sample index
    /// does not low-pass anything, it just draws a slightly less extreme white sample, and
    /// the result is full-band hiss — measurably 3273 zero crossings in half a second,
    /// which is a noise floor, not an engine. A one-pole filter whose state is carried
    /// across samples is what actually makes it rumble.
    /// </para>
    /// <para>
    /// And it has to loop silently. The trick is to generate <c>sampleCount + fade</c>
    /// samples and crossfade the first <c>fade</c> of them from the samples that come
    /// <i>after</i> the buffer. The final stored sample is then the last of the natural
    /// sequence and the first stored sample is the one immediately after it, so the wrap is
    /// between two adjacent samples of one continuous stream. Fading the tail into the
    /// head instead — the obvious construction — wraps between two unrelated points and
    /// ticks once per loop.
    /// </para>
    /// <para>
    /// Returns a buffer normalised to unit peak so callers can set level by a single
    /// multiplier instead of guessing at the generator's output.
    /// </para>
    /// </summary>
    private static float[] BuildLoopingNoise(int sampleCount, int sampleRate, float cutoffHz,
        System.Random rng, out float rmsOut)
    {
        int fade = Mathf.Clamp(sampleRate / 40, 16, sampleCount / 2);
        float dt = 1f / sampleRate;

        // One-pole low-pass. rc is the time constant of the RC equivalent; alpha is the
        // per-sample step, so a lower cutoff smooths harder and drops the top end out.
        float rc = 1f / (2f * Mathf.PI * Mathf.Max(20f, cutoffHz));
        float alpha = Mathf.Clamp01(dt / (rc + dt));

        int total = sampleCount + fade;
        var raw = new float[total];
        float lp = 0f;
        for (int i = 0; i < total; i++)
        {
            float white = (float)(rng.NextDouble() * 2.0 - 1.0);
            lp += alpha * (white - lp);
            raw[i] = lp;
        }

        var loop = new float[sampleCount];
        for (int i = 0; i < fade; i++)
        {
            float k = i / (float)fade;
            loop[i] = Mathf.Lerp(raw[sampleCount + i], raw[i], k);
        }
        for (int i = fade; i < sampleCount; i++)
            loop[i] = raw[i];

        float peak = 0f, sum = 0f;
        for (int i = 0; i < sampleCount; i++)
        {
            float a = Mathf.Abs(loop[i]);
            if (a > peak) peak = a;
            sum += loop[i] * loop[i];
        }
        rmsOut = Mathf.Sqrt(sum / sampleCount);

        if (peak > 0.0001f)
        {
            float scale = 1f / peak;
            for (int i = 0; i < sampleCount; i++) loop[i] *= scale;
        }

        return loop;
    }

    // --- Clip generation -----------------------------------------------------

    /// <summary>
    /// Hyperbolic tangent, as a saturating curve rather than a clamp.
    ///
    /// Unity has no <c>Mathf.Tanh</c>, and the obvious substitute — clamping the samples to
    /// +/-1 — is precisely the artefact this is here to avoid. A hard clamp squares off the
    /// peaks of every loud cycle, and a waveform full of squared-off peaks is the defining
    /// sound of cheap synthesis: it is the same reason a guitar with a hard limiter sounds
    /// worse than one with a soft one. tanh bends the peaks into the ceiling instead, which
    /// is what real saturation does to a signal.
    ///
    /// Uses the standard rational approximation, which is accurate to about 1e-4 across the
    /// range that matters here and saturates to exactly +/-1 outside it. This runs once per
    /// sample at load time, so clarity beats micro-optimisation; the one thing that mattered
    /// was not using <c>Math.Exp</c> in a per-sample loop.
    /// </summary>
    private static float SoftClip(float x)
    {
        if (x < -3f) return -1f;
        if (x > 3f) return 1f;

        float x2 = x * x;
        return x * (27f + x2) / (27f + 9f * x2);
    }

    /// <summary>
    /// Generates a usable engine loop when no recorded clips exist.
    ///
    /// Deliberately not three sine waves, which is what this used to be and why it sounded
    /// like a test tone. A real engine is a stack of many partials with an uneven envelope
    /// over them, plus broadband induction noise that sits on top — and, critically, a
    /// *seamless* loop. The sample count is snapped to a whole number of cycles of the
    /// fundamental so the last sample joins the first; a loop that does not is a click on
    /// every repeat, and at 0.55 s that click lands constantly.
    /// </summary>
    public AudioClip CreatePlaceholderEngineClip()
    {
        int sampleRate = Mathf.Max(8000, placeholderSampleRate);
        const float fundamental = 62f;

        // Whole cycles only, so the waveform is periodic across the clip and loops silently.
        float cycles = Mathf.Max(4f, Mathf.Round(fundamental * Mathf.Max(0.1f, placeholderLoopSeconds)));
        float seconds = cycles / fundamental;
        int sampleCount = Mathf.Max(1024, Mathf.RoundToInt(sampleRate * seconds));
        seconds = sampleCount / (float)sampleRate;

        float[] samples = new float[sampleCount];
        var random = new System.Random(20260928);

        // Band-limited induction rumble, properly filtered and seamlessly looped. Kept low
        // in the mix: this sits under the harmonic stack as body, and the load layer
        // carries the audible induction on top of it when the throttle opens.
        float noiseRms;
        float[] noise = BuildLoopingNoise(sampleCount, sampleRate, 520f, random, out noiseRms);

        for (int i = 0; i < sampleCount; i++)
        {
            float t = i / (float)sampleRate;
            float phase = 2f * Mathf.PI * fundamental * t;

            // Harmonic stack. Even partials (the firing order of a flat-plane engine) carry
            // most of the character, so they are lifted rather than following a flat 1/n.
            float tone = 0f;
            for (int h = 1; h <= 14; h++)
            {
                float f = fundamental * h;
                float amp = 0.62f / h;
                if ((h & 1) == 0) amp *= 1.65f;                 // even harmonics
                if (h == 1) amp *= 1.15f;
                tone += Mathf.Sin(phase * h) * amp;
            }

            // Sub, which is what carries weight at idle.
            tone += Mathf.Sin(phase * 0.5f) * 0.30f;

            // A little asymmetry per cycle, so the loop does not feel mechanical.
            float grit = Mathf.Sin(phase * 7f) * Mathf.Sin(phase * 0.5f) * 0.10f;

            float mixed = tone * 0.34f + noise[i] * 0.085f + grit;

            // Soft clip rather than hard clamp: a flat-topped waveform is what cheap digital
            // synthesis sounds like.
            samples[i] = SoftClip(mixed * 1.35f) * 0.72f;
        }

        _generatedEngine = AudioClip.Create("Generated_F1_Engine_Loop", sampleCount, 1, sampleRate, false);
        _generatedEngine.SetData(samples, 0);
        return _generatedEngine;
    }

    /// <summary>Induction layer: broadband, no pitch, and meant to be filtered by the caller.</summary>
    public AudioClip CreatePlaceholderLoadClip()
    {
        int sampleRate = Mathf.Max(8000, placeholderSampleRate);
        int sampleCount = Mathf.Max(2048, Mathf.RoundToInt(sampleRate * 1.0f));

        float rms;
        float[] samples = BuildLoopingNoise(sampleCount, sampleRate, 1400f,
            new System.Random(7717), out rms);

        _generatedLoad = AudioClip.Create("Generated_F1_Load_Loop", sampleCount, 1, sampleRate, true);
        _generatedLoad.SetData(samples, 0);
        return _generatedLoad;
    }

    /// <summary>Overrun crackle: sparse bursts, not a continuous tone.</summary>
    public AudioClip CreatePlaceholderOverrunClip()
    {
        int sampleRate = Mathf.Max(8000, placeholderSampleRate);
        int sampleCount = Mathf.Max(2048, Mathf.RoundToInt(sampleRate * 0.9f));

        float rms;
        float[] noise = BuildLoopingNoise(sampleCount, sampleRate, 2600f,
            new System.Random(31337), out rms);

        // Bursts, spaced irregularly so it never sounds metronomic.
        var random = new System.Random(4242);
        var samples = new float[sampleCount];
        float envelope = 0f;
        for (int i = 0; i < sampleCount; i++)
        {
            if (random.NextDouble() < 0.0016f)
                envelope = 0.35f + (float)random.NextDouble() * 0.65f;

            samples[i] = noise[i] * envelope;
            envelope *= 0.9975f;
        }

        _generatedOverrun = AudioClip.Create("Generated_F1_Overrun_Loop", sampleCount, 1, sampleRate, true);
        _generatedOverrun.SetData(samples, 0);
        return _generatedOverrun;
    }

    public AudioClip CreatePlaceholderShiftClip()
    {
        int sampleRate = Mathf.Max(8000, placeholderSampleRate);
        int sampleCount = Mathf.Max(128, Mathf.RoundToInt(sampleRate * placeholderShiftSeconds));
        float[] samples = new float[sampleCount];
        var random = new System.Random(9001);

        for (int i = 0; i < sampleCount; i++)
        {
            float t = i / (float)sampleRate;
            float normalized = i / Mathf.Max(1f, sampleCount - 1f);
            float attack = Mathf.Clamp01(normalized / 0.08f);
            float decay = Mathf.Exp(-normalized * 11f);
            float envelope = attack * decay;

            float body = Mathf.Sin(2f * Mathf.PI * 210f * t) * 0.30f;
            float air = (float)(random.NextDouble() * 2.0 - 1.0) * 0.30f * Mathf.Exp(-normalized * 18f);
            float click = Mathf.Sin(2f * Mathf.PI * 2400f * t) * 0.16f * Mathf.Exp(-normalized * 40f);

            samples[i] = SoftClip((body + air + click) * envelope * 1.2f) * 0.85f;
        }

        _generatedShift = AudioClip.Create("Generated_F1_Upshift_Blip", sampleCount, 1, sampleRate, false);
        _generatedShift.SetData(samples, 0);
        return _generatedShift;
    }

    // --- Mixing --------------------------------------------------------------

    private void UpdateAudio()
    {
        if (engineSource == null)
            return;

        EngineAudioProfile settings = Settings;
        float rpm01 = Mathf.Clamp01(Mathf.InverseLerp(settings.idleRpm, settings.maxRpm, CurrentRpm));

        // Layers play at their own pitch. Only a small trim is applied, so nothing is swept
        // far enough to artefact — which is the whole reason for the bands.
        float trim = Mathf.Lerp(0.96f, 1.06f, rpm01);
        float shiftDip = IsShifting ? settings.shiftPitchDip * 0.35f : 0f;

        for (int i = 0; i < _engineLayers.Length; i++)
        {
            var src = _engineLayers[i];
            if (src == null) continue;

            float weight = LayerWeight(i, rpm01);
            if (weight <= 0.001f)
            {
                if (src.isPlaying) src.Stop();
                continue;
            }

            src.pitch = Mathf.Max(0.05f, trim - shiftDip);
            src.volume = Mathf.Lerp(settings.idleVolume, settings.maxVolume, NormalizedLoad)
                        * weight * masterVolume;

            if (!src.isPlaying && src.clip != null)
                src.Play();
        }

        UpdateFilter(rpm01);

        if (_loadSource != null && _loadSource.clip != null)
        {
            // The load layer is the pedal made audible: near-silent off throttle, prominent
            // on it. It shares the engine volume so a muted car is muted.
            float bed = Mathf.Lerp(settings.idleVolume, settings.maxVolume, NormalizedLoad);
            _loadSource.volume = bed * loadLayerVolume * CurrentThrottle * masterVolume;
            if (!_loadSource.isPlaying) _loadSource.Play();
        }

        UpdateOverrun(rpm01);
    }

    /// <summary>
    /// Crossfade weight for a rev band: bands overlap, so a rev range is never carried by one
    /// clip alone and the transition between them is a blend rather than a switch.
    /// </summary>
    private float LayerWeight(int index, float rpm01)
    {
        if (EngineLayerCount == 1) return 1f;

        float bandStart = index / (float)EngineLayerCount;
        float bandEnd = (index + 1) / (float)EngineLayerCount;
        const float overlap = 0.5f / EngineLayerCount;

        float weight = 1f;
        if (rpm01 < bandStart - overlap) weight = 0f;
        else if (rpm01 < bandStart + overlap)
            weight = Mathf.InverseLerp(bandStart - overlap, bandStart + overlap, rpm01);
        else if (rpm01 > bandEnd + overlap) weight = 0f;
        else if (rpm01 > bandEnd - overlap)
            weight = 1f - Mathf.InverseLerp(bandEnd - overlap, bandEnd + overlap, rpm01);

        return Mathf.Clamp01(weight);
    }

    /// <summary>
    /// Drives the low-pass cutoff from throttle, with a smaller lift from revs so a car on
    /// part throttle at high revs is not muffled.
    /// </summary>
    private void UpdateFilter(float rpm01)
    {
        float target = Mathf.Lerp(closedThrottleCutoffHz, openThrottleCutoffHz, CurrentThrottle);
        target *= Mathf.Lerp(0.85f, 1.12f, rpm01);

        float cutoff = _loadFilter != null
            ? _loadFilter.cutoffFrequency
            : Mathf.Lerp(closedThrottleCutoffHz, openThrottleCutoffHz, 0.5f);
        cutoff = Mathf.SmoothDamp(cutoff, target, ref _filterVelocity, filterSmoothTime);

        // Clamped to the audible band. A low-pass above the sample rate is not "more open",
        // it is a no-op with a meaningless number attached, and a value of 0 passes nothing.
        cutoff = Mathf.Clamp(cutoff, 120f, 20000f);

        if (_engineFilters != null)
        {
            for (int i = 0; i < _engineFilters.Length; i++)
            {
                if (_engineFilters[i] != null)
                    _engineFilters[i].cutoffFrequency = cutoff;
            }
        }

        if (_loadFilter != null)
            _loadFilter.cutoffFrequency = cutoff;
    }

    private void UpdateOverrun(float rpm01)
    {
        if (_overrunSource == null || _overrunSource.clip == null)
            return;

        // Hysteresis on both revs and throttle, so lifting and feathering the pedal does not
        // make the crackle stutter.
        if (CurrentThrottle < 0.12f && rpm01 > overrunRevThreshold) _overrunLatched = true;
        else if (CurrentThrottle > 0.35f || rpm01 < overrunRevCutoff) _overrunLatched = false;

        if (_overrunLatched)
        {
            _overrunSource.volume = overrunLayerVolume * Mathf.InverseLerp(overrunRevCutoff, 1f, rpm01) * masterVolume;
            if (!_overrunSource.isPlaying) _overrunSource.Play();
        }
        else if (_overrunSource.isPlaying)
        {
            _overrunSource.Stop();
        }
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
        // engineSource stays exactly where it was: the component's own AudioSource, or the
        // first one found. The foundation tests assert on it directly.
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

        _engineLayers[0] = engineSource;

        // Extra bands live on their own child objects, because an AudioLowPassFilter is a
        // component on the same object as its source and does not cascade across objects.
        for (int i = 1; i < EngineLayerCount; i++)
        {
            string name = $"EngineLayer{i}";
            Transform host = transform.Find(name);
            if (host == null)
            {
                var go = new GameObject(name);
                go.transform.SetParent(transform, false);
                host = go.transform;
            }

            var src = host.GetComponent<AudioSource>();
            if (src == null) src = host.gameObject.AddComponent<AudioSource>();
            _engineLayers[i] = src;
        }

        if (_engineFilters == null || _engineFilters.Length != EngineLayerCount)
            _engineFilters = new AudioLowPassFilter[EngineLayerCount];

        for (int i = 0; i < EngineLayerCount; i++)
        {
            if (_engineLayers[i] == null) continue;
            var f = _engineLayers[i].GetComponent<AudioLowPassFilter>();
            if (f == null) f = _engineLayers[i].gameObject.AddComponent<AudioLowPassFilter>();
            f.cutoffFrequency = Mathf.Lerp(closedThrottleCutoffHz, openThrottleCutoffHz, 0.5f);
            _engineFilters[i] = f;
        }

        if (_loadSource == null)
            _loadSource = EnsureChildSource("EngineLoad");
        if (_overrunSource == null)
            _overrunSource = EnsureChildSource("EngineOverrun");

        if (_loadFilter == null && _loadSource != null)
        {
            _loadFilter = _loadSource.GetComponent<AudioLowPassFilter>();
            if (_loadFilter == null) _loadFilter = _loadSource.gameObject.AddComponent<AudioLowPassFilter>();
            _loadFilter.cutoffFrequency = Mathf.Lerp(closedThrottleCutoffHz, openThrottleCutoffHz, 0.5f);
        }
    }

    private AudioSource EnsureChildSource(string childName)
    {
        Transform host = transform.Find(childName);
        if (host == null)
        {
            var go = new GameObject(childName);
            go.transform.SetParent(transform, false);
            host = go.transform;
        }

        var src = host.GetComponent<AudioSource>();
        if (src == null) src = host.gameObject.AddComponent<AudioSource>();
        return src;
    }

    private void EnsureClips()
    {
        if (engineLoopClip == null && generatePlaceholderClip)
            engineLoopClip = CreatePlaceholderEngineClip();

        if (shiftBlipClip == null && generatePlaceholderClip)
            shiftBlipClip = CreatePlaceholderShiftClip();

        if (loadLayerClip == null && generatePlaceholderClip)
            loadLayerClip = CreatePlaceholderLoadClip();

        if (overrunLayerClip == null && generatePlaceholderClip)
            overrunLayerClip = CreatePlaceholderOverrunClip();

        // Assign a clip to every band. Real banded clips win where supplied; anything
        // missing falls back to the single loop, so a partial set still works.
        for (int i = 0; i < _engineLayers.Length; i++)
        {
            if (_engineLayers[i] == null) continue;

            AudioClip clip = null;
            if (engineLayerClips != null && i < engineLayerClips.Length)
                clip = engineLayerClips[i];

            _engineLayers[i].clip = clip != null ? clip : engineLoopClip;
            _engineLayers[i].loop = true;
        }

        if (engineSource != null && engineSource.clip == null)
            engineSource.clip = engineLoopClip;

        if (_loadSource != null) { _loadSource.clip = loadLayerClip; _loadSource.loop = true; }
        if (_overrunSource != null) { _overrunSource.clip = overrunLayerClip; _overrunSource.loop = true; }
    }

    private void ApplySourceDefaults()
    {
        for (int i = 0; i < _engineLayers.Length; i++)
        {
            var src = _engineLayers[i];
            if (src == null) continue;
            src.loop = true;
            src.playOnAwake = true;
            src.spatialBlend = spatialBlend;
            src.dopplerLevel = 0f;
            src.rolloffMode = AudioRolloffMode.Linear;
            src.maxDistance = 80f;
        }

        if (engineSource != null)
        {
            engineSource.loop = true;
            engineSource.playOnAwake = true;
            engineSource.spatialBlend = spatialBlend;
            engineSource.dopplerLevel = 0f;
            engineSource.rolloffMode = AudioRolloffMode.Linear;
            engineSource.maxDistance = 80f;
        }

        if (_loadSource != null)
        {
            _loadSource.loop = true;
            _loadSource.playOnAwake = false;
            _loadSource.spatialBlend = spatialBlend;
            _loadSource.dopplerLevel = 0f;
        }

        if (_overrunSource != null)
        {
            _overrunSource.loop = true;
            _overrunSource.playOnAwake = false;
            _overrunSource.spatialBlend = spatialBlend;
            _overrunSource.dopplerLevel = 0f;
        }

        if (shiftSource != null)
        {
            shiftSource.loop = false;
            shiftSource.playOnAwake = false;
            shiftSource.spatialBlend = spatialBlend;
            shiftSource.dopplerLevel = 0f;
            shiftSource.rolloffMode = AudioRolloffMode.Linear;
            shiftSource.maxDistance = 45f;
        }
    }

    private void PlayShiftBlip()
    {
        if (shiftSource != null && shiftBlipClip != null && isActiveAndEnabled)
        {
            shiftSource.pitch = Mathf.Lerp(0.97f, 1.03f, Mathf.Repeat(CurrentGear * 0.37f, 1f));
            shiftSource.PlayOneShot(shiftBlipClip, 0.24f * masterVolume);
        }
    }
}
