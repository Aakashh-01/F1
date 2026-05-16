using UnityEngine;

public enum AIDifficultyPreset
{
    Easy,
    Medium,
    Hard
}

[CreateAssetMenu(menuName = "F1/AI Difficulty Profile", fileName = "AI_Difficulty_Profile")]
public class AIDifficultyProfile : ScriptableObject
{
    public AIDifficultyPreset preset = AIDifficultyPreset.Medium;
    [Range(0.5f, 1.3f)] public float speedMultiplier = 0.92f;
    [Range(0.6f, 2.0f)] public float brakingMargin = 1.1f;
    [Range(0.5f, 1.2f)] public float cornerConfidence = 0.82f;
    [Range(0f, 0.5f)] public float reactionDelay = 0.12f;
    [Range(0f, 1f)] public float overtakeWillingness = 0.45f;
    [Range(0f, 1f)] public float avoidanceCaution = 0.72f;

    public void ApplyPreset(AIDifficultyPreset targetPreset)
    {
        preset = targetPreset;

        switch (targetPreset)
        {
            case AIDifficultyPreset.Easy:
                speedMultiplier = 0.78f;
                brakingMargin = 1.45f;
                cornerConfidence = 0.68f;
                reactionDelay = 0.22f;
                overtakeWillingness = 0.18f;
                avoidanceCaution = 0.9f;
                break;
            case AIDifficultyPreset.Hard:
                speedMultiplier = 1.18f;
                brakingMargin = 0.74f;
                cornerConfidence = 1.07f;
                reactionDelay = 0.03f;
                overtakeWillingness = 0.98f;
                avoidanceCaution = 0.34f;
                break;
            default:
                speedMultiplier = 1.1f;
                brakingMargin = 0.9f;
                cornerConfidence = 1f;
                reactionDelay = 0.08f;
                overtakeWillingness = 0.74f;
                avoidanceCaution = 0.5f;
                break;
        }
    }

    public static AIDifficultyProfile CreateRuntimeProfile(AIDifficultyPreset preset)
    {
        AIDifficultyProfile profile = CreateInstance<AIDifficultyProfile>();
        profile.ApplyPreset(preset);
        return profile;
    }
}
