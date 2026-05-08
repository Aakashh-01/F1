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
                speedMultiplier = 1.05f;
                brakingMargin = 0.88f;
                cornerConfidence = 0.96f;
                reactionDelay = 0.04f;
                overtakeWillingness = 0.82f;
                avoidanceCaution = 0.56f;
                break;
            default:
                speedMultiplier = 0.92f;
                brakingMargin = 1.1f;
                cornerConfidence = 0.82f;
                reactionDelay = 0.12f;
                overtakeWillingness = 0.45f;
                avoidanceCaution = 0.72f;
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
