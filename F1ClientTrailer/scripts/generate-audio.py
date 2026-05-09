import math
import struct
import wave
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
AUDIO = ROOT / "public" / "audio"
AUDIO.mkdir(parents=True, exist_ok=True)
RATE = 48_000


def write_wav(path: Path, samples: list[float]) -> None:
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(2)
        wav.setsampwidth(2)
        wav.setframerate(RATE)
        frames = bytearray()
        for sample in samples:
            value = max(-1.0, min(1.0, sample))
            packed = struct.pack("<h", int(value * 32767))
            frames += packed + packed
        wav.writeframes(frames)


def envelope(t: float, duration: float) -> float:
    attack = min(1.0, t / 0.12)
    release = min(1.0, (duration - t) / 0.65)
    return max(0.0, min(1.0, attack, release))


def music_loop() -> list[float]:
    duration = 8.0
    samples: list[float] = []
    bass_notes = [55, 55, 65.41, 73.42, 82.41, 73.42, 65.41, 55]
    for i in range(int(duration * RATE)):
        t = i / RATE
        beat = int(t * 2) % len(bass_notes)
        local = (t * 2) % 1
        bass = math.sin(2 * math.pi * bass_notes[beat] * t) * (0.35 + 0.65 * math.exp(-local * 5))
        pulse = math.sin(2 * math.pi * 110 * t) * 0.16
        grit = math.sin(2 * math.pi * 880 * t) * 0.035 * (1 if local < 0.18 else 0)
        hat = ((math.sin(2 * math.pi * 7200 * t) > 0) * 2 - 1) * 0.035 * (1 if (t * 8) % 1 < 0.08 else 0)
        samples.append((bass * 0.48 + pulse + grit + hat) * envelope(t, duration))
    return samples


def whoosh() -> list[float]:
    duration = 0.55
    samples: list[float] = []
    for i in range(int(duration * RATE)):
        t = i / RATE
        sweep = 180 + 1600 * (t / duration) ** 1.8
        noise = math.sin(2 * math.pi * sweep * t) + math.sin(2 * math.pi * (sweep * 1.37) * t)
        samples.append(noise * 0.18 * envelope(t, duration))
    return samples


def impact() -> list[float]:
    duration = 1.0
    samples: list[float] = []
    for i in range(int(duration * RATE)):
        t = i / RATE
        hit = math.sin(2 * math.pi * 48 * t) * math.exp(-t * 5)
        click = math.sin(2 * math.pi * 2200 * t) * math.exp(-t * 30)
        samples.append((hit * 0.85 + click * 0.22) * envelope(t, duration))
    return samples


write_wav(AUDIO / "music-loop.wav", music_loop())
write_wav(AUDIO / "whoosh.wav", whoosh())
write_wav(AUDIO / "impact.wav", impact())
print(f"Wrote audio assets to {AUDIO}")
