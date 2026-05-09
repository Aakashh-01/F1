import { Audio } from "@remotion/media";
import {
  AbsoluteFill,
  Easing,
  Img,
  Sequence,
  interpolate,
  staticFile,
  useCurrentFrame,
  useVideoConfig,
} from "remotion";
import { shots, type TrailerShot } from "./assets";

const clamp = {
  extrapolateLeft: "clamp" as const,
  extrapolateRight: "clamp" as const,
};

const beatFrames = [0, 150, 300, 420, 540, 660, 780, 900, 1020, 1140, 1260, 1380, 1500, 1620];

const panelStyle: React.CSSProperties = {
  position: "absolute",
  left: 92,
  bottom: 86,
  width: 760,
  color: "white",
  fontFamily: "Inter, Arial, Helvetica, sans-serif",
  textTransform: "uppercase",
  letterSpacing: 0,
};

const useEase = (frame: number, input: [number, number], output: [number, number]) =>
  interpolate(frame, input, output, {
    ...clamp,
    easing: Easing.bezier(0.16, 1, 0.3, 1),
  });

const ShotScene = ({
  shot,
  duration,
  index,
}: {
  shot: TrailerShot;
  duration: number;
  index: number;
}) => {
  const frame = useCurrentFrame();
  const { fps } = useVideoConfig();
  const progress = useEase(frame, [0, duration], [0, 1]);
  const scale = interpolate(progress, [0, 1], [shot.startScale, shot.endScale]);
  const x = interpolate(progress, [0, 1], [shot.x, -shot.x * 0.25]);
  const y = interpolate(progress, [0, 1], [shot.y, -shot.y * 0.2]);
  const fadeIn = useEase(frame, [0, 0.32 * fps], [0, 1]);
  const fadeOut = useEase(frame, [duration - 0.45 * fps, duration], [1, 0]);
  const textIn = useEase(frame, [0.2 * fps, 0.9 * fps], [34, 0]);
  const accentWidth = useEase(frame, [0.1 * fps, 0.85 * fps], [0, 260]);
  const speedLine = useEase(frame, [0.2 * fps, duration * 0.66], [-420, 1980]);

  return (
    <AbsoluteFill style={{ backgroundColor: "#050607", overflow: "hidden" }}>
      <Img
        src={staticFile(shot.file)}
        style={{
          position: "absolute",
          width: "100%",
          height: "100%",
          objectFit: "cover",
          transform: `translate(${x}px, ${y}px) scale(${scale})`,
          filter: "contrast(1.12) saturate(1.13) brightness(0.86)",
        }}
      />
      <AbsoluteFill
        style={{
          background:
            "linear-gradient(90deg, rgba(3,5,8,0.82) 0%, rgba(3,5,8,0.36) 42%, rgba(3,5,8,0.08) 100%)",
        }}
      />
      <AbsoluteFill
        style={{
          background:
            "linear-gradient(180deg, rgba(0,0,0,0.2) 0%, transparent 46%, rgba(0,0,0,0.72) 100%)",
        }}
      />
      <div
        style={{
          position: "absolute",
          inset: 0,
          opacity: 0.28,
          background:
            "repeating-linear-gradient(0deg, transparent 0px, transparent 8px, rgba(255,255,255,0.055) 9px)",
        }}
      />
      <div
        style={{
          position: "absolute",
          top: 0,
          left: speedLine,
          width: 330,
          height: "100%",
          transform: "skewX(-20deg)",
          background: `linear-gradient(90deg, transparent, ${shot.tint}55, transparent)`,
          opacity: 0.5 * fadeIn * fadeOut,
        }}
      />
      <div style={{ ...panelStyle, opacity: fadeIn * fadeOut, transform: `translateY(${textIn}px)` }}>
        <div
          style={{
            width: accentWidth,
            height: 6,
            backgroundColor: shot.tint,
            marginBottom: 26,
          }}
        />
        <div style={{ fontSize: 26, fontWeight: 800, color: shot.tint, marginBottom: 14 }}>
          {String(index + 1).padStart(2, "0")} / PREMIUM RACING DEMO
        </div>
        <div style={{ fontSize: 82, lineHeight: 0.88, fontWeight: 950 }}>{shot.title}</div>
        <div style={{ marginTop: 22, fontSize: 28, fontWeight: 700, color: "#e8edf5" }}>{shot.kicker}</div>
      </div>
    </AbsoluteFill>
  );
};

const TitleCard = () => {
  const frame = useCurrentFrame();
  const titleY = useEase(frame, [12, 60], [72, 0]);
  const titleOpacity = useEase(frame, [0, 42], [0, 1]);
  const stripe = useEase(frame, [12, 100], [0, 1160]);
  const cut = useEase(frame, [102, 140], [0, 1]);

  return (
    <AbsoluteFill style={{ backgroundColor: "#030406", color: "white", overflow: "hidden" }}>
      <Img
        src={staticFile("stills/still01.png")}
        style={{
          position: "absolute",
          width: "100%",
          height: "100%",
          objectFit: "cover",
          transform: `scale(${useEase(frame, [0, 150], [1.06, 1.18])})`,
          filter: "brightness(0.55) contrast(1.2) saturate(1.05)",
        }}
      />
      <AbsoluteFill style={{ background: "linear-gradient(90deg, rgba(0,0,0,0.88), rgba(0,0,0,0.18))" }} />
      <div
        style={{
          position: "absolute",
          left: 92,
          top: 110,
          width: stripe,
          height: 10,
          background: "linear-gradient(90deg, #e5252a, #ffffff, #2f80ed)",
        }}
      />
      <div
        style={{
          position: "absolute",
          left: 90,
          top: 206,
          fontFamily: "Inter, Arial, Helvetica, sans-serif",
          textTransform: "uppercase",
          opacity: titleOpacity,
          transform: `translateY(${titleY}px)`,
        }}
      >
        <div style={{ fontSize: 42, color: "#ffcb3d", fontWeight: 900, marginBottom: 26 }}>Client Gameplay Showcase</div>
        <div style={{ fontSize: 156, lineHeight: 0.82, fontWeight: 1000 }}>F1 Racing</div>
        <div style={{ fontSize: 104, lineHeight: 0.92, fontWeight: 950, color: "#dfe8f5" }}>Demo</div>
        <div style={{ marginTop: 36, fontSize: 30, fontWeight: 800, color: "#d5dce8" }}>
          Speed, pack racing, camera feel, and playable presentation
        </div>
      </div>
      <div
        style={{
          position: "absolute",
          inset: 0,
          background: "#050607",
          opacity: cut,
        }}
      />
    </AbsoluteFill>
  );
};

const FinalCard = () => {
  const frame = useCurrentFrame();
  const reveal = useEase(frame, [0, 48], [0, 1]);
  const line = useEase(frame, [24, 100], [0, 1180]);

  return (
    <AbsoluteFill style={{ backgroundColor: "#030406", color: "white", overflow: "hidden" }}>
      <Img
        src={staticFile("stills/still18.png")}
        style={{
          position: "absolute",
          width: "100%",
          height: "100%",
          objectFit: "cover",
          transform: `scale(${useEase(frame, [0, 180], [1.05, 1.16])})`,
          filter: "brightness(0.55) contrast(1.16) saturate(1.1)",
        }}
      />
      <AbsoluteFill style={{ background: "linear-gradient(90deg, rgba(0,0,0,0.9), rgba(0,0,0,0.25), rgba(0,0,0,0.76))" }} />
      <div
        style={{
          position: "absolute",
          left: 92,
          top: 185,
          width: line,
          height: 8,
          background: "linear-gradient(90deg, #e5252a, #ffcb3d, #2f80ed)",
        }}
      />
      <div
        style={{
          ...panelStyle,
          top: 270,
          bottom: "auto",
          width: 1200,
          opacity: reveal,
          transform: `translateY(${interpolate(reveal, [0, 1], [42, 0])}px)`,
        }}
      >
        <div style={{ fontSize: 34, fontWeight: 850, color: "#ffcb3d", marginBottom: 24 }}>Built From Live Unity Gameplay</div>
        <div style={{ fontSize: 110, lineHeight: 0.88, fontWeight: 1000 }}>Ready For Client Review</div>
        <div style={{ marginTop: 34, display: "flex", gap: 22, fontSize: 24, fontWeight: 850 }}>
          {["Unity MCP Capture", "Remotion Edit", "1080p Trailer"].map((item) => (
            <div key={item} style={{ border: "2px solid rgba(255,255,255,0.34)", padding: "14px 18px" }}>
              {item}
            </div>
          ))}
        </div>
      </div>
    </AbsoluteFill>
  );
};

const BeatFlash = () => {
  const frame = useCurrentFrame();
  const opacity = beatFrames.reduce((peak, beat) => {
    const local = Math.abs(frame - beat);
    const value = interpolate(local, [0, 8], [0.22, 0], clamp);
    return Math.max(peak, value);
  }, 0);

  return (
    <AbsoluteFill
      style={{
        pointerEvents: "none",
        background: "white",
        opacity,
        mixBlendMode: "screen",
      }}
    />
  );
};

const TimelineHud = () => {
  const frame = useCurrentFrame();
  const { durationInFrames } = useVideoConfig();
  const progress = interpolate(frame, [0, durationInFrames - 1], [0, 1], clamp);

  return (
    <AbsoluteFill style={{ pointerEvents: "none" }}>
      <div
        style={{
          position: "absolute",
          left: 74,
          right: 74,
          top: 58,
          height: 2,
          backgroundColor: "rgba(255,255,255,0.24)",
        }}
      >
        <div style={{ width: `${progress * 100}%`, height: "100%", backgroundColor: "#ffcb3d" }} />
      </div>
      <div
        style={{
          position: "absolute",
          right: 74,
          top: 78,
          color: "rgba(255,255,255,0.78)",
          fontFamily: "Inter, Arial, Helvetica, sans-serif",
          fontSize: 22,
          fontWeight: 800,
          letterSpacing: 0,
          textTransform: "uppercase",
        }}
      >
        F1 Racing Demo
      </div>
    </AbsoluteFill>
  );
};

export const F1ClientTrailer = () => {
  const { fps } = useVideoConfig();
  const titleDuration = 5 * fps;
  const finalDuration = 8 * fps;
  const shotDuration = Math.floor((60 * fps - titleDuration - finalDuration) / shots.length);

  return (
    <AbsoluteFill style={{ backgroundColor: "#030406" }}>
      <Audio src={staticFile("audio/music-loop.wav")} loop volume={0.58} />
      <Sequence from={0}>
        <Audio src={staticFile("audio/impact.wav")} volume={0.72} />
      </Sequence>
      {beatFrames.slice(1, -1).map((beat) => (
        <Sequence from={beat} key={beat}>
          <Audio src={staticFile("audio/whoosh.wav")} volume={0.18} />
        </Sequence>
      ))}
      <Sequence durationInFrames={titleDuration}>
        <TitleCard />
      </Sequence>
      {shots.map((shot, index) => (
        <Sequence
          key={shot.file}
          from={titleDuration + index * shotDuration}
          durationInFrames={shotDuration + 8}
        >
          <ShotScene shot={shot} duration={shotDuration + 8} index={index} />
        </Sequence>
      ))}
      <Sequence from={60 * fps - finalDuration} durationInFrames={finalDuration}>
        <FinalCard />
      </Sequence>
      <BeatFlash />
      <TimelineHud />
    </AbsoluteFill>
  );
};
