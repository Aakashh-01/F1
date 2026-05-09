import { Composition } from "remotion";
import { F1ClientTrailer } from "./F1ClientTrailer";

export const RemotionRoot = () => {
  return (
    <Composition
      id="F1ClientTrailer"
      component={F1ClientTrailer}
      durationInFrames={1800}
      fps={30}
      width={1920}
      height={1080}
    />
  );
};
