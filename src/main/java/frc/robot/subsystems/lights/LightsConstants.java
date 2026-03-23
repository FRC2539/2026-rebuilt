package frc.robot.subsystems.lights;

import com.ctre.phoenix6.signals.RGBWColor;

public class LightsConstants {
  // #region Constants
  public static int CandleID = 19;
  public static int lastIndex = 8;

  // #region Segments
  public static LEDSegment[] RightSegments = {LEDSegment.RightBottom, LEDSegment.RightTop};
  public static LEDSegment[] ShooterSegments = {LEDSegment.ShooterBottom, LEDSegment.ShooterTop};
  public static LEDSegment[] CenterSegments = {LEDSegment.CenterBottom, LEDSegment.CenterTop};
  public static LEDSegment[] LeftSegments = {LEDSegment.LeftBottom, LEDSegment.LeftTop};

  public static LEDSegment[] InnerSegments = {
    LEDSegment.ShooterTop, LEDSegment.ShooterBottom, LEDSegment.CenterBottom, LEDSegment.CenterTop
  };
  public static LEDSegment[] OuterSegments = {
    LEDSegment.RightTop, LEDSegment.RightBottom, LEDSegment.LeftBottom, LEDSegment.LeftTop
  };

  public static LEDSegment[] AllSegments = {
    LEDSegment.ShooterTop,
    LEDSegment.RightTop,
    LEDSegment.RightBottom,
    LEDSegment.ShooterBottom,
    LEDSegment.CenterBottom,
    LEDSegment.LeftBottom,
    LEDSegment.LeftTop,
    LEDSegment.CenterTop
  };
  public static LEDSegment[] NotShooterSegments = {
    LEDSegment.RightTop,
    LEDSegment.RightBottom,
    LEDSegment.CenterBottom,
    LEDSegment.LeftBottom,
    LEDSegment.LeftTop,
    LEDSegment.CenterTop
  };

  public static LEDSegment[] QuadsLeft = {LEDSegment.QuadLeftTop, LEDSegment.QuadLeftBottom};
  public static LEDSegment[] QuadsRight = {LEDSegment.QuadRightTop, LEDSegment.QuadRightBottom};
  public static LEDSegment[] QuadsTop = {LEDSegment.QuadLeftTop, LEDSegment.QuadRightTop};
  public static LEDSegment[] QuadsBottom = {LEDSegment.QuadLeftBottom, LEDSegment.QuadRightBottom};

  public static LEDSegment[] AllQuadSegments = {
    LEDSegment.QuadRightTop,
    LEDSegment.QuadRightBottom,
    LEDSegment.QuadLeftBottom,
    LEDSegment.QuadLeftTop
  };

  // #region Colors
  public static class ColorPalette {
    public static RGBWColor Orange = new RGBWColor(255, 25, 0);
    public static RGBWColor Red = new RGBWColor(255, 0, 0);
    public static RGBWColor Yellow = new RGBWColor(242, 60, 0);
    public static RGBWColor Green = new RGBWColor(56, 209, 0);
    public static RGBWColor Blue = new RGBWColor(8, 32, 255);
    public static RGBWColor White = new RGBWColor(255, 255, 255);
    public static RGBWColor Purple = new RGBWColor(200, 0, 200);
    public static RGBWColor Black = new RGBWColor(0, 0, 0);

    static RGBWColor Crossfade(RGBWColor a, RGBWColor b, double ratio) {
      return new RGBWColor(
          (int) (a.Red * (1 - ratio) - b.Red),
          (int) (a.Green * (1 - ratio) - b.Green),
          (int) (a.Blue * (1 - ratio) - b.Blue));
    }
  }
}
