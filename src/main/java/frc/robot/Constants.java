// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class driveSettings{
    public static final int kMaxSpeed = 3;
    public static final double kMaxAngularSpeed = Math.PI;
    public static final double kTrackWidth = Units.inchesToMeters(12.585*2);
    public static final double kWheelRadius = Units.inchesToMeters(6.0);
    public static final double kEncoderResolution = 2048;
  }
  public static class pid{
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
  public static class driveSpeeds{
    public static final double maxSpeed = 0.8;
  }
}
