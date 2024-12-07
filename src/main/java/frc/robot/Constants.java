package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static final Double TRACKWIDTHMETERS = 0.64008;
    public static final double DRIVEDEADBAND_MPS = 0.01;
    public static final double MAXVELOCITY_MPS = 4.885671;
    public static final double WHEELDIAMETER_METERS = 0.1016;
    public static final double MAXRMP = 918.4;
    public static final Pose2d DRIVEODOMETRY_ORIGN = new Pose2d(1.0, 1.0, new Rotation2d());
}
