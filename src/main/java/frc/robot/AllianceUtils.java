package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtils {
    //Field length in meters (should be updated every season)
    public static final double FIELD_LENGTH = 17.548;

    public static Pose2d allianceFlip(Pose2d pose) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            return new Pose2d(
                new Translation2d(FIELD_LENGTH - pose.getX(), pose.getY()),
                pose.getRotation().plus(Rotation2d.fromDegrees(180))
            );
        }
        return pose;
    }
}
