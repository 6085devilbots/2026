package frc.robot;
/*/
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends Command {

    private final DriveSubsystem m_drive;
    public static ProfiledPIDController m_pidController;

    public TurnToAngle(double targetAngleDegrees, DriveSubsystem drive){
        m_drive = drive;
        // Tune these constants for your robot's weight and friction
        m_pidController = new ProfiledPIDController(0.05, 0, 0.001,
        new TrapezoidProfile.Constraints(2, 90)
        );

        // MAXSwerve usually uses -180 to 180 degrees
        m_pidController.enableContinuousInput(-180, 180);
        m_pidController.setTolerance(2.0, 5);

        addRequirements(m_drive);
        m_pidController.setGoal(targetAngleDegrees);
    }

    @Override
    public void execute() {

      double rotationSpeed = m_pidController.calculate(m_drive.getHeading());
      

      // Use the standard drive method: (xSpeed ySpeed rotSpeed FieldRelative
      // We set X and Y to 0 so the robot only rotates in place.
      DriveSubsystem.drive(RobotContainer.m_driverController.getLeftX(), RobotContainer.m_driverController.getLeftY(), rotationSpeed, true);

    }

    @Override
    public boolean isFinished() {

      //command ends when robot is at the target angle
      return m_pidController.atGoal();

    }

    @Override
    public void end(boolean interrupted) {

     // stop the robot when finished
     DriveSubsystem.drive(0, 0, 0, true);

    }

    

}
*/