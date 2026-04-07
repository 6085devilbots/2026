// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LiveBottom;
import frc.robot.subsystems.AutoLaunch;
import frc.robot.subsystems.AutoLaunchCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/*
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.AutoLaunch;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.pathplanner.lib.auto.NamedCommands;
*/
import edu.wpi.first.wpilibj2.command.button.POVButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
 
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final AutoLaunch m_autoLaunch = new AutoLaunch();
  @SuppressWarnings("unused")
  public static final Intake m_intake = new Intake();

  //------------------------------------------------
  // Added Offline 3-14-26 in a attempt to get Launcher.java and LiveBottom.java
  @SuppressWarnings("unused")
  private final LiveBottom liveBottom= new LiveBottom();
  @SuppressWarnings("unused")
  private final Launcher m_launcher= new Launcher();
  @SuppressWarnings("unused")
  private final Climber climber= new Climber();
  
  

  private final SendableChooser<Command> m_Chooser = new SendableChooser<>();

 
  

   

   
  
  //------------------------------------------------


  private final SendableChooser<Command> autonChooser;

  

  // The driver's controller
  public static XboxController m_driverController = new XboxController(0);//(OIConstants.kDriverControllerPort);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    
  //NamedCommands.registerCommand( "TurnToAngle", new TurnToAngle(45, m_robotDrive));


  NamedCommands.registerCommand( "SpinUp",
    m_launcher.LauncherOutCommand);

  //NamedCommands.registerCommand( "LaunchSequenceBlue", getFullAutonBlue());

   
 

    // Build an auto chooser. This will use Commands.none() as the default option.
 
  SmartDashboard.putData("Auto Mode", m_Chooser);

 
  
  autonChooser = AutoBuilder.buildAutoChooser();
  SmartDashboard.putData("Auton Chooser", autonChooser);


 



    

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
      

    

  

    
    // Configure default commands

    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> {
            if (Launcher.rotOverRide) {
                // True logic: Uses Launcher.rotCmmd
                DriveSubsystem.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
                    MathUtil.applyDeadband(Launcher.rotCmmd, OIConstants.kDriveDeadband),
                    true);
            } else {
                // False logic: Uses RightX
                DriveSubsystem.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true);
            }
        },
        m_robotDrive // Correctly declares requirement
      )
    );





  

  }

  






public Command getFullAutonBlue(){
      return Commands.sequence(

      Commands.run(()->
    m_autoLaunch.LaunchAuto(DriveSubsystem.blueHub),
    m_autoLaunch
    ) 
      );
}






public Command getFullAutonRed(){
      return Commands.sequence(

      Commands.run(()->
    m_autoLaunch.LaunchAuto(DriveSubsystem.redHub),
    m_autoLaunch
    ),

    Commands.waitSeconds(12),

    Commands.runOnce(()->
       m_autoLaunch.StopLaunch(), m_autoLaunch)

      );
}



 







 

   /*public Command getFullAutonBlue(){
      return Commands.sequence(

      Commands.run(()->
    m_autoLaunch.Launch(DriveSubsystem.blueHub),
    m_autoLaunch
    ),

    m_robotDrive.run(()-> 
       m_robotDrive.driveRobotRelative(new ChassisSpeeds(-0.4, 0, 0)))
       .withTimeout(3.5),

       Commands.runOnce(()-> {
       m_robotDrive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
       m_autoLaunch.StopLaunch();
       }, m_robotDrive, m_autoLaunch)
      );

       }
    
*/


/* 
       public Command getFullAutonRed(){
      return Commands.sequence(

      Commands.run(()->
    m_autoLaunch.Launch(DriveSubsystem.redHub),
    m_autoLaunch
    ),

    m_robotDrive.run(()-> 
       m_robotDrive.driveRobotRelative(new ChassisSpeeds(-0.4, 0, 0)))
       .withTimeout(3.5),

       Commands.runOnce(()-> {
       m_robotDrive.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
       
       }, m_robotDrive, m_autoLaunch)
      );

       }
    
    */


   //public Command getAutonomousCommand() {

   // return m_Chooser.getSelected();

   //}

   //public Command getAutonomousCommand() {

    //return getFullAutonRed();

//   }



  

  public Command getAutonomousCommand() {
    
    return autonChooser.getSelected();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {


// - - - - - - - - - - - - Blue Launch - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


    new JoystickButton(Robot.stick2, Wire.yButton)
    .whileTrue(m_autoLaunch.runEnd(
        () -> m_autoLaunch.Launch(DriveSubsystem.blueHub),
        () -> m_autoLaunch.StopLaunch()

        )
        
        );

        new JoystickButton(Robot.stick2, Wire.xButton)
    .whileTrue(m_autoLaunch.runEnd(
        () -> m_autoLaunch.Launch(DriveSubsystem.blueHub),
        () -> m_autoLaunch.StopLaunch()

        )
        
        );


         new JoystickButton(Robot.stick2, Wire.bButton)
    .whileTrue(m_autoLaunch.runEnd(
        () -> m_autoLaunch.Launch(DriveSubsystem.blueHub),
        () -> m_autoLaunch.StopLaunch()

        )
        
        );


       
        
  
  


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 



// - - - - - - - - - - - - Red Launch - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


    new POVButton(Robot.stick2, Wire.upDpad) 
    .whileTrue(m_autoLaunch.runEnd(
        () -> m_autoLaunch.Launch(DriveSubsystem.redHub),
        () -> m_autoLaunch.StopLaunch()

        )
        
        );

        new POVButton(Robot.stick2, Wire.leftDpad)
    .whileTrue(m_autoLaunch.runEnd(
        () -> m_autoLaunch.Launch(DriveSubsystem.leftFieldRed),
        () -> m_autoLaunch.StopLaunch()

        )
        
        );


         new POVButton(Robot.stick2, Wire.rightDpad)
    .whileTrue(m_autoLaunch.runEnd(
        () -> m_autoLaunch.Launch(DriveSubsystem.rightFieldRed),
        () -> m_autoLaunch.StopLaunch()

        )
        
        );





// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 



    new JoystickButton(m_driverController, Wire.rBumper)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

  }


  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   
    /*
    // Query the latest result from PhotonVision
            var result = Robot.Cam_1.getLatestResult();
            Robot.hasTargets = result.hasTargets();
            SmartDashboard.putBoolean("has target", Robot.hasTargets);

  // Check if the latest result has any targets.
            //if (Robot.hasTargets){
            SmartDashboard.putString("message", "in if loop");
  // Get the current best target.
            PhotonTrackedTarget target = result.getBestTarget();
            int targetID = target.getFiducialId();
  // Get information from target.
            
            double poseAmbiguity = target.getPoseAmbiguity();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            SmartDashboard.putNumber("poseX", bestCameraToTarget.getX());
            SmartDashboard.putNumber("poseY", bestCameraToTarget.getY());
            double finalY = bestCameraToTarget.getY();
            double finalX = bestCameraToTarget.getX();
            //Robot.test = Robot.test + 1;
            //SmartDashboard.putNumber("auton loops", Robot.test);
            */
            

            
    // Create config for trajectory.
   /*  TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.+00
    
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(10.25, 2.05, new Rotation2d(29.07)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(10.5, 2.5), new Translation2d(11.5, 3.75)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(11.92, 4.22, new Rotation2d(0)),
        config);
    
        

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetPose(exampleTrajectory.getInitialPose(), false);


    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> DriveSubsystem.drive(0, 0, 0, false));
    
  }*/
}

