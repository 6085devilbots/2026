package frc.robot.subsystems;


import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Launcher;
import frc.robot.Constants.DriveConstants;

import com.pathplanner.lib.events.Event;

import java.util.HashMap;

public class AutoLaunchCommand extends InstantCommand {

    
    /*NamedCommands.registerCommand(
    "LaunchBall",
    new AutoLaunchCommand()
    );

    */

    public AutoLaunchCommand() {
        super(() -> {
            
        Launcher.m_launcherClosedLoopController12.setSetpoint(DriveConstants.launcherInSpeed, SparkMax.ControlType.kVelocity);
        Launcher.m_launcherClosedLoopController13.setSetpoint(DriveConstants.launcherInSpeed, SparkMax.ControlType.kVelocity);

        });
    }
}





