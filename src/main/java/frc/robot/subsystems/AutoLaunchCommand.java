package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
/*
import frc.robot.subsystems.Launcher;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import java.util.HashMap;
*/

public class AutoLaunchCommand extends InstantCommand {

    
    

    

    public AutoLaunchCommand() {
        super(() -> {
            
        Launcher.m_launcherClosedLoopController12.setSetpoint(DriveConstants.launcherInSpeed, SparkMax.ControlType.kVelocity);
        Launcher.m_launcherClosedLoopController13.setSetpoint(DriveConstants.launcherInSpeed, SparkMax.ControlType.kVelocity);

        });
    }
}





