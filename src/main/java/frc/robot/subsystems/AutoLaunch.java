package frc.robot.subsystems;


import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.events.Event;

import java.util.HashMap;

public class LaunchBallCommand extends InstantCommand {

    public LaunchBallCommand() {
        super(() -> {
            Launcher.m_launcherClosedLoopController12
                .setSetpoint(DriveConstants.launcherOutSpeed);
        });
    }
}





