package frc.robot.subsystems;

public class Event {


import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PathPlannerLogging;

import java.util.HashMap;

public class EventLaunch {

    public static void main(String[] args) {
        // 1. Create an event map (marker name -> Event)
        HashMap<String, Event> eventMap = new HashMap<>();

        // 2. Add an event that runs custom code
        eventMap.put("LaunchBall", new Event(() -> {
            System.out.println("Launching ball now!");
            // Your robot-specific code here
            // e.g., launcherMotor.set(1.0);

        Launcher.m_launcherClosedLoopController12.setSetpoint(DriveConstants.launcherOutSpeed, SparkMax.ControlType.kVelocity);
        Launcher.m_launcherClosedLoopController13.setSetpoint(DriveConstants.launcherOutSpeed, SparkMax.ControlType.kVelocity);

        }));

        eventMap.put("LowerTarAngle", new Event(() -> {
            System.out.println("Lowering target...");
            // targetMotor.setPosition(0.0);

        Launcher.m_targetClosedLoopController.setSetpoint(DriveConstants.tarDecreaseAng_Pos, SparkMax.ControlType.kPosition);

        }));

        // 3. Load a path that contains these markers
        PathPlannerPath path = PathPlannerPath.fromPathFile("ExamplePath");

        // 4. Create a trajectory from the path
        PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, 0.0, 0.0);

        // 5. Simulate following the path and triggering events
        for (State state : trajectory.getStates()) {
            // Check if this state has an event marker
            if (state.eventMarker != null) {
                String markerName = state.eventMarker.name;
                Event event = eventMap.get(markerName);
                if (event != null) {
                    event.execute(); // Trigger the event
                }
            }
        }
    }
}




}
