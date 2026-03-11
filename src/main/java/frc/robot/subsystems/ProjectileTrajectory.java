package frc.robot.subsystems;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;

import frc.robot.Constants.DriveConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;



public class ProjectileTrajectory {

        public static double avgLaunchVelocity(){
            double rVelocity = Intake.m_launcherEncoder12.getVelocity();
            double lVelocity = Intake.m_launcherEncoder13.getVelocity();

         return ((rVelocity + lVelocity) / 2);
        };

        /**
         * 
         * Use this method to calculate the launch angle (from horizontal) that that the
         * shooter should be set to.
         * 
         * @param initialVelocity Initial Velocity in meters per second
         * @param distanceFromGoal Distance from the goal in meters
         * @param initialHeight Height that the robot is shooting from in meters
         * @param targetHeight Height of the target in meters
         * @return The angle that the robot should launch at for the ball to go in the
         *         goal in degrees
         */
        public static double calcLaunchAngle(double initialVelocity, double distanceFromGoal, double initialHeight,
                double targetHeight) {
            final double g = 9.81; // Switch to 32.2 to use feet as your units

            double quadraticTerm = (Math.pow(initialVelocity, 2) + Math.sqrt(Math.pow(initialVelocity, 4)
                    - g * (g * Math.pow(distanceFromGoal, 2) + 2 * (targetHeight - initialHeight) * Math.pow(initialVelocity, 2))))
                    / (g * distanceFromGoal);
                    
            SmartDashboard.putNumber("Quadratic Term", quadraticTerm);

            return Math.toDegrees(Math.atan(quadraticTerm));
        }

        /**
         * 
         * Use this method to calculate how fast the ball comes out of the shooter in meters per second. 
         * 
         * @param motorRPM
         * @return The initial velocity of the ball coming out of the shooter in meters per second
         */
        public static double calcInitialVelocity(double motorRPM) {
            // Launcher wheel diameter. Probably should be put in Constants.java
            double shooterWheelDia = 0.1016; // meters
            
            // Calculate the rpm of the wheel
            double wheelRPM = motorRPM * 2.8;

            // Since we want the output in meters per second, we need to convert rpm to rps
            double wheelRPS = wheelRPM / 60;

            // Calculate the tangential velocity of the shooter wheel
            double tangentialVelocity = shooterWheelDia * Math.PI * wheelRPS; // meters per second

            // The ball is acted upon like a rack and pinion mechanism. This means that the ball velocity will the average of its "racks"
            // One of the racks is the wheel going at whatever tangential velocity, and the other is the wall moving at 0 velocity
            double ballExitVelocity = tangentialVelocity / 2;

            // Unfortuantely, the world is not perfect so we need an efficency factor. We calculated it ot be about .785 earlier
            return ballExitVelocity * .785;
        }

    }
