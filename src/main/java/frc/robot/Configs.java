package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.ClosedLoopConfig.feedbackSensor;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.ModuleConstants;

public final class Configs {

    public static final class IntakeLiftSetup {
        public static final SparkMaxConfig intakeliftConfig = new SparkMaxConfig();

// - - - - - - - Intake Lift Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -    
        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double liftFactor = 1;
                double liftVelocityFeedForward = 0;
    
                intakeliftConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(2);
                intakeliftConfig.absoluteEncoder
                        .positionConversionFactor(liftFactor) // meters
                        .velocityConversionFactor(0.00000001); // meters per second
                intakeliftConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(0.5, 0, 0.1) // p 0.04
                         .outputRange(-0.1, 0.1)
                        .feedForward.kV(liftVelocityFeedForward);
                       
            }
        
    }

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 





// - - - - - - - Arm Extend Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class ExtnedArmSetup {
        public static final SparkMaxConfig armExtendConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double extendFactor = 1;
                double extendVelocityFeedForward = 0.02;
    
                armExtendConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(45);
                armExtendConfig.encoder
                        .positionConversionFactor(extendFactor) // meters
                        .velocityConversionFactor(extendFactor / 60.0);  // meters per second
                armExtendConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(0.5, 0, 0)
                         .outputRange(-0.2, 0.2)
                        .feedForward.kV(extendVelocityFeedForward) ;
                       
            }
        

    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


// - - - - - - - Rotate Wrist Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class TargetingMotorSetup {
        public static final SparkMaxConfig targetShooterConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double targetingFactor = 1;
                double targetingVelocityFeedForward = 0;
    
                targetShooterConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(4);
                targetShooterConfig.absoluteEncoder
                        .positionConversionFactor(targetingFactor) // meters
                        .velocityConversionFactor(targetingFactor ); // meters per second
                targetShooterConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(1, 0, 0)
                        .outputRange(-1, 1)
                        .feedForward.kV(targetingVelocityFeedForward) ;
                        
            }
        
    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 







// - - - - - - - Intake Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class intakeSetup {
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double intakeFactor = 1;
                double intakeVelocityFeedForward = 1;
    
                intakeConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40);
                intakeConfig.encoder
                        .positionConversionFactor(intakeFactor) // meters
                        .velocityConversionFactor(intakeFactor / 60.0); // meters per second
                intakeConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(1, 0, 0)
                         .outputRange(-1, 1)
                        .feedForward.kV(intakeVelocityFeedForward) ;
                       
            }
        
    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 







// - - - - - - - Right Launch Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class launcherRightSetup {
        public static final SparkMaxConfig launcherRightConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double launcherRightFactor = 1;
                double launcherRightVelocityFeedForward = .00286;
    
                launcherRightConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40);
                 launcherRightConfig.encoder
                        .positionConversionFactor(launcherRightFactor) // meters
                        .velocityConversionFactor(launcherRightFactor); // meters per second
                launcherRightConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(.0004, 0, .00001)
                         .outputRange(0, 1)
                        .feedForward.kV(launcherRightVelocityFeedForward) ;
                       
            }




            
        
    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 




// - - - - - - - Right Launch Motor Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class launcherLeftSetup {
        public static final SparkMaxConfig launcherLeftConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double launcherLeftFactor = 1;
                double launcherLeftVelocityFeedForward = 0.0034;
    
                launcherLeftConfig
                        .idleMode(IdleMode.kCoast)
                        .inverted(true)
                        .smartCurrentLimit(40);
                launcherLeftConfig.encoder
                        .positionConversionFactor(launcherLeftFactor) // meters
                        .velocityConversionFactor(launcherLeftFactor); // meters per second
                launcherLeftConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(.001, 0, .0001)
                         .outputRange(0, 1)
                        .feedForward.kV(launcherLeftVelocityFeedForward) ;
                       
            }




            
        
    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 







// - - - - - - - Climber Configuration - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

public static final class ClimbSetup {
        public static final SparkMaxConfig ClimbConfig = new SparkMaxConfig();

        
        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double climbFactor = 1;
                double climbVelocityFeedForward = 1;
    
                ClimbConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(100);
                ClimbConfig.encoder
                        .positionConversionFactor(climbFactor) // meters
                        .velocityConversionFactor(climbFactor / 60.0); // meters per second
                ClimbConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(1, 0, 0)
                         .outputRange(-1, 1)
                        .feedForward.kV(climbVelocityFeedForward) ;
                       
            }
        
    }


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 







// - - - - - - - Drive Motors Config - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;
                                             // 2
            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0.1) //.04
                     .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward) ;
                    

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .inverted(true)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.3, 0.0, 0.0) //0.3
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
}
