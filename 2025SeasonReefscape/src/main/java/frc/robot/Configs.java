package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.KitbotDriveConstants;
import frc.robot.Constants.ModuleConstants;
 
public final class Configs {
    public static final class MAXSwerveModule {
        // Swerve
        // public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();

        public static final SparkMaxConfig frontRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig frontLeftDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rearRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rearLeftDrivingConfig = new SparkMaxConfig();

        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();


        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double d_drivingFactor = ModuleConstants.k_WheelDiameterMeters * Math.PI
                    / ModuleConstants.k_DrivingMotorReduction;
            double d_turningFactor = 2 * Math.PI;
            double d_drivingVelocityFeedForward = 1 / ModuleConstants.k_DriveWheelFreeSpeedRps;
            
            // drivingConfig
            //         .idleMode(IdleMode.kBrake)
            //         .smartCurrentLimit(50)
            //         .inverted(true);
            // drivingConfig.encoder
            //         .positionConversionFactor(drivingFactor) // meters
            //         .velocityConversionFactor(drivingFactor / 60.0);
            // drivingConfig.closedLoop
            //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //         // These are example gains you may need to them for your own robot!
            //         .pid(0.04, 0, 0)
            //         .velocityFF(drivingVelocityFeedForward)
            //         .outputRange(-1, 1);

            frontRightDrivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);
            frontRightDrivingConfig.encoder
                    .positionConversionFactor(d_drivingFactor) // meters
                    .velocityConversionFactor(d_drivingFactor / 60.0);
            frontRightDrivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to (change?) them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(d_drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            frontLeftDrivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);
            frontLeftDrivingConfig.encoder
                    .positionConversionFactor(d_drivingFactor) // meters
                    .velocityConversionFactor(d_drivingFactor / 60.0);
            frontLeftDrivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(d_drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            rearRightDrivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(true);
            rearRightDrivingConfig.encoder
                    .positionConversionFactor(d_drivingFactor) // meters
                    .velocityConversionFactor(d_drivingFactor / 60.0);
            rearRightDrivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(d_drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            rearLeftDrivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50)
                    .inverted(false);
            rearLeftDrivingConfig.encoder
                    .positionConversionFactor(d_drivingFactor) // meters
                    .velocityConversionFactor(d_drivingFactor / 60.0);
            rearLeftDrivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(d_drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(d_turningFactor) // radians
                    .velocityConversionFactor(d_turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, d_turningFactor); 
        }
    }
    
    public static final class KitbotConfigs {
        // Kitbot Drive
        public static final SparkMaxConfig frontRightKitbot = new SparkMaxConfig();
        public static final SparkMaxConfig frontLeftKitbot = new SparkMaxConfig();
        public static final SparkMaxConfig rearRightKitbot = new SparkMaxConfig();
        public static final SparkMaxConfig rearLeftKitbot = new SparkMaxConfig();

        static {
            double kitBotFactor = 1 / ( 2 * Math.PI * KitbotDriveConstants.k_kitbotWheelRadius * KitbotDriveConstants.k_kitbotGearRatio);

                //right motor no invert sets encoder, limits, and brakes
                frontRightKitbot
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                frontRightKitbot.encoder
                    .positionConversionFactor(kitBotFactor) // meters
                    .velocityConversionFactor(kitBotFactor / 60.0); // meters per second
                frontRightKitbot
                    .inverted(false); // ORBB: false | AR: true    


                //left motor no invert sets encoder, limits, and brakes
                frontLeftKitbot
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                frontLeftKitbot.encoder
                    .positionConversionFactor(kitBotFactor) // meters
                    .velocityConversionFactor(kitBotFactor / 60.0); // meters per second
                
                    //right back motor not invert sets encoder, limits, and brakes
                rearRightKitbot
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                rearRightKitbot.encoder
                    .positionConversionFactor(kitBotFactor) // meters
                    .velocityConversionFactor(kitBotFactor / 60.0); // meters per second
                rearRightKitbot
                    .follow(KitbotDriveConstants.k_frontRightMotorID, false);

                    //left motor not invert sets encoder, limits, and brakes
                rearLeftKitbot
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                rearLeftKitbot.encoder
                    .positionConversionFactor(kitBotFactor) // meters
                    .velocityConversionFactor(kitBotFactor / 60.0); // meters per second
                rearLeftKitbot
                    .follow(KitbotDriveConstants.k_frontLeftMotorID, false);
        }
    }

    public static final class AlgaeConfigs {

        //AlgaeSubsystem
        public static final SparkMaxConfig algaeIntakeMotor = new SparkMaxConfig();
        public static final SparkMaxConfig algaeArmMotor = new SparkMaxConfig();

        static {
            double d_algaeFactor = 1 / 45 * 2 * Math.PI; //TODO: Double Check

            algaeIntakeMotor
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50); //TODO

            algaeArmMotor
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50); //TODO
            algaeArmMotor.encoder
                .positionConversionFactor(d_algaeFactor) //degrees
                .velocityConversionFactor(d_algaeFactor / 60.0); 
            algaeArmMotor.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // These are example gains you may need to (change?) them for your own robot!
                .pid(0.01, 0, 0)
                .outputRange(AlgaeConstants.k_algaeArmMinOutput, AlgaeConstants.k_algaeArmMaxOutput); // TODO, FIND RELEASTIC VALUES
        }

    }
    
    public static final class ClimbConfigs {

        //ClimbSubsystem
        public static final SparkMaxConfig climbMotor = new SparkMaxConfig();

        static {
            double d_climbFactor = 1 / 108 * 2 * Math.PI; //TODO: Double Check
            climbMotor
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO
            climbMotor.encoder
                .positionConversionFactor(d_climbFactor) //degrees
                .velocityConversionFactor(d_climbFactor / 60.0); 
        }
    }

    public static final class CoralConfigs {

        //Coral Subsystem
        public static final SparkMaxConfig coralIntakeMotor = new SparkMaxConfig();
        public static final SparkMaxConfig coralArmMotor = new SparkMaxConfig();

        static {
            // double d_coralFactor = 1 / 20 * 2 * Math.PI; //TODO: Double Check

            coralIntakeMotor
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO 

            coralArmMotor.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
            coralArmMotor
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(0.1)
                .outputRange(-1, 1)
                .maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(2000) // TODO
                .maxAcceleration(10000) // TODO
                .allowedClosedLoopError(0.25); // TODO

            // coralArmMotor
            //     .idleMode(IdleMode.kBrake)
            //     .smartCurrentLimit(50); //TODO
            // coralArmMotor.encoder
            //     .positionConversionFactor(d_coralFactor) //degrees
            //     .velocityConversionFactor(d_coralFactor / 60.0); 
        }
    }

}