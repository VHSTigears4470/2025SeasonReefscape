package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.KitbotDriveConstants;
import frc.robot.Constants.ModuleConstants;
 
public final class Configs {
    public static final class MAXSwerveModule {
        // Swerve
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        // Kitbot Drive
        public static final SparkMaxConfig frontRightKitbot = new SparkMaxConfig();
        public static final SparkMaxConfig frontLeftKitbot = new SparkMaxConfig();
        public static final SparkMaxConfig rearRightKitbot = new SparkMaxConfig();
        public static final SparkMaxConfig rearLeftKitbot = new SparkMaxConfig();

        //Coral Subsystem
        public static final SparkMaxConfig intakeMotorTop = new SparkMaxConfig();
        public static final SparkMaxConfig intakeMotorBottom = new SparkMaxConfig();
        public static final SparkMaxConfig intakeMotorArm= new SparkMaxConfig();

        //AlgaeSubsystem
        public static final SparkMaxConfig algaeTopMotor = new SparkMaxConfig();
        public static final SparkMaxConfig algaeArmMotor = new SparkMaxConfig();

        //ClimbSubsystem
        public static final SparkMaxConfig climbMotor = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.k_WheelDiameterMeters * Math.PI
                    / ModuleConstants.k_DrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.k_DriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
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
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);


                double kitBotFactor = 2 * Math.PI * KitbotDriveConstants.k_kitbotWheelRadius / KitbotDriveConstants.k_kitbotGearRatio;
                frontRightKitbot
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                frontRightKitbot.encoder
                    .positionConversionFactor(kitBotFactor) // meters
                    .velocityConversionFactor(kitBotFactor / 60.0); // meters per second

                frontLeftKitbot
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                frontLeftKitbot.encoder
                    .positionConversionFactor(kitBotFactor) // meters
                    .velocityConversionFactor(kitBotFactor / 60.0); // meters per second

                rearRightKitbot
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                    rearRightKitbot.encoder
                    .positionConversionFactor(kitBotFactor) // meters
                    .velocityConversionFactor(kitBotFactor / 60.0); // meters per second
                rearRightKitbot
                    .follow(KitbotDriveConstants.k_frontRightMotorID, false);

                rearLeftKitbot
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
                rearLeftKitbot.encoder
                    .positionConversionFactor(kitBotFactor) // meters
                    .velocityConversionFactor(kitBotFactor / 60.0); // meters per second
                rearRightKitbot
                    .follow(KitbotDriveConstants.k_frontLeftMotorID, false);

                intakeMotorTop
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO
                intakeMotorTop.encoder
                    .positionConversionFactor(drivingFactor) //TODO
                    .velocityConversionFactor(drivingFactor / 60.0); 
                intakeMotorTop
                    .follow(IntakeConstants.k_topID, false);

                intakeMotorBottom
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO
                intakeMotorBottom.encoder
                    .positionConversionFactor(drivingFactor) //TODO
                    .velocityConversionFactor(drivingFactor / 60.0); 
                intakeMotorBottom
                    .follow(IntakeConstants.k_botID, false);

                intakeMotorArm
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO
                intakeMotorArm.encoder
                    .positionConversionFactor(drivingFactor) //TODO
                    .velocityConversionFactor(drivingFactor / 60.0); 
                intakeMotorArm
                    .follow(IntakeConstants.k_armID, false);

                algaeTopMotor
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO
                algaeTopMotor.encoder
                    .positionConversionFactor(drivingFactor) //TODO
                    .velocityConversionFactor(drivingFactor / 60.0); 
                algaeTopMotor
                    .follow(AlgaeConstants.k_algaeTopID, false);

                algaeArmMotor
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO
                algaeArmMotor.encoder
                    .positionConversionFactor(drivingFactor) //TODO
                    .velocityConversionFactor(drivingFactor / 60.0); 
                algaeArmMotor
                    .follow(AlgaeConstants.k_algaeArmID, false);

                climbMotor
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50); //TODO
                climbMotor.encoder
                    .positionConversionFactor(drivingFactor) //TODO
                    .velocityConversionFactor(drivingFactor / 60.0); 
                climbMotor
                    .follow(ClimbConstants.k_climbMotorID, false);

                

                
        }
    }
}