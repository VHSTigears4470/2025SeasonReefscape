package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.DebuggingConstants;
import frc.robot.Constants.KitbotDriveConstants;

public class KitbotDriveSubsystem extends SubsystemBase {

    // motors
    private final SparkMax m_frontRightMotor;
    private final SparkMax m_frontLeftMotor;
    private final SparkMax m_rearRightMotor;
    private final SparkMax m_rearLeftMotor;

    private final RelativeEncoder m_frontRightEncoder;
    private final RelativeEncoder m_frontLeftEncoder;
    private final RelativeEncoder m_rearRightEncoder;
    private final RelativeEncoder m_rearLeftEncoder;

    private final DifferentialDrive m_differentialDrive;

    /* Kitbot Drive Subsystem */
    public KitbotDriveSubsystem() {

        // Initialize variables for motors
        m_frontRightMotor = new SparkMax(KitbotDriveConstants.k_frontRightMotorID, MotorType.kBrushless);
        m_frontLeftMotor = new SparkMax(KitbotDriveConstants.k_frontLeftMotorID, MotorType.kBrushless);
        m_rearRightMotor = new SparkMax(KitbotDriveConstants.k_rearRightMotorID, MotorType.kBrushless);
        m_rearLeftMotor = new SparkMax(KitbotDriveConstants.k_rearLeftMotorID, MotorType.kBrushless);

        m_frontRightMotor.configure(Configs.KitbotConfigs.frontRightKitbot, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        m_frontLeftMotor.configure(Configs.KitbotConfigs.frontLeftKitbot, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        m_rearRightMotor.configure(Configs.KitbotConfigs.rearRightKitbot, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        m_rearLeftMotor.configure(Configs.KitbotConfigs.rearLeftKitbot, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        m_frontRightEncoder = m_frontRightMotor.getEncoder();
        m_frontLeftEncoder = m_frontLeftMotor.getEncoder();
        m_rearRightEncoder = m_rearRightMotor.getEncoder();
        m_rearLeftEncoder = m_rearLeftMotor.getEncoder();

        m_differentialDrive = new DifferentialDrive(m_frontLeftMotor, m_frontRightMotor);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        m_differentialDrive.arcadeDrive(xSpeed, zRotation);
    }

    public void tankDrive(double xSpeed, double ySpeed) {
        m_differentialDrive.tankDrive(xSpeed, ySpeed);
    }

    // Gets encoders
    public double getFrontRightEncoder() {
        return m_frontRightEncoder.getPosition();
    }

    public double getFrontLeftEncoder() {
        return m_frontLeftEncoder.getPosition();
    }

    public double getRearRightEncoder() {
        return m_rearRightEncoder.getPosition();
    }

    public double getRearLeftEncoder() {
        return m_rearLeftEncoder.getPosition();
    }

    // This stops the motors
    public void stopMotors() {
        m_frontRightMotor.stopMotor();
        m_frontLeftMotor.stopMotor();
        m_rearRightMotor.stopMotor();
        m_rearLeftMotor.stopMotor();
    }

    @Override
    public void periodic() {
        if(DebuggingConstants.k_kitbotDriveDebug) {
            updateSmartDashboard();
        } 
    }

    // Dashboard methods
    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Front Right Encoder", getFrontRightEncoder());
        SmartDashboard.putNumber("Front Left Encoder", getFrontLeftEncoder());
        SmartDashboard.putNumber("Rear Right Encoder", getRearRightEncoder());
        SmartDashboard.putNumber("Rear Left Encoder", getRearLeftEncoder());
    }
}
