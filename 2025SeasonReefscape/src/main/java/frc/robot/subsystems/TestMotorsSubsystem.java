package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotorsSubsystem extends SubsystemBase{

    private final SparkMax motor;

    public TestMotorsSubsystem(int id, SparkBaseConfig config) {
        motor = new SparkMax(id, MotorType.kBrushless);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setSpeed(double speed) {
        speed = Math.signum(speed) * Math.min(Math.max(Math.abs(speed), 0), 1);
        motor.set(speed);
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}