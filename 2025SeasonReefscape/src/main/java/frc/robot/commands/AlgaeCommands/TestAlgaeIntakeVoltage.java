package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class TestAlgaeIntakeVoltage extends Command {
    private AlgaeSubsystem m_algaeSub;
    private double m_voltage;
    private final String m_stringMessage;
        
    //Constructor for TestAlgaeIntake, also adds requirments so that this is the only command using algaeSub.
    public TestAlgaeIntakeVoltage(AlgaeSubsystem algaeSub, double voltage, String message) {
        m_algaeSub = algaeSub;
        m_voltage = voltage;
        m_stringMessage = message;
        SmartDashboard.putNumber(m_stringMessage, m_voltage);
        addRequirements(m_algaeSub);
    }

    // Called when the command is initially scheduled.
     @Override
     public void initialize() {
        SmartDashboard.putString("Commanding Algae Subsystem", m_stringMessage);
     }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        m_voltage = SmartDashboard.getNumber(m_stringMessage, m_voltage);
        if(Math.abs(m_voltage) > 12) {
            m_voltage = Math.signum(m_voltage) * 12;
            SmartDashboard.putNumber(m_stringMessage, m_voltage);
        }
        m_algaeSub.testIntakeMotorsVoltage(m_voltage);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        SmartDashboard.putString("Commanding Algae Subsystem", "None");
        m_algaeSub.stopIntakeMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return false;
    }
}