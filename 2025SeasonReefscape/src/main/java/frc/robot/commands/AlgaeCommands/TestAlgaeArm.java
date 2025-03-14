package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class TestAlgaeArm extends Command {
    private AlgaeSubsystem m_algaeSub;
    private double m_speed;
    private final String m_stringMessage;
        
    //Constructor for TestAlgaeArm, also adds requirments so that this is the only command using algaeSub.
    public TestAlgaeArm(AlgaeSubsystem algaeSub, double speed, String message) {
        m_algaeSub = algaeSub;
        m_speed = speed;
        m_stringMessage = message;
        SmartDashboard.putNumber(m_stringMessage, m_speed);
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
        m_speed = SmartDashboard.getNumber(m_stringMessage, m_speed);
        if(Math.abs(m_speed) > 1) {
            m_speed = Math.signum(m_speed);
            SmartDashboard.putNumber(m_stringMessage, m_speed);
        }
        m_algaeSub.testArmMotors(m_speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        SmartDashboard.putString("Commanding Algae Subsystem", "None");
        m_algaeSub.stopArmMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return false;
    }
}