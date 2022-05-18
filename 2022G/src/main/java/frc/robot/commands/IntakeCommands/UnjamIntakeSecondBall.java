package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.utils.Constants;

public class UnjamIntakeSecondBall extends CommandBase {
    private Intake m_intake;
    private Hopper m_hopper;
    private Flywheel m_flywheel;
    private boolean unjamOneBall;
    private double initialTime;
    public boolean armed, armedAgain;
    private double speed;
    private int state;

    public UnjamIntakeSecondBall(double speed) {
        m_intake = Intake.getInstance();
        m_hopper = Hopper.getInstance();
        m_flywheel = Flywheel.getInstance();
        addRequirements(m_intake, m_hopper, m_flywheel);
        this.speed = speed;
        
    }

    @Override
    public void initialize() {
        m_intake.setIntakeSolenoid(true);
        m_flywheel.runFlywheelSetpoint(0);
        m_intake.reverseIntake(speed);
        m_hopper.reverseHopper(Constants.HOPPER_INDEX_POWER);
        if(m_hopper.sensesBallBottomFiltered() && m_hopper.sensesBallTopFiltered()){
            state = 1;
        } else{
            state = 0;
        }


    }    

    @Override

    public void execute() {
        if(state == 1){
            if(!m_hopper.sensesBallBottomFiltered()){
                m_hopper.stopHopper();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_hopper.stopHopper();
        m_intake.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
}
}