package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tower;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMap;

public class IndexCargo extends CommandBase{
    private Tower tower;
    private Hopper hopper;
    private Intake intake;
    private Flywheel flywheel;

    public IndexCargo() {
        hopper = Hopper.getInstance();
        tower = Tower.getInstance();
        intake = Intake.getInstance();
        flywheel = Flywheel.getInstance();

        addRequirements(hopper);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        //hopper 
        if (intake.isIntaking()){
            hopper.runHopper(-0.2, 0.2, -0.2);
        } else if (flywheel.getFlywheelSetpoint() == 200) {
            hopper.runHopper(-0.2, 0.2, -0.2);
          } 
        else {
            hopper.stopHopper();
          }
      
          //tower and roller 

           if ((tower.sensesBallMiddle() || tower.sensesBallTop()) && tower.sensesBallBottom()) {
            //stop the belts
            tower.stopTower();
          } else if (tower.sensesBallTop() || tower.sensesBallMiddle()){
            //stop the upper belt, run the lower belt
            tower.runTower(0.0, 0.2);
          } else {
            //run both the belts
            tower.runTower(0.5, 0.5);
          }
        }
      
        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
          tower.stopTower();
          hopper.stopHopper();
        }
      
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
          return false;
        }
    
    }