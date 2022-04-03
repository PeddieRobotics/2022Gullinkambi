package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LightCommands.FlashLights;

public class Lights extends SubsystemBase{

    private static Lights lights;
    private Solenoid strip1, strip2;
    private boolean thirtySecondFlashComplete, fifteenSecondFlashComplete;

    public Lights() {
        strip1 = new Solenoid(21, PneumaticsModuleType.REVPH, 14);
        strip2 = new Solenoid(21, PneumaticsModuleType.REVPH, 15);
        thirtySecondFlashComplete = false;
        fifteenSecondFlashComplete = false;
    }

    public static Lights getInstance(){
        if(lights == null){
          lights = new Lights();
        }
        return lights;
      }

    public boolean isOn(){
      return strip1.get() && strip2.get();
    }
  
    public void on(){
        strip1.set(true);
        strip2.set(true);
    }  

    public void off(){
        strip1.set(false);
        strip2.set(false);
    }  

    public void resetLightBooleans(){
      thirtySecondFlashComplete = false;
      fifteenSecondFlashComplete = false;
    }

    public void timeCheck(){
      if (!thirtySecondFlashComplete && Timer.getMatchTime() <= 30){
        CommandScheduler.getInstance().schedule(new FlashLights(15, 0.5));
        thirtySecondFlashComplete = true;
      }

      if (!fifteenSecondFlashComplete && Timer.getMatchTime() <= 15){
        CommandScheduler.getInstance().schedule(new FlashLights(15, 0.1));
        fifteenSecondFlashComplete = true;
      }
    }

    @Override
    public void periodic() {
      if(!DriverStation.isAutonomous() && Timer.getMatchTime() > 0){
        timeCheck();
      }
    }
}