package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LightCommands.FlashLights;

public class Lights extends SubsystemBase{

    private static Lights lights;
    private Solenoid strip;
    private boolean lightsFlashing;

    public Lights() {
        strip = new Solenoid(21,PneumaticsModuleType.REVPH, 15);
        lightsFlashing = false;
    }

    public static Lights getInstance(){
        if(lights == null){
          lights = new Lights();
        }
        return lights;
      }

    public void setLightsFlashing(boolean areLightsFlashing){
      lightsFlashing = areLightsFlashing;
    }

    public boolean isOn(){
      return strip.get();
    }
  
    public void on(){
        strip.set(true);
    }  

    public void off(){
        strip.set(false);
    }  

    public void timeCheck(){
      if (!lightsFlashing && Timer.getMatchTime() <= 30){
        CommandScheduler.getInstance().schedule(new FlashLights(15));
      }
    }
    @Override
    public void periodic() {
      //timeCheck();
    }
}