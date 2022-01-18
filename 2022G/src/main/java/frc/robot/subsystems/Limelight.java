package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /**
   * Creates a new Limelight.
   */
  
  private static Limelight limelight;
  double[] thorInputs = {};// make sure to go in increasing order, so from 1->100 vs 100->1
  double[] velocityOutputs = {};
  NetworkTable limes = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = limes.getEntry("tx");
  NetworkTableEntry ty = limes.getEntry("ty");
  NetworkTableEntry thor = limes.getEntry("thor");
  NetworkTableEntry tvert = limes.getEntry("tvert");
  NetworkTableEntry ta = limes.getEntry("ta");
    
  public Limelight() {
  }
   
  public static Limelight getInstance() {
    if (limelight == null) {
      limelight = new Limelight();
    }
  return limelight;

  }

  //Tvert is the vertical sidelength of the rough bounding box (0 - 320 pixels)
  public double getTvert(){
    return tvert.getDouble(0.0);
  }
  //Thor is the horizontal sidelength of the rough bounding box (0 - 320 pixels)
  public double getThor(){
    return thor.getDouble(0.0);
  }
  //Tx is the Horizontal Offset From Crosshair To Target
  public double getTx(){
    return tx.getDouble(0.0);
  }
  //Ty is the Vertical Offset From Crosshair To Target
  public double getTy(){
    return ty.getDouble(0.0);
  }
  public double getTa(){
    return ta.getDouble(0.0);
  }
  /*public double getDistance(){
    if(ty.getDouble(0.0)==0) return 0;
    else return (98.25-24)/(Math.tan(Math.toRadians(25+ty.getDouble(0.0))));
  }*/
  public boolean hasTarget(){
    if (limes.getEntry("tv").getDouble(0.0)==1){
      return true;
    }else return false;
  }
  
  public void putSmartDashboard(){
    SmartDashboard.putNumber("Limelight Vertical Error", getTy());
    SmartDashboard.putNumber("Limelight Horizontal Error", getTx());
  }
}