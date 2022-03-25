package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;
import frc.robot.utils.Constants.OIConfig;

public class Limelight extends SubsystemBase {
  /**
   * Creates a new Limelight.
   */

  private static Limelight limelight;

  private PIDController limelightPIDController;

  private double ff;
  
  private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = limelightTable.getEntry("tx");
  private NetworkTableEntry ty = limelightTable.getEntry("ty");
  private NetworkTableEntry thor = limelightTable.getEntry("thor");
  private NetworkTableEntry tvert = limelightTable.getEntry("tvert");
  private NetworkTableEntry ta = limelightTable.getEntry("ta");
  private NetworkTableEntry tv = limelightTable.getEntry("tv");
  
  private RollingAverage txAverage = new RollingAverage();
  private RollingAverage tyAverage = new RollingAverage();

  private static UpdateLogs updateLogs = UpdateLogs.getInstance();

    
  public Limelight() {
    limelightPIDController = new PIDController(Constants.LL_P, Constants.LL_I, Constants.LL_D);
    ff = Constants.LL_FF;
  }

  public static Limelight getInstance() {
    if (limelight == null) {
      limelight = new Limelight();
    }
    return limelight;

  }

  @Override
  public void periodic() {
    updateRollingAverages();
    if(Constants.USE_LOGGING){
      updateLogs.updateLimelightLogData();
    }

  }

  public boolean isActive(){
    return limelightTable.getKeys().toArray().length > 0;
  }

  public PIDController getPIDController(){
    return limelightPIDController;
  }

  public double getFF(){
    return ff;
  }

  public void reloadLimelightTable(){
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  // Tv is whether the limelight has a valid target
  // 1 is true, 0 is false
  public double getTv(){
    return tv.getDouble(0.0);
  }

  // Tvert is the vertical sidelength of the rough bounding box (0 - 320 pixels)
  public double getTvert() {
    return tvert.getDouble(0.0);
  }

  // Thor is the horizontal sidelength of the rough bounding box (0 - 320 pixels)
  public double getThor() {
    return thor.getDouble(0.0);
  }

  // Tx is the Horizontal Offset From Crosshair To Target
  public double getTx() {
    return tx.getDouble(0.0);
  }

  // Ty is the Vertical Offset From Crosshair To Target
  public double getTy() {
    return ty.getDouble(0.0);
  }

  public double getTa() {
    return ta.getDouble(0.0);
  }

  public double getTxAverage(){
    return txAverage.getAverage();
  }

  public double getTyAverage(){
    return tyAverage.getAverage();
  }
  
  public double getDistance(){
    if(ty.getDouble(0.0)==0){
      return 0;
    }
     else return (Constants.TARGET_HEIGHT-Constants.LL_HEIGHT)/
     (Math.tan(Math.toRadians(Constants.LL_ANGLE+Constants.LL_PANNING_ANGLE+ty.getDouble(0.0))));
  }
  
  public boolean hasTarget(){
    if (tv.getDouble(0.0)==1){
      return true;
    } else
      return false;
  }

  public void updateRollingAverages(){
    if(hasTarget()){
      txAverage.add(getTx());
      tyAverage.add(getTy());
    }
  }

  public void updateLimelightInfoOnDashboard(){
    if(Constants.OI_CONFIG != OIConfig.COMPETITION){
      SmartDashboard.putNumber("LL vt error", getTy());
      SmartDashboard.putNumber("LL hz error", getTx());
    }
    
    SmartDashboard.putNumber("LL dist", getDistance());
    SmartDashboard.putBoolean("LL has target", hasTarget());
  }

  public void updateLimelightFromDashboard(){
    limelightPIDController.setP(SmartDashboard.getNumber("LL P", Constants.LL_P));
    limelightPIDController.setI(SmartDashboard.getNumber("LL I", Constants.LL_I));
    limelightPIDController.setD(SmartDashboard.getNumber("LL D", Constants.LL_D));
    limelight.setFF(SmartDashboard.getNumber("LL FF", Constants.LL_FF));
  }

  public void putSmartDashboardOverrides(){
    SmartDashboard.putData("Reload LL Network Table", new InstantCommand(this::reloadLimelightTable));
    SmartDashboard.putNumber("LL P", Constants.LL_P);
    SmartDashboard.putNumber("LL I", Constants.LL_I);
    SmartDashboard.putNumber("LL D", Constants.LL_D);
    SmartDashboard.putNumber("LL FF", Constants.LL_FF);
    SmartDashboard.putNumber("LL ANGLE BOUND", Constants.LL_ANGLE_BOUND);
  }

  public void setFF(double feedforward){
    ff = feedforward;
  }
}