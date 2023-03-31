package frc.robot.subsystems;


import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
//import frc.robot.Constants.VisionConstants;

     


/**
 *
 */
public class Vision extends SubsystemBase {
    static double distanceFromLimelightToGoalInches;
    static double angleToGoalRadians;
    static double angleToGoalDegrees;
    private static double targetOffsetAngle_Vertical;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
//private LimeLight limelight;
    private static NetworkTable table;
    private static PhotonCamera camera;


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    /**
    * Ca
    */
    public Vision() {
         
        table = NetworkTableInstance.getDefault().getTable("limelight");
        
        //29 1/4 at init
  }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Target Found", isTargetFound());
        SmartDashboard.putNumber("Horizontal Distance", getDistance());
        SmartDashboard.putNumber("Tx", getTx());
    }
    public static double getDistance(){
        // calculate horizontal distance
        targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0.0);
        angleToGoalDegrees = VisionConstants.kLimelightMountAngleDegrees + targetOffsetAngle_Vertical;
        angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        distanceFromLimelightToGoalInches = (VisionConstants.kGoalHeightInches - VisionConstants.kLimelightLensHeightInches)
            / Math.tan(angleToGoalRadians); 
        return distanceFromLimelightToGoalInches;
      }
      
    
      public void setLEDMode(double mode){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
      }
    
      public boolean isTargetFound(){
        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1){
         //public double saveDistance = getDistance();
          return true;
        }
        return false;
      
      } 
    
    
      public static double getTx() {
            return table.getEntry("tx").getDouble(0.0);
        }
    
    
      public static double getTy() {
            return table.getEntry("ty").getDouble(0.0);
        }
}
 
 


     
    

//     @Override
//     public void periodic() {
//         // This method will be called once per scheduler run

//     }

//     @Override
//     public void simulationPeriodic() {
//         // This method will be called once per scheduler run when in simulation

//     }

//     // Put methods for controlling this subsystem
//     // here. Call these from Commands.

// }

