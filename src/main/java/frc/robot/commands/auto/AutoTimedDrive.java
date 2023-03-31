package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import edu.wpi.first.wpilibj.Timer;


public class AutoTimedDrive extends CommandBase{
    
    private DriveTrain m_driveTrain;
    private double startTime;
    private Pose2d initPose;


public AutoTimedDrive(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    addRequirements(driveTrain);

}

@Override
public void initialize() {
   startTime = Timer.getFPGATimestamp();
   initPose = this.m_driveTrain.getPose();
}

@Override
  public void execute() {
    double time = Timer.getFPGATimestamp();
    System.out.println(time - startTime);

    if (time - startTime < 5) {
        m_driveTrain.arcadeDrive(.4 * -1, 0);
    } else {
        m_driveTrain.arcadeDrive(0 * 0, 0 );
    }
  }

// public double getDistance() {
//     return new Transform2d(initPose, m_driveTrain.getPose()).getTranslation().getX();
// }

// private double getDisplacement() {
//     return this.initPose.getTranslation().getDistance(this.m_driveTrain.getPose().getTranslation());
//   }

@Override
public boolean isFinished() {
    if (Timer.getFPGATimestamp()- startTime > 5){
        return true;
    } 
    return false; 
}

@Override 
public void end(boolean interrupt) {
    m_driveTrain.arcadeDrive(0, 0);
}
}
