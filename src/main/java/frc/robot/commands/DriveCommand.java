// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.*;
import java.util.function.DoubleSupplier;
import frc.robot.OI;
import frc.robot.OIReporters;

/**
 * Creates teleop drivetrain drive command
  * @author 23BBrenner
 */
public class DriveCommand extends CommandBase {

    // DriveCommand Variable Declarations 

    // DriveCommand Constructors

    private DriveTrain m_drivetrain; // = DriveTrain.getInstance();
    // private OI m_oi;
    private XboxController m_driveController;
    private int m_driverControlsChoice;
    private int m_driveTypeChoice;
    private int m_controllerScalingChoice;
    public double speed;
    public double rotation;
       
    public DriveCommand(DriveTrain subsystem, XboxController controller) {
        m_drivetrain = subsystem;
        m_driveController = controller; 
        // Ensures that two commands that need the same subsystem dont mess each other up. 
        addRequirements (m_drivetrain);  
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // speed = 0;
        // rotation = 0;

        m_driverControlsChoice = OI.getInstance().getDriverControlsChooser();
        m_driveTypeChoice = OI.getInstance().getDriveTypeChooser();
        m_controllerScalingChoice = OI.getInstance().getControllerScalingChooser();

        
        driverControls(m_driverControlsChoice);
        
        controllerScaling(speed, rotation, m_controllerScalingChoice);
        driveType(speed, rotation, m_driveTypeChoice);

        double leftSpeed = speed + rotation;
        double rightSpeed = speed - rotation;

        double max = Math.max(leftSpeed, rightSpeed);

        if(max > 1) {
			leftSpeed /= max;
			rightSpeed /= max;
		}

        m_drivetrain.setWheelSpeeds(leftSpeed, rightSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    }

    public void driverControls(int choice) {
        speed = 0;
        rotation = 0;

            if (choice != 0){ //accelerate with trigger controls 
                speed = ((m_driveController.getLeftTriggerAxis() - m_driveController.getRightTriggerAxis()) * DriveConstants.kSpeedOutputModifier);

                OIReporters.driveControllerMode = "Trigger Accel";
                OIReporters.tAccelSpeed = speed;
                return;

             } //else { //default left stick controls
                speed = (m_driveController.getLeftY() * DriveConstants.kSpeedOutputModifier);
                
                OIReporters.driveControllerMode = "Left Stick";
                OIReporters.lStickSpeed = speed;

            rotation = (m_driveController.getLeftX() * DriveConstants.kRotationOutputModifier);

            //}
        }

    public void controllerScaling(double speed, double rotation, int choice){
        OIReporters.originalSpeed = speed;
        OIReporters.originalRotation = rotation;

        if (choice == 1){ //linear scaling
            this.speed = speed; 
            this.rotation = rotation;

            OIReporters.scalingMode = "Linear";
            OIReporters.linearScaled = ("Speed: " + speed + "& Rotation: " + rotation);
            return;
        }
        if (choice == 2) { //squared scaling
            speed = Math.copySign(speed * speed, speed);
                
            OIReporters.scalingMode = "Squared";
            OIReporters.squaredScaled = ("Speed: " + speed + "& Rotation: " + rotation);
            return;
        }
        if (choice == 3) { //non polynomic (fancy)
            speed = speed * 0.5 + Math.pow(3,(speed * 0.5));

            OIReporters.scalingMode = "Fancy";
            OIReporters.fancyScaled = ("Speed: " + speed + "& Rotation: " + rotation);
           return;
        }
       // else { //cubic scaling
            speed = speed * speed * speed;
            rotation = rotation * rotation * rotation;

            OIReporters.scalingMode = "Cubic";
            OIReporters.cubicScaled = ("Speed: " + speed + "& Rotation: " + rotation);
       // }
    }

    public void driveType(double speed, double rotation, int choice){
        if (choice == 1){ //regular curvature
            m_drivetrain.curvatureDrive(speed, rotation, false);

            OIReporters.driveType = "Curvature";
            OIReporters.semiCurvature = false;
           return;
        }
        if (choice == 2){ //arcade
            m_drivetrain.arcadeDrive(speed, rotation);

            OIReporters.driveType = "Arcade";
            return;
        }
        //else { //semi-curvature
            m_drivetrain.curvatureDrive(speed,rotation,true);
            
            OIReporters.driveType = "Semi-Curvature";
            OIReporters.semiCurvature = true;
       // }
    }

    

    
}