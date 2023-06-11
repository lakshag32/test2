package frc.robot.subsystems;
import frc.robot.Constants; 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class WristSubsystem extends SubsystemBase{
    private final WPI_TalonFX m_motor;
    //private final PIDController m_pid;
 
    //create a joystick
    private final Joystick m_joy; 

    public WristSubsystem() {
      // configure the motor.
      m_motor = new WPI_TalonFX(0); 
      m_joy = new Joystick(0); 
      m_motor.setNeutralMode(Constants.kNeutralMode);
      m_motor.setInverted(true); 
    }
  
    public double getEncoderDist(){
      return m_motor.getSelectedSensorPosition(); 
    }

    public double getEncoderVelocity(){
      return m_motor.getSelectedSensorVelocity(); 
    }

    public void spinMotor(double power){
      m_motor.set(power); 
    } 

    public double returnJoyPos(double pos){
      return pos; 
    }

    @Override
    public void simulationPeriodic() {
      SmartDashboard.putNumber("Joy Output:", m_joy.getRawAxis(0));
    }
}