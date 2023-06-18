package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase{

    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d m_armTower =
        m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
    private final MechanismLigament2d m_arm; 

    private double m_pidOutput; 

    //create a motor
    private final PWMSparkMax m_motor; 

    //create a joystick

    //create a arm sim
    private final SingleJointedArmSim m_armSim; 

    //create a PID controller
    private final PIDController m_controller; 

    private final Encoder m_encoder = new Encoder(0,1); 

    //create encoder sim
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder); 

    //constructor for wrist subsystem 
    public WristSubsystem() {
      m_motor = new PWMSparkMax(0); 
      m_controller = new PIDController(1, 0, 0); 
      m_armSim =
      new SingleJointedArmSim(
          Constants.kGearBox,
          200.0,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30), 8.0),
          Units.inchesToMeters(30),
          Units.degreesToRadians(-75),
          Units.degreesToRadians(255),
          true,
          VecBuilder.fill(Constants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
          );

      //connect arm to ArmPivot
      m_arm = m_armPivot.append(
        new MechanismLigament2d(
          "Arm",
          30,
          Units.radiansToDegrees(m_armSim.getAngleRads()),
          6,
          new Color8Bit(Color.kYellow))
      );

      m_encoder.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);

      // Put Mechanism 2d to SmartDashboard
      SmartDashboard.putData("Arm Sim", m_mech2d);

      m_armTower.setColor(new Color8Bit(Color.kBlue));

    }

    public void spinMotor(double power){
      m_motor.set(power); 
    } 

    public double returnJoyPos(double pos){
      return pos; 
    }

    @Override
    public void simulationPeriodic() {

      m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());
      m_armSim.update(0.020);

      m_encoderSim.setDistance(m_armSim.getAngleRads());
      SmartDashboard.putNumber("Encoder Val output:", m_encoderSim.getDistance());
      SmartDashboard.putNumber("PID OUTput: ", m_pidOutput); 
      RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps())
      );

      m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }
  
  public void goToSetpoint(double setpoint) {
    m_controller.setSetpoint(setpoint);
    m_pidOutput = m_controller.calculate(m_encoder.getDistance());
    m_pidOutput = MathUtil.clamp(m_pidOutput, -1, 1); 
    m_motor.set(m_pidOutput);
  }

  public boolean atSetpoint(double setpoint){
    return m_controller.atSetpoint(); 
  }

  public void stop() {
    m_motor.set(0.0);
  }

  public void close() {
    m_motor.close();
    m_encoder.close();
    m_mech2d.close();
    m_armPivot.close();
    m_controller.close();
    m_arm.close();
  }

}
