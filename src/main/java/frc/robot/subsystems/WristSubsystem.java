package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
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
    private double m_armSetpointDegrees = Constants.kDefaultArmSetpointDegrees;

    private double m_armKp = Constants.kDefaultArmKp;

    //create a motor
    private final WPI_TalonFX m_motor;
 
    //create a joystick
    private final Joystick m_joy; 

    //create a arm sim
    private final SingleJointedArmSim m_armSim; 

    //create a PID controller
    private final PIDController m_controller; 

    //create encoder
    private final Encoder m_encoder = new Encoder(1, 2); 

    //create encoder sim
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder); 

    //constructor for wrist subsystem 
    public WristSubsystem() {
      m_motor = new WPI_TalonFX(0); 
      m_joy = new Joystick(0); 
      m_controller = new PIDController(1, 0, 0); 
      m_armSim = new SingleJointedArmSim(
        Constants.kGearBox, 
        Constants.kGearRatio,
        Constants.kMomentOfInertia,
        Constants.kLength, 
        Constants.kMinPos, 
        Constants.kMaxPos, 
        true
      ); 

      m_motor.setNeutralMode(Constants.kNeutralMode);
      m_motor.setInverted(true); 
      
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

      //goToDefaultSetpoint();
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

      m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

      m_armSim.update(0.020);

      m_encoderSim.setDistance(m_armSim.getAngleRads());
      RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps())
      );

      m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }

  /** Load setpoint and kP from preferences. */
  public void loadPreferences() {
    // Read Preferences for Arm setpoint and kP on entering Teleop
    m_armSetpointDegrees = Preferences.getDouble(Constants.kArmPositionKey, m_armSetpointDegrees);
    if (m_armKp != Preferences.getDouble(Constants.kArmPKey, m_armKp)) {
      m_armKp = Preferences.getDouble(Constants.kArmPKey, m_armKp);
      m_controller.setP(m_armKp);
    }
  }

  /** Run the control loop to reach and maintain the setpoint from the preferences. */
  public void goToDefaultSetpoint() {
    var pidOutput =
        m_controller.calculate(
            m_encoder.getDistance(), Units.degreesToRadians(m_armSetpointDegrees));
    m_motor.setVoltage(pidOutput);
  }

  public void goToSetpoint(double setpoint) {
    var pidOutput =
        m_controller.calculate(
            m_encoder.getDistance(), Units.degreesToRadians(setpoint));
    m_motor.setVoltage(pidOutput);
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
