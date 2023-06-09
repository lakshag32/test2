package frc.robot.subsystems;
import frc.robot.Constants; 
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_motor;
  private final PIDController m_pid;

  // unit tests want access to the DutyCycleEncoder and DutyCycleEncoderSim
  protected final DutyCycleEncoder m_absEncoder;
  protected DutyCycleEncoderSim m_absEncoderSim;

  private double m_pidPower = 0;
  private double m_power = 0;
  private double m_lastPos = 0;

  /** Physics Simulator for the wrist. takes in a motor voltage and calculates how much the arm will move. */
  private final SingleJointedArmSim m_armSim;

  public WristSubsystem(ShuffleboardTab wristTab) {
    // configure the motor.
    m_motor = new WPI_TalonFX(3); 
    m_motor.setNeutralMode(Constants.kNeutralMode);
    m_motor.setInverted(true); 
    m_motor.enableVoltageCompensation(true);

    // config deadband to be less, may be powering at small values to keep it up
    m_motor.configNeutralDeadband(0.005);

    // configure the encoder
    m_absEncoder = new DutyCycleEncoder(Constants.kAbsEncoderPort);
    // Cleaner encoder implementation
    // offset to zero (arm horizontal)
    m_absEncoder.setPositionOffset(Constants.kEncoderOffset);
    // scale to radians and invert direction
    m_absEncoder.setDistancePerRotation(-2.0 * Math.PI);

    // make the PID controller
    m_pid = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    // set the PID controller's tolerance
    m_pid.setTolerance(Constants.kTolerance);

    // go to the initial position
    setSetpoint(Constants.kStowPos);

    if (Constants.kUseTelemetry) {
      setupShuffleboardTab(wristTab);
    }

    m_armSim = new SingleJointedArmSim(
      Constants.kGearBox,
      0.1,
      0.1,
      0.1,
      0.1,
      0.1, 
      true
    ); 

      // make the encoder simulator
      // this allows us to set the encoder during simulations...
      m_absEncoderSim = new DutyCycleEncoderSim(m_absEncoder);
    }


  /**
   * Set the Wrist's desired position.
   * @param setpoint the desired arm position (in rotations)
   */
  public void setSetpoint(double setpoint) {
    // set the PID integration error to zero.
    m_pid.reset();
    // set the PID desired position
    m_pid.setSetpoint(setpoint);
  }

  @Override
  public void periodic() {
    // obtain the wrist position
    double position = getAbsEncoderPos();

    // calculate the PID power level
    // for safety, clamp the setpoint to prevent tuning with SmartDashboard/Shuffleboard from commanding out of range
    // This method continually changes the setpoint.
    m_pidPower = m_pid.calculate(position, MathUtil.clamp(m_pid.getSetpoint(), Constants.kMinPos, Constants.kMaxPos));
    
    // calculate the value of kGravityCompensation
    double feedforwardPower = Constants.kGravityCompensation * Math.cos(position);

    // set the motor power
    setMotorPower(m_pidPower + feedforwardPower);
  }

  /**
   * Whether the wrist has reached its commanded position.
   * @returns true when position has been reached
   */
  public boolean reachedSetpoint() {
    return m_pid.atSetpoint();
  }

  /**
   * Sets the motor power, clamping it and ensuring it will not activate below/above the min/max positions
   */
  private void setMotorPower(double power) {

    m_power = MathUtil.clamp(power, -Constants.kMotorPowerClamp, Constants.kMotorPowerClamp);
    
    double pos = getAbsEncoderPos();

    // double safety check incase encoder outputs weird values again
    if (pos <= Constants.kMinPos && m_power < 0) {
      m_power = 0;
    }
    if (pos >= Constants.kMaxPos && m_power > 0) {
      m_power = 0;
    }
    
    m_motor.set(m_power);
  }

  /**
   * @return the absolute encoder position in rotations, zero being facing forward
   */
  public double getAbsEncoderPos() {
    double pos = m_absEncoder.getDistance();
    if (pos > Constants.kMaxPos || pos < Constants.kMinPos) {
      pos = m_lastPos;
    }
    m_lastPos = pos;
    return pos;
  }

  private void setupShuffleboardTab(ShuffleboardTab wristTab) {
    wristTab.addNumber("Wrist Position", () -> getAbsEncoderPos());
    wristTab.add("wrist PID", m_pid);
    wristTab.addNumber("wrist power final", () -> m_power);
    wristTab.addNumber("Wrist PID output", () -> m_pidPower);
    wristTab.addNumber("Wrist Error", () -> m_pid.getSetpoint() - getAbsEncoderPos());
    wristTab.addBoolean("At Setpoint", () -> reachedSetpoint());
    wristTab.addNumber("Current Draw", () -> m_motor.getSupplyCurrent());
  }

  @Override
  public void simulationPeriodic() {
    // First, we set our "inputs" (the motor voltage)    
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // update the physics simulation, telling it how much time has passed, and it will calculate how much the wrist has moved
    m_armSim.update(0.02);

    // set the distance to what we get from the sim
    m_absEncoderSim.setDistance(m_armSim.getAngleRads());

    // calculate the battery voltage based on the theoretical drawn amps.
    // TODO: this is the wrong way to calculate battery voltage; it needs the sum of all currents.
    // RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // update the drawing of the robot
    
    //DrawMechanism.getInstance().setWristAngle(m_armSim.getAngleRads());
  }

  /**
   * Deallocate resources.
   * <p>
   * Test routines need to deallocate simulation resources.
   */
  public void close() {
    m_motor.close();
    m_absEncoder.close();
  }
}
