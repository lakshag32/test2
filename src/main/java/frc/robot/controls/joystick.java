package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.WristSubsystem;

public class joystick {
  private Joystick m_joy = new Joystick(0); 
  private WristSubsystem m_wrist; 
  private Trigger m_button = new JoystickButton(m_joy, 1); 

  public joystick(WristSubsystem wrist) {
    m_wrist = wrist;
  }
  
  public void configureControls() { 
    m_button.onTrue(new InstantCommand(() -> m_wrist.reachSetpoint())); 
  }
  
  public double getJoyValueAxis0(){
    return m_joy.getRawAxis(0); 
  }
}
