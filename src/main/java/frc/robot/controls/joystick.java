package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.WristSubsystem;

public class joystick {
  private Joystick m_joy = new Joystick(0); 
  private WristSubsystem m_wrist; 
  private Trigger m_button1 = new JoystickButton(m_joy, 1); 
  private Trigger m_button2 = new JoystickButton(m_joy, 2); 
  private Trigger m_button3 = new JoystickButton(m_joy, 3); 
  private Trigger m_button4 = new JoystickButton(m_joy, 4); 

  public joystick(WristSubsystem wrist) {
    m_wrist = wrist;
  }
  
  public void configureControls() { 
    m_button1.onTrue(new MoveWrist(m_wrist, 90)); 
    m_button2.onTrue(new MoveWrist(m_wrist, 180)); 
    m_button3.onTrue(new MoveWrist(m_wrist, 270)); 
    m_button4.onTrue(new MoveWrist(m_wrist, 360)); 


  }
  
  public double getJoyValueAxis0(){
    return m_joy.getRawAxis(0); 
  }
}
