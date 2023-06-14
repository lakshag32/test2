package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.WristSubsystem;

public class controller {
  private Joystick m_joy = new Joystick(0); 
  private WristSubsystem m_wrist; 
  public controller(WristSubsystem wrist) {
    m_wrist = wrist;
  }
  
  public void configureControls() { 
    if(m_joy.getTrigger()){
      new InstantCommand(()-> m_wrist.reachSetpoint()); 
    }
    else{
      ;
    }
  }
  
  public double getJoyValueAxis0(){
    return m_joy.getRawAxis(0); 
  }
}
