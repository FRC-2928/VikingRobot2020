package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
/**
 * Feedersubsystem is responsible for feeding balls into the shooter
 * We'll use sensors to keep track of ball positions to hold some in the tower
 * We can't have too many in the hopper or they'll jam
*/

public class FeederSubsystem extends SubsystemBase {
  private WPI_VictorSPX m_hopperMotor;
  private WPI_VictorSPX m_towerMotor;
  public FeederSubsystem() {
   m_hopperMotor = new WPI_VictorSPX(RobotMap.kHopperVictorSPX);
   m_towerMotor = new WPI_VictorSPX(RobotMap.kTowerVictorSPX);

   for(WPI_VictorSPX feederMotors: new WPI_VictorSPX[]{m_hopperMotor, m_towerMotor}){
   feederMotors.configFactoryDefault();

   feederMotors.configVoltageCompSaturation(12);
   feederMotors.enableVoltageCompensation(true);
   feederMotors.configNominalOutputForward(0);
   feederMotors.configNominalOutputReverse(0);
   feederMotors.configNeutralDeadband(0.01);

   feederMotors.setNeutralMode(NeutralMode.Coast);
   }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
