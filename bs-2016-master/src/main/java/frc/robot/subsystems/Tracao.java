/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.RobotMap;
import frc.robot.commands.Drive;


/**
 * Add your docs here.
 */


public class Tracao extends PIDSubsystem {
  /**
   * Add your docs here.
   */

   Spark tFrontalEsquerda = null;
   Spark tTraseiraEsquerda = null;
   Spark tFrontalDireita = null;
   Spark tTraseiraDireita = null;

   DifferentialDrive drive = null;

   AHRS navX = RobotMap.navX;

   double pidOutput;
  public Tracao() {
    // Intert a subsystem name and PID values here
    super("Tracao", 1, 0, 0);

    tFrontalEsquerda = new Spark(RobotMap.TRACAO_FRONTAL_ESQUERDA);
    tTraseiraEsquerda = new Spark(RobotMap.TRACAO_TRASEIRA_ESQUERDA);
    tFrontalDireita = new Spark(RobotMap.TRACAO_FRONTAL_DIREITA);
    tTraseiraDireita = new Spark(RobotMap.TRACAO_TRASEIRA_DIREITA);

    SpeedControllerGroup tDireita = new SpeedControllerGroup(tTraseiraDireita, tFrontalDireita);
    SpeedControllerGroup tEsquerda = new SpeedControllerGroup(tTraseiraEsquerda, tFrontalEsquerda);

    drive = new DifferentialDrive(tEsquerda, tDireita);

    
    setInputRange(-180f,180f);
    setOutputRange(-0.5f, 0.5f);
    setAbsoluteTolerance(3);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Drive());
  }

  public void arcadeDrive(double mover, double girar){
    drive.arcadeDrive(mover, girar);
  }
  

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return (int) navX.getYaw();
  }

  @Override
  protected void usePIDOutput(double output) {
    this.pidOutput = output;
  }
}
