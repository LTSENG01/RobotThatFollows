/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ltseng01.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

    private static WPI_TalonSRX leftFront;
    private static WPI_TalonSRX leftBack;
    private static WPI_TalonSRX rightFront;
    private static WPI_TalonSRX rightBack;

    private static SpeedControllerGroup leftDrive;
    private static SpeedControllerGroup rightDrive;

    private static DifferentialDrive differentialDrive;

    private static Encoder leftEncoder;
    private static Encoder rightEncoder;

    private static XboxController xboxController;

    @Override
    public void robotInit() {

        xboxController = new XboxController(0);

        leftFront = new WPI_TalonSRX(1);
        leftBack = new WPI_TalonSRX(2);
        rightFront = new WPI_TalonSRX(3);
        rightBack = new WPI_TalonSRX(4);

        leftDrive = new SpeedControllerGroup(leftFront, leftBack);
        rightDrive = new SpeedControllerGroup(rightFront, rightBack);

        differentialDrive = new DifferentialDrive(leftDrive, rightDrive);

        leftEncoder = new Encoder(1, 2);
        rightEncoder = new Encoder(3, 4, true);

        resetEncoders();

    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Left Encoder Ticks", leftEncoder.get());
        SmartDashboard.putNumber("Right Encoder Ticks", rightEncoder.get());
        SmartDashboard.putNumber("Left Encoder Velocity", leftEncoder.getRate());
        SmartDashboard.putNumber("Right Encoder Velocity", rightEncoder.getRate());

    }

    @Override
    public void autonomousInit() {


    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {

        // Manual Drive for Encoder Data
        // TODO: Get Wheel Diameters --> Velocity calculation

        differentialDrive.arcadeDrive(-xboxController.getY(GenericHID.Hand.kLeft),
                xboxController.getX(GenericHID.Hand.kRight));

    }

    @Override
    public void testPeriodic() {

    }

    private static void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

}
