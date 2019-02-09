//Run from the package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.subsystems.dunk.Dunk;
import org.firstinspires.ftc.teamcode.subsystems.dunk.servoArmDunk;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.hang.linearActuator;
import org.firstinspires.ftc.teamcode.subsystems.imu.BoschIMU;
import org.firstinspires.ftc.teamcode.subsystems.imu.IIMU;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.intakeMinerals;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.Flip;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.mineralFlip;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.*;

import org.firstinspires.ftc.teamcode.subsystems.DriveFunctions;

@Autonomous(name="Auto Crater") //Name the program
//@Disabled
public class autoCrater extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    BNO055IMU boschIMU;
    IIMU imu;
    //***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");

        boschIMU = hardwareMap.get(BNO055IMU.class, "boschIMU");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions chasis = new DriveFunctions(DcMotor.ZeroPowerBehavior.BRAKE, leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, boschIMU);

        imu = new BoschIMU(boschIMU);

        //Set the sensor to active mode
        //Set the directions and modes of the motors.
//        chasis.initializeRobotBrake();

        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {

            chasis.rightTurnIMU((float) 0.6, 90);

//            Thread.sleep(1000);
//
//            chasis.leftTurnIMU((float) 0.6, 90);




//            teamMarker.hold();
//            hang.down();
//
//            chasis.leftShiftAutonomous(shiftPower, 150);
//
//            chasis.driveAutonomous(drivePower, 950);

//            chasis.leftTurnIMU(turnPower, 90);
//
////            chasis.leftTurnAutonomous(turnPower, 950);
//
//            chasis.driveAutonomous(drivePower, 1350);
//
////            chasis.leftTurnAutonomous(turnPower, 470);
//
//            chasis.leftTurnIMU(turnPower, 45);
//
//            chasis.driveAutonomous(drivePower, 2650);
//
////            chasis.leftTurnAutonomous(turnPower, 500);
//
//            chasis.leftTurnIMU(turnPower, 45);
//
//            teamMarker.drop();
//
//            Thread.sleep(500);
//
////            chasis.leftTurnAutonomous(turnPower, 1600);
//
//            chasis.leftTurnIMU(turnPower, 135);
//
//            //Go to Our crater
//            chasis.driveAutonomous(drivePower, 1500);
//
////            chasis.rightTurnAutonomous(turnPower, 100);
//
//            chasis.rightTurnIMU(turnPower, 20);
//
//            chasis.driveAutonomous(drivePower, 1700);
//
//            dunk.dunkNoPause();
//            mineralSpool.setPower(0.5);
//            Thread.sleep(1000);
//            mineralSpool.setPower(0.0);
//            flip.up();
//            mineralSpool.setPower(0.5);
//            Thread.sleep(2000);
//            mineralSpool.setPower(0.0);
//            dunk.down();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
            //Break the loop after one run
            break;
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program