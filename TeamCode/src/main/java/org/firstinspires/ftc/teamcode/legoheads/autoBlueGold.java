//Run from the package
package org.firstinspires.ftc.teamcode.legoheads;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Auto Blue Gold") //Name the program
public class autoBlueGold extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define glyph motors
    DcMotor mineralSpool;
    DcMotor spinner;
    DcMotor mineralFlipper;
    DcMotor hanger;

    CRServo pin;
    Servo markerDropper;
    Servo mineralFlipInit;

    //Define drive powers to avoid magic numbers
    float drivePower = (float) -0.3;
    float shiftPower = (float) 0.3;
    float turnPower = (float) 0.3;

//***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        //Get references to the DC Motors from the hardware map
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");

        mineralSpool = hardwareMap.dcMotor.get("mineralSpool");
        spinner = hardwareMap.dcMotor.get("spinner");
        mineralFlipper = hardwareMap.dcMotor.get("mineralFlipper");
        hanger = hardwareMap.dcMotor.get("hanger");

        pin = hardwareMap.crservo.get("pin");
        markerDropper = hardwareMap.servo.get("markerDropper");
        mineralFlipInit = hardwareMap.servo.get("mineralFlipInit");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, mineralSpool, spinner, mineralFlipper, hanger, pin, markerDropper, mineralFlipInit);

        //Set the sensor to active mode
        //Set the directions and modes of the motors.
        functions.initializeMotorsAndSensors();

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            markerDropper.setPosition(0.4);

            functions.dropDown();

            spinner.setPower(0.3);

            functions.setDriveMotorPowers(drivePower, drivePower, drivePower, drivePower);
            Thread.sleep(2500);
            functions.setDriveMotorPowers((float) 0.0, (float)0.0, (float)0.0, (float)0.0);

//            functions.knockCube(colorSensor);

            spinner.setPower(-0.3);

            Thread.sleep(1000);

            functions.setDriveMotorPowers(-drivePower, -drivePower, drivePower, drivePower);
            Thread.sleep(4000);
            functions.setDriveMotorPowers((float) 0.0, (float)0.0, (float)0.0, (float)0.0);

            markerDropper.setPosition(-0.3);

            Thread.sleep(1000);

            spinner.setPower(0.3);

            functions.setDriveMotorPowers(drivePower, drivePower, drivePower, drivePower);
            Thread.sleep(7000);
            functions.setDriveMotorPowers((float) 0.0, (float)0.0, (float)0.0, (float)0.0);

            spinner.setPower(0.0);

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
            //Break the loop after one run
            break;
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program