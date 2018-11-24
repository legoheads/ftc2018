package org.firstinspires.ftc.teamcode.legoheads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="wires test ") //Name the class
public class wiresTest extends LinearOpMode
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
    float power = (float) 0.5;

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
            if (gamepad1.y)
            {
                functions.oneMotorEncoder(leftMotorFront, power, 300);
                telemetry.addData("motor forwards", leftMotorFront);
            }
            if (gamepad1.b)
            {
                functions.oneMotorEncoder(rightMotorFront, power, 300);
                telemetry.addData("motor forwards", rightMotorFront);
            }
            if (gamepad1.a)
            {
                functions.oneMotorEncoder(leftMotorBack, power, 300);
                telemetry.addData("motor forwards", leftMotorBack);
            }
            if (gamepad1.x)
            {
                functions.oneMotorEncoder(rightMotorBack, power, 300);
                telemetry.addData("motor forwards", rightMotorBack);
            }

            if (gamepad1.dpad_up)
            {
                functions.oneMotorEncoder(leftMotorFront, -power, -300);
                telemetry.addData("motor backwards", leftMotorFront);
            }
            if (gamepad1.dpad_right)
            {
                functions.oneMotorEncoder(rightMotorFront, -power, -300);
                telemetry.addData("motor backwards", rightMotorFront);
            }
            if (gamepad1.dpad_down)
            {
                functions.oneMotorEncoder(leftMotorBack, -power, -300);
                telemetry.addData("motor backwards", leftMotorBack);
            }
            if (gamepad1.dpad_left)
            {
                functions.oneMotorEncoder(rightMotorBack, -power, -300);
                telemetry.addData("motor backwards", rightMotorBack);
            }

            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program
