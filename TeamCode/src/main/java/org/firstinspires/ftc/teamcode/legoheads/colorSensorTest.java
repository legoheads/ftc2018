//Run from the necessary package
package org.firstinspires.ftc.teamcode.legoheads;

//Import necessary items

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@TeleOp(name="Color Sensor Test") //Name the class
public class colorSensorTest extends LinearOpMode
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

//    //Define the color sensor
//    ColorSensor colorSensorCenter;
//    ColorSensor colorSensorRight;

    //Define a float array that will be used to store sensor input
    float hsvValues[] = {0F, 0F, 0F};

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

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
//            //Convert the color sensor values from RGB to HSV
//            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
//
//            //Display the sensor outputs on the screen
//            telemetry.addData("Clear ", colorSensor.alpha());
//            telemetry.addData("Red ", colorSensor.red());
//            telemetry.addData("Green ", colorSensor.green());
//            telemetry.addData("Blue ", colorSensor.blue());
//            telemetry.addData("Hue ", hsvValues[0]);
//            telemetry.addData("Saturation ", hsvValues[1]);
//            telemetry.addData("Value ", hsvValues[2]);
//
//            //Update the data if/when it changes
//            telemetry.update();
//
            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program