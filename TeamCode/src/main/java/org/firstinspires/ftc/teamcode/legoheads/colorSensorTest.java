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

//@Disabled
@TeleOp(name="Color Sensor Test") //Name the class
public class colorSensorTest extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define glyph motors
    DcMotor glyphWheelLeft;
    DcMotor glyphWheelRight;
    DcMotor glyphLift;
    Servo glyphFlip;

    //Define relic motors
    Servo relicGrab;
    CRServo relicFlip;
    DcMotor relicSpool;

    //Define the jewel motor
    Servo sensorArm;

    //Define the color sensor
    ColorSensor colorSensorCenter;
    ColorSensor colorSensorRight;

    //Define a float array that will be used to store sensor input
    float hsvValuesCenter[] = {0F, 0F, 0F};
    float hsvValuesRight[] = {0F, 0F, 0F};

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
        glyphWheelLeft = hardwareMap.dcMotor.get("glyphWheelLeft");
        glyphWheelRight = hardwareMap.dcMotor.get("glyphWheelRight");
        glyphLift = hardwareMap.dcMotor.get("glyphLift");
        relicSpool = hardwareMap.dcMotor.get("relicSpool");

        //Get references to the Servo Motors from the hardware map
        glyphFlip = hardwareMap.servo.get("glyphFlip");
        relicGrab = hardwareMap.servo.get("relicGrab");
        relicFlip = hardwareMap.crservo.get("relicFlip");
        sensorArm = hardwareMap.servo.get("sensorArm");

        //Get references to the Color Sensor from the hardware map
        colorSensorCenter = hardwareMap.colorSensor.get("colorSensorCenter");
        colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, glyphWheelLeft, glyphWheelRight, glyphLift, glyphFlip, relicGrab, relicFlip, relicSpool, sensorArm, colorSensorCenter, colorSensorRight);

        functions.initializeMotorsAndSensors();

        //Wait for start button to be clicked
        waitForStart();

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            //Convert the color sensor values from RGB to HSV
            Color.RGBToHSV(colorSensorCenter.red() * 8, colorSensorCenter.green() * 8, colorSensorCenter.blue() * 8, hsvValuesCenter);
            Color.RGBToHSV(colorSensorRight.red() * 8, colorSensorRight.green() * 8, colorSensorRight.blue() * 8, hsvValuesRight);

            //Display the sensor outputs on the screen
            telemetry.addData("Clear Center", colorSensorCenter.alpha());
            telemetry.addData("Red Center", colorSensorCenter.red());
            telemetry.addData("Green Center", colorSensorCenter.green());
            telemetry.addData("Blue Center", colorSensorCenter.blue());
            telemetry.addData("Hue Center", hsvValuesCenter[0]);
            telemetry.addData("Saturation Center", hsvValuesCenter[1]);
            telemetry.addData("Value Center", hsvValuesCenter[2]);

            //Display the sensor outputs on the screen
            telemetry.addData("Clear Right", colorSensorRight.alpha());
            telemetry.addData("Red Right", colorSensorRight.red());
            telemetry.addData("Green Right", colorSensorRight.green());
            telemetry.addData("Blue Right", colorSensorRight.blue());
            telemetry.addData("Hue Right", hsvValuesRight[0]);
            telemetry.addData("Saturation Right", hsvValuesRight[1]);
            telemetry.addData("Value Right", hsvValuesRight[2]);

            //Update the data if/when it changes
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program