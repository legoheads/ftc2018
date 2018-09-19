//Run from the necessary package
package org.firstinspires.ftc.teamcode;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp") //Name the class
public class Teleop extends commonFunctions
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
    Servo jewelArm;

    //Define the color sensor
    ColorSensor colorSensor;

    //Define floats to be used as joystick inputs and trigger inputs
    float drivePower;
    float shiftPower;
    float leftTurnPower;
    float rightTurnPower;

//***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
//        //Get references to the DC Motors from the hardware map
//        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
//        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
//        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
//        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
//        glyphWheelLeft = hardwareMap.dcMotor.get("glyphWheelLeft");
//        glyphWheelRight = hardwareMap.dcMotor.get("glyphWheelRight");
//        glyphLift = hardwareMap.dcMotor.get("glyphLift");
//        relicSpool = hardwareMap.dcMotor.get("relicSpool");
//
//        //Get references to the Servo Motors from the hardware map
//        glyphFlip = hardwareMap.servo.get("glyphFlip");
//        relicGrab = hardwareMap.servo.get("relicGrab");
//        relicFlip = hardwareMap.crservo.get("relicFlip");
//        jewelArm = hardwareMap.servo.get("jewelArm");
//
//        //Get references to the Color Sensor from the hardware map
//        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        super.runOpMode();

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        commonFunctions commonFunctions = new commonFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack);
        teleOpFunctions teleOpFunctions = new teleOpFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack);

        //Set the sensor to active mode and set the directions of the motors
        commonFunctions.initializeMotorsAndSensors();

        //Wait for start button to be clicked
        waitForStart();

//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {

    //DRIVE MOTOR CONTROLS
            //Set float variables as the inputs from the joysticks and the triggers
            drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.85);
            shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.85);
            leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.75);
            rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.75);

            //Drive if joystick pushed more Y than X on gamepad1 (fast)
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
                teleOpFunctions.straightDrive(drivePower);
            }

            //Shift if pushed more on X than Y on gamepad1 (fast)
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
                teleOpFunctions.shift(shiftPower);
            }

            //If the left trigger is pushed on gamepad1, turn left at that power (fast)
            if (leftTurnPower > 0)
            {
                teleOpFunctions.leftTurn(leftTurnPower);
            }

            //If the right trigger is pushed on gamepad1, turn right at that power (fast)
            if (rightTurnPower > 0)
            {
                teleOpFunctions.rightTurn(rightTurnPower);
            }

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
            {
                commonFunctions.stopDriving();
            }

            //Count time
            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program
