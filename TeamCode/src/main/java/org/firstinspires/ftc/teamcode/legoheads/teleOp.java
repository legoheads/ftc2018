package org.firstinspires.ftc.teamcode.legoheads;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Tele Op ") //Name the class
public class teleOp extends LinearOpMode
{

    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    DcMotor lifter;
    CRServo pin;
    CRServo intake;

    //Define drive powers to avoid magic numbers
    float drivePower;
    float shiftPower;
    float leftTurnPower;
    float rightTurnPower;

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

        lifter = hardwareMap.dcMotor.get("lifter");
        pin = hardwareMap.crservo.get("pin");
        intake = hardwareMap.crservo.get("intake");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions functions = new DriveFunctions(leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, lifter, pin, intake);

        //Set the sensor to active mode
        //Set the directions and modes of the motors.
        functions.initializeMotorsAndSensors();

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.85);
            shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.85);
            leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.75);
            rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.75);

            //Drive if joystick pushed more Y than X on gamepad1 (fast)
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
                functions.driveTeleop(drivePower);
            }

            //Shift if pushed more on X than Y on gamepad1 (fast)
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
                functions.shiftTeleop(shiftPower);
            }

            //If the left trigger is pushed on gamepad1, turn left at that power (fast)
            if (leftTurnPower > 0)
            {
                functions.leftTurnTeleop(leftTurnPower);
            }

            //If the right trigger is pushed on gamepad1, turn right at that power (fast)
            if (rightTurnPower > 0)
            {
                functions.rightTurnTeleop(rightTurnPower);
            }

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
            {
                functions.setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
            }
            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            if (gamepad1.a)
            {
                //Use the encoder
                lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Reset the encoder
                lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //Use the encoder
                lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Set up the motor to run to the given position
                lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Set the target position as the value entered
                lifter.setTargetPosition(2925);

                //Turn the motor on at the corresponding power
                lifter.setPower(0.6);

                //Empty while loop while the motor is moving
                while ((lifter.isBusy()))
                { }

                //Stop the motor
                lifter.setPower(0.0);

                //Use the encoder in the future
                lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad1.y)
            {
                //Use the encoder
                lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Reset the encoder
                lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //Use the encoder
                lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //Set up the motor to run to the given position
                lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //Set the target position as the value entered
                lifter.setTargetPosition(-2925);

                //Turn the motor on at the corresponding power
                lifter.setPower(-0.6);

                //Empty while loop while the motor is moving
                while ((lifter.isBusy()))
                { }

                //Stop the motor
                lifter.setPower(0.0);

                //Use the encoder in the future
                lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad1.b)
            {
                pin.setPower(0.75);
                Thread.sleep(7200);
                pin.setPower(0.0);
            }

            if (gamepad1.x)
            {
                pin.setPower(-0.75);
                Thread.sleep(7200);
                pin.setPower(0.0);
            }

            if (gamepad1.right_bumper)
            {
                intake.setPower(1.0);
            }

            if (gamepad1.left_bumper)
            {
                intake.setPower(-1.0);
            }

        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program
