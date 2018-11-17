//Run from the necessary package
package org.firstinspires.ftc.teamcode.legoheads;

//Import necessary items
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name="Tele Op Test") //Name the class
public class testBotTele extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
//    DcMotor leftMotorBack;
//    DcMotor rightMotorBack;



    //Define floats to be used as joystick and trigger inputs
    float drivePower;
    float shiftPower;
    float rightTurnPower;
    float leftTurnPower;

    DcMotor lifter;
    CRServo pin;
    Servo marker;

    //Define a function to use to set motor powers
    public void setDriveMotorPowers(float leftFrontPower, float leftBackPower, float rightFrontPower, float rightBackPower)
    {
        //Use the entered powers and feed them to the motors
        leftMotorFront.setPower(leftFrontPower);
//        leftMotorBack.setPower(leftBackPower);
        rightMotorFront.setPower(rightFrontPower);
//        rightMotorBack.setPower(rightBackPower);
    }

    //***********************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        //Get references to the DC motors from the hardware map
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
//        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
//        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");
        lifter = hardwareMap.dcMotor.get("lifter");

        pin = hardwareMap.crservo.get("pin");
        marker = hardwareMap.servo.get("marker");

        //Reverse some motors and keep others forward
        leftMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Wait for start button to be clicked
        waitForStart();
//***********************************************************************************************************
        //LOOP BELOW
        //While the op mode is active, do anything within the loop
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            //Set float variables as the inputs from the joysticks and the triggers
            drivePower = -gamepad1.left_stick_y;
            shiftPower = -gamepad1.left_stick_x;
            leftTurnPower = gamepad1.left_trigger / 2;
            rightTurnPower = gamepad1.right_trigger / 2;




            //Drive if the joystick is pushed more Y than X
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
                setDriveMotorPowers(drivePower, drivePower, drivePower, drivePower);
            }

            //Shift if the joystick is pushed more on X than Y
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
                setDriveMotorPowers(-shiftPower, shiftPower, shiftPower, -shiftPower);
            }

            //If the left trigger is pushed, turn left at that power
            if (leftTurnPower > 0)
            {
                setDriveMotorPowers(-leftTurnPower, -leftTurnPower, leftTurnPower, leftTurnPower);
            }

            //If the right trigger is pushed, turn right at that power
            if (rightTurnPower > 0)
            {
                setDriveMotorPowers(rightTurnPower, rightTurnPower, -rightTurnPower, -rightTurnPower);
            }

            //Do nothing if joystick is stationary and triggers are stationary
            if ((drivePower == 0) && (shiftPower == 0) && (leftTurnPower == 0) && (rightTurnPower == 0))
            {
                setDriveMotorPowers(0, 0, 0, 0);
            }

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
                pin.setPower(0.75) ;
                Thread.sleep(7200);
                pin.setPower(0.0);
            }

            if (gamepad1.x)
            {
                pin.setPower(-0.75);
                Thread.sleep(7200);
                pin.setPower(0.0);
            }

            //Dump team marker
            if(gamepad1.dpad_down)
            {
                marker.setPosition(-0.3);
            }




            //Update the data
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        } //Close "while (opModeIsActive())" loop
    } //Close main
} //Close class and end program