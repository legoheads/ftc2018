//Run from the package
package org.firstinspires.ftc.teamcode;

//Import necessary items

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public class commonFunctions extends initialization
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

    public commonFunctions(){

    }

    /**
     * Initialize all the hardware
     * This creates a data type DriveFunctions to store all the hardware devices
     */
    public commonFunctions(DcMotor leftMotorFront, DcMotor rightMotorFront, DcMotor leftMotorBack, DcMotor rightMotorBack)
    {
        //These lines enable us to store the motors, sensors and CDI without having to write them over and over again
        //Initialize DC and Servo motors
        this.leftMotorFront = leftMotorFront;
        this.leftMotorBack = leftMotorBack;
        this.rightMotorFront = rightMotorFront;
        this.rightMotorBack = rightMotorBack;
        this.glyphWheelLeft = glyphWheelLeft;
        this.glyphWheelRight = glyphWheelRight;
        this.glyphLift = glyphLift;
        this.glyphFlip = glyphFlip;
        this.relicGrab = relicGrab;
        this.relicSpool = relicSpool;
        this.relicFlip = relicFlip;
        this.jewelArm = jewelArm;

        //Initialize sensors
        this.colorSensor = colorSensor;
    }

    /**
     * Set sensor addresses, modes and DC motor directions, modes
     */
    public void initializeMotorsAndSensors()
    {
        //Set the sensor to the mode that we want
        colorSensor.enableLed(true);

        //Reverse some motors and keep others forward
        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set the drive motors to brake mode to prevent rolling due to chain
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set the relic spool on brake mode to not allow the motor move when it has no power set to it
        relicSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set the glyph lifter on brake mode to lock the glyphter at a certain height when not being used
        glyphLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * Takes in motor powers for 4 drive motors
     */
    public void setDriveMotorPowers(float leftFrontPower, float leftBackPower, float rightFrontPower, float rightBackPower)
    {
        //Use the entered powers and feed them to the motors
        leftMotorFront.setPower(leftFrontPower);
        leftMotorBack.setPower(leftBackPower);
        rightMotorFront.setPower(rightFrontPower);
        rightMotorBack.setPower(rightBackPower);
    }

    /**
     * If this function is called, stop the drive motors
     */
    public void stopDriving()
    {
        //Set all drive motor powers as zero
        setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
    }

    /**
     * If this function is called, turn on the intake wheels either intaking or outputing
     */
    public void intake(float power)
    {
        //Start the intake wheels
        glyphWheelLeft.setPower(-power);
        glyphWheelRight.setPower(power);
    }



//    /**
//     * If this function is called, it enables us to run one servo motor for a specific time
//     * without affecting the rest of the robot
//     */
//    public void servoTime(Servo motor, double position1, double position2, int time)
//    {
//        //Define an elapsed time variable to count time in milliseconds
//        ElapsedTime counter = new ElapsedTime();
//
//        //Set the servo motor to the first position entered
//        motor.setPosition(position1);
//
//        //Reset the time counter
//        counter.reset();
//
//        //While the timecounter is less than the entered time, delay
//        if (counter.time() <= time)
//        {
//
//        }
//
//        //After the entered time has elapsed, move the servo to the second position
//        motor.setPosition(position2);
//    }

//    /**
//     * @param colorSensor take in the correct color sensor
//     * @return returns true if the supplied ColorSensor either red or blue.  False otherwise
//     */
//    public boolean iSeeAColor(ColorSensor colorSensor)
//    {
//        //This is an array that stores the hue[0], saturation[1], and value[2], values
//        float[] hsvValues = {0F, 0F, 0F};
//
//        //Convert from RGB to HSV (red-green-blue to hue-saturation-value)
//        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
//
//        //If no value, return false
//        if (hsvValues[2] == 0)
//        {
//            return false;
//        }
//
//        //Otherwise return true
//        return true;
//    }

//    /**
//     * Determines what color the color sensor is seeing
//     * @param colorSensor take in the correct color sensor
//     * @return The string "Blue" if we see the color blue, "Red" if we see the color red
//     */
//    public String whatColor(ColorSensor colorSensor)
//    {
//        //Define float for hue
//        float hue;
//
//        //This is an array that stores the hue[0], saturation[1], and value[2], values
//        float[] hsvValues = {0F, 0F, 0F};
//
//        //Convert from RGB to HSV (red-green-blue to hue-saturation-value)
//        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
//
//        //store the first value from the array into hue
//        hue = hsvValues[0];
//
//        //If hue is greater than 120, we are looking at blue, so return blue
//        if (hue > 120)
//        {
//            return "Blue";
//        }
//
//        //Otherwise return red
//        return "Red";
//    }

//    public void jewelPush(ColorSensor colorSensor, String color, String colorSeen) throws InterruptedException
//    {
//        //Define constants to avoid magic numbers
//        float power = (float) 0.7;
//        int shortDistance = 120;
//        int longDistance = 170;
//
//        //Drop the arm
//        jewelArm.setPosition(0.7);
//
//        //Wait for 1 second
//        Thread.sleep(1000);
//
//        //Delay until a color is seen
//        while (!iSeeAColor(colorSensor))
//        { }
//
//        //If a color is seen, set it to the variable colorSeen
//        if (iSeeAColor(colorSensor))
//        {
//            colorSeen = whatColor(colorSensor);
//        }
//
//        //If the color seen is our team color
//        if (colorSeen.equals(color))
//        {
//            //Turn to the right to hit the opposite ball
//            rightTurnAutonomous(power, longDistance);
//
//            //Lift the arm
//            jewelArm.setPosition(0.0);
//
//            //Turn back to the original position so we can continue
//            leftTurnAutonomous(power, longDistance);
//        }
//
//        //If the color seen is not our team color
//        if (!colorSeen.equals(color))
//        {
//            //Turn to the left to hit the ball we see
//            leftTurnAutonomous(power, shortDistance);
//
//            //Lift the arm
//            jewelArm.setPosition(0.0);
//
//            //Turn back to the original position so we can continue
//            rightTurnAutonomous(power, shortDistance);
//        }
//    }
} //Close class and end program

