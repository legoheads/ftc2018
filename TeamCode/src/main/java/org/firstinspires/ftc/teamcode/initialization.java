package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class initialization extends LinearOpMode
{
    //Define drive motors
    public DcMotor leftMotorFront;
    public DcMotor rightMotorFront;
    public DcMotor leftMotorBack;
    public DcMotor rightMotorBack;

    //Define glyph motors
    public DcMotor glyphWheelLeft;
    public DcMotor glyphWheelRight;
    public DcMotor glyphLift;
    public Servo glyphFlip;

    //Define relic motors
    public Servo relicGrab;
    public CRServo relicFlip;
    public DcMotor relicSpool;

    //Define the jewel motor
    public Servo jewelArm;

    //Define the color sensor
    public ColorSensor colorSensor;

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
        jewelArm = hardwareMap.servo.get("jewelArm");

        //Get references to the Color Sensor from the hardware map
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
    }
}
