package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.DriveFunctions;

public class linearActuator implements Hang{

    private final float HANG_POWER = (float) 1.0;
    private final int HANG_DISTANCE = 2925;
    private final int MAX_DISTANCE = 3100;

    private DcMotor hanger;

    public linearActuator(DcMotor linearActuator){
        hanger = linearActuator;
    }

    public void up(){
        DriveFunctions.oneMotorEncoder(hanger, HANG_POWER, HANG_DISTANCE);
    }
    public void down(){
        DriveFunctions.oneMotorEncoder(hanger, HANG_POWER, -HANG_DISTANCE);
    }
    public void maxUp(){
        DriveFunctions.oneMotorEncoder(hanger, HANG_POWER, MAX_DISTANCE);
    }
    public void maxDown(){
        DriveFunctions.oneMotorEncoder(hanger, HANG_POWER, -MAX_DISTANCE);
    }
}
