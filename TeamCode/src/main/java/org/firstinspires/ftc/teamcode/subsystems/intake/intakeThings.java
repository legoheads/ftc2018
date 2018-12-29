package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.Servo;

public class intakeThings implements Intake{

    Servo flip;

    final double UP_POSITION = 0.3;
    final double DOWN_POSITION = 0.0;
    final double FLIP_POSITION = 0.6;

    public intakeThings(Servo mineralFlip) { this.flip = mineralFlip; }

    @Override
    public void up(){
        flip.setPosition(UP_POSITION);
    }

    @Override
    public void down(){
        flip.setPosition(DOWN_POSITION);
    }

    @Override
    public void flip() {
        flip.setPosition(FLIP_POSITION);
    }
}