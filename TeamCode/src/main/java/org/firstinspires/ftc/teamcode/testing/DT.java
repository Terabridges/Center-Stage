package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DT {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;


    void setMotors(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br){
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    void setDirections(DcMotorSimple.Direction fl, DcMotorSimple.Direction fr, DcMotorSimple.Direction bl, DcMotorSimple.Direction br){
        this.fl.setDirection(fl);
        this.fr.setDirection(fr);
        this.bl.setDirection(bl);
        this.br.setDirection(br);
    }

    void motorPow(double fl, double fr, double bl, double br){
        this.fl.setPower(fl);
        this.fr.setPower(fr);
        this.bl.setPower(bl);
        this.br.setPower(br);
    }
}
