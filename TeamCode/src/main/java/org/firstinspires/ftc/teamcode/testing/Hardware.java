package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class Hardware {
    private Drivetrain drivetrain;
    private IMU imu;

    Hardware(){

    }

    void setHardware(Drivetrain drivetrain, IMU imu){
        this.drivetrain = drivetrain;
        this.imu = imu;
    }

    void setHardware(HardwareMap hardwareMap){
        final Drivetrain drivetrain = new Drivetrain();
        drivetrain.setMotors(hardwareMap);
        setHardware(
                drivetrain,
                hardwareMap.get(IMU.class, "imu")
        );
    }

    Drivetrain getDrivetrain(){
        return drivetrain;
    }

    IMU getImu(){
        return imu;
    }
}
