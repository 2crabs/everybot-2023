package frc.robot;

import edu.wpi.first.wpilibj.I2C;

import java.nio.ByteBuffer;

public class Vision {
    private I2C Wire;
    public int val;

    public Vision() {
        //Address of controller is 0
        Wire = new I2C(I2C.Port.kOnboard, 1);
    }

    public void update(){
        int numBytes = 1;
        byte[] data = new byte[numBytes];
        //Address of peripheral
        Wire.read(12, numBytes, data);
        val = data[0];
    }
}
