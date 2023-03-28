package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.gamepieces.AprilTag;
import frc.robot.gamepieces.Cone;
import frc.robot.gamepieces.Cube;
import frc.robot.gamepieces.GamePieceType;

import java.nio.ByteBuffer;
import java.util.Vector;

public class Vision {
    private I2C Wire;
    public Vector<Cube> cubes;
    public Vector<Cone> cones;
    public AprilTag aprilTag;

    public GamePieceType mode = GamePieceType.APRILTAG;

    public Vision() {
        //Address of peripheral?
        Wire = new I2C(I2C.Port.kMXP, 1);
    }

    public void update(){
        //reset data
        cubes.clear();
        cones.clear();

        int numBytes = 128;
        byte[] data = new byte[numBytes];
        Wire.read(1, numBytes, data);

        GamePieceType type = dataToGamepiece(data[0]);

        if (type == GamePieceType.CONE){
            int numberOfCones = data[1];
            for (int i = 0; i <= numberOfCones; i++) {
                // 3rd and 4th byte
                cones.add(new Cone(data[2+(i*2)], data[3+(i*2)]));
            }
        }

        if (type == GamePieceType.CUBE){
            int numberOfCubes = data[1];
            for (int i = 0; i <= numberOfCubes; i++) {
                // 3rd and 4th byte
                cubes.add(new Cube(data[2+(i*2)], data[3+(i*2)]));
            }
        }

        if(type == GamePieceType.APRILTAG){
            aprilTag.id = data[2];
            aprilTag.active = data[3];
            aprilTag.x = data[4];
            aprilTag.y = data[5];
            aprilTag.dist = data[6];
        }
    }

    // 0
    public void setMode(GamePieceType type){
        switch (type) {
            case CONE:
                Wire.write(1, 0);
            case CUBE:
                Wire.write(1, 1);
            case APRILTAG:
                Wire.write(1, 2);
        }
    }

    private GamePieceType dataToGamepiece(int data){
        switch (data){
            case 0:
                return GamePieceType.CONE;
            case 1:
                return GamePieceType.CUBE;
            case 2:
                return GamePieceType.APRILTAG;
            default:
                return GamePieceType.CONE;
        }
    }
}
