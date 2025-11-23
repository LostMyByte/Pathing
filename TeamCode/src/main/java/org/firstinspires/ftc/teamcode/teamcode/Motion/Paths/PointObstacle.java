package org.firstinspires.ftc.teamcode.teamcode.Motion.Paths;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class PointObstacle extends Obstacle {

    double repulsion;
    double size;
    Vector obstaclePosition;

    final Matrix projection = new GeneralMatrix(5, 5, new double[] {
            1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
    });

    public PointObstacle(double x, double y, double size, double repulsion) {
        this.obstaclePosition = new Vector(x, y, 0, 0, 0);
        this.size = size;
        this.repulsion = repulsion;
    }

    private Vector getError(Vector pos) {
        return pos.subtracted(obstaclePosition);
    }

    @Override
    public double getCost(Vector pos) {
        Vector error = getError(pos);
        return repulsion * Math.exp (- error.dotProduct(projection.multiplied(error))/(size*size));
    }

    @Override
    public Vector getDerivative(Vector pos) {
        Vector error = getError(pos);
        return projection.multiplied(error).multiplied(-2*getCost(pos) / (size*size));
    }

    @Override
    public Matrix get2ndDerivative(Vector pos) {
        Vector error = getError(pos);
        Matrix result = projection.multiplied(2 / (size * size));
        Vector v1 = projection.multiplied(error).multiplied(2 / (size*size*size*size));
        for (int i = 0; i < 5; i++) {
            for (int j = 0; j < 5; j++) {
                result.add(i, j, v1.get(i) * v1.get(j));
            }
        }
        return result.multiplied(-getCost(pos));
    }
}
