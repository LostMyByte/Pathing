package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous;


public class    AAA_Paths {

    public static double[] stageDoor = {0, -11};
    public static double[] leftStack = {-12, -58};
    public static double[] rightTruss = {-56.3, -11};
    public static double[] outsideTruss = {-56.3, 12};

    static double movementTestY = 38;
    static double movementTestX = -60;

    public enum Path {
        //BACK AND FORTH


        Forth(new double[][]{{0,0}, {0,80}}),
        Back(new double[][]{{0,80}, {0,0}}),

        //TEST AUTO
        TestPath(new double[][]{{0,0}, {0,55}}),
        TestPath2(new double[][]{{0,55}, {-55,55}}),
        TestPath3(new double[][]{{-55,55}, {-55,0}}),
        TestPath4(new double[][]{{55,0}, {0,0}}),
        TestPath2a(new double[][]{{0,55}, {0,0}}),
        holdPos(new double[][]{{0,55},{0,0}}),
        Hold2,

        //APRIL TAG RESET TEST
        DriveForward(new double[][]{{0, 0}, {0, 200}}),
        DriveBackward(new double[][]{{0, 0}, {-50, -50}}),


        //BLUE AUTOS
        //Gate Side

        //All
//Last year code
      /*  BoardToInterimBlueGate(new double[][]{{tag3x, halfRobotSizeYDepositorExtended -2}, {gateStackBlue+2, 120}}),
        InterimToStackBlueGate(new double[][]{{gateStackBlue+2, 120}, {gateStackBlue+3, halfRobotSizeYIntakeExtended+1}}),
        StackToInterimBlueGate(new double[][]{{gateStackBlue+3, halfRobotSizeYIntakeExtended+2}, {gateStackBlue+2, 90}}),
        InterimToBoardBlueGate(new double[][]{{gateStackBlue+2, 90}, {tag3x, backdropY - 25}}),
*/
        //Back-Specific

        //Close Side
//last year code
       /* ToStackBlueClose(new double[][]{{12, 22}, {closeStackBlue, halfRobotSizeYIntakeExtended + 2}}),
        StackToInterim1BlueClose(new double[][]{{closeStackBlue - halfRobotSizeYIntakeExtended / Math.sqrt(2), halfRobotSizeYIntakeExtended / Math.sqrt(2)}, {11, 22}}),
        Interim1ToInterim2BlueClose(new double[][]{{18, 22}, {18, 84}}),
        Interim2ToBoardBlueClose(new double[][]{{18, 84}, {tag1x + 5, backdropY - 30}}),
        BoardToInterimBlueClose(new double[][]{{tag1x, backdropY - 13}, {10, 95}}),
        Interim2ToInterim1BlueClose(new double[][]{{10, 95}, {9, 22}}),
*/

        //RED AUTOS
        //Gate Side
//last year code
      /*  StartToBoardRedFront(new double[][]{{fieldSizeX - halfRobotSizeY, 87.5}, {tag6x, backdropY - 25}}),
        BoardToInterimRedGate(new double[][]{{tag4x, backdropY - 13}, {gateStackRed, 120}}),
        InterimToStackRedGate(new double[][]{{gateStackRed, 120},{gateStackRed, halfRobotSizeYIntakeExtended+2}}),
        StackToInterimRedGate(new double[][]{{gateStackRed,halfRobotSizeYIntakeExtended+2}, {gateStackRed,120}}),
        InterimToBoardRedGate(new double[][]{{gateStackRed,120},{tag4x,backdropY - halfRobotSizeYDepositorExtended+2}}),
*/
        //Close Side
//last year code
      /*  ToStackRedClose(new double[][]{{fieldSizeX - 12, 22}, {fieldSizeX - closeStackBlue - halfRobotSizeYIntakeExtended / Math.sqrt(2), halfRobotSizeYIntakeExtended + 2}}),
        StackToInterim1RedClose(new double[][]{{fieldSizeX - closeStackBlue - halfRobotSizeYIntakeExtended / Math.sqrt(2), halfRobotSizeYIntakeExtended / Math.sqrt(2)}, {fieldSizeX - 11, 22}}),
        Interim1ToInterim2RedClose(new double[][]{{fieldSizeX - 12, 22}, {fieldSizeX - 12, 84}}),
        Interim2ToBoardRedClose(new double[][]{{fieldSizeX - 12, 84}, {tag6x, backdropY - 20}}),
        BoardToInterimRedClose(new double[][]{{tag6x, backdropY - 13}, {fieldSizeX - 12, 95}}),
        Interim2ToInterim1RedClose(new double[][]{{fieldSizeX - 12, 95}, {fieldSizeX - 12, 22}}),
*/
        //Pathless States
        Score1,
        BeforeSpikes,
        BehindSpikes,
        BehindFarSpike,
        PushFarSpike,
        BehindFarSpike2,
        BehindMiddleSpike,
        PushMiddleSpike,
        BehindMiddleSpike2,
        BehindWallSpike,
        PushWallSpike,
        DropYellow,
        Intake,
        HoldPark,
        CheckForTags,
        BackStart,
        SpikeToStack,
        ToStackNoExtendo,
        SpikeToBoard,
        StartToSpike,
        ShimmyShimmyAhShimmyAhShimmyAhDrankSwaLaLaLa,
        StopState,

        Score2,

        Score3,
        Lupine,

        Score4,
        Submersible,
        ScoreFailure,
        TransferFail,
        GoToScore,

        SampleToHumanPlayer,
        BeforeIntakeHumanPlayer,
        IntakeHumanPlayer,
        SpecimenScoreReady,
        SpecimenScore,
        WallSpike,
        MiddleSpike,
        FarSpike,

        PushSpikes,
        Intake1,
        Intake2,
        Sub2
        ;




        private double[][] pathPoints;
        private AAA_Paths.Function[] path;
        private PathType pathType;

        public double[] endPoint;

        public double x, y, t = 0;
        public double dx, dy, d2x, d2y;

        public boolean curvy;
        public boolean targeting = false;

        public double targetingPower = .2;
        public double tIncrementCM = .2;
        public double tIncrement;

        public enum PathType {
            Curve, Line, Orbit, NONE
        }

        Path(double[][] pathPoints) {
            if (pathPoints.length > 2) {
                pathType = PathType.Curve;
            } else {
                pathType = PathType.Line;
            }
            construct(pathPoints, pathType);
        }

        public void construct(double[][] pathPoints, PathType pathType) {
            this.pathPoints = pathPoints;
            this.pathType = pathType;
        }

        public void compile() {
            calculatePath();
            getTIncrement();
            resetPath();
        }

        public void resetPath() {
            x = getX(0);
            y = getY(0);
            t = 0;
            targeting = false;
            updatePathDerivatives();
        }

        public void compile(double tIncrement) {
            calculatePath();
            this.tIncrement = tIncrement;
        }

        Path() {
            this.pathPoints = new double[][]{{0}, {0}};
            this.pathType = PathType.NONE;
        }

        private void calculatePath() {
            switch (pathType) {
                default:
                case Line:
                    path = AAA_Paths.getLine(pathPoints);
                    curvy = false;
                    break;

                case Curve:
                    path = AAA_Paths.getBezierCurve(pathPoints);
                    curvy = true;
                    break;

                case Orbit:
                    path = AAA_Paths.getOrbit(pathPoints);
                    curvy = true;
                    break;
            }
            x = getX(0);
            y = getY(0);
            setEndPoint(getX(1), getY(1));
        }


        private void getTIncrement() {
//            integrate to find the length of the path
            double dt = .001;
            double td = 1 / dt;
            double pathLength = 0;
            for (int t = 0; t <= td; t++) {
                double f = t * dt;
                pathLength += Math.hypot(getDX(f), getDY(f)) * dt;
            }
            tIncrement = tIncrementCM / pathLength;
//            tIncrement = .003;
            //tIncrement .003 for 200cm

        }

        public double distanceToTarget() {
            return Math.hypot(x - endPoint[0], y - endPoint[1]);
        }

        public void setEndPoint(double x, double y) {
            endPoint = new double[]{x, y};
        }

        public void setTargetingPower(double p) {
            targetingPower = p;
        }

        //getting specific points on path
        public double[] getPoint(double t) {
            return new double[]{getX(t), getY(t)};
        }

        public double getX(double t) {
            if (t > 1) {
                return getX(1);
            }
            return path[0].get(t);
        }

        public double getY(double t) {
            if (t > 1) {
                return getY(1);
            }
            return path[1].get(t);
        }

        public double getDX(double t) {
            if (t > 1) {
                return getDX(1);
            }
            double dx = path[2].get(t);
            return (dx == 0) ? .00000001 : dx;
        }

        public double getDY(double t) {
            if (t > 1) {
                return getDY(1);
            }
            return path[3].get(t);
        }

        public double getD2X(double t) {
            if (t > 1) {
                return getD2X(1);
            }
            double d2x = path[4].get(t);
            return (d2x == 0) ? .000000001 : d2x;
        }

        public double getD2Y(double t) {
            if (t > 1) {
                return getD2Y(1);
            }
            return path[5].get(t);
        }

        //updating current class
        public void updatePathDerivatives() {
            updateDX(t);
            updateDY(t);
            updateD2X(t);
            updateD2Y(t);
        }

        public double updateX(double t) {
            x = getX(t);
            return x;
        }

        public double updateY(double t) {
            y = getY(t);
            return y;
        }

        public double updateDX(double t) {
            dx = getDX(t);
            return dx;
        }

        public double updateDY(double t) {
            dy = getDY(t);
            return dy;
        }

        public double updateD2X(double t) {
            d2x = getD2X(t);
            return d2x;
        }

        public double updateD2Y(double t) {
            d2y = getD2Y(t);
            return d2y;
        }
    }

    ;

    //Input points of a bezier curve in the order you want them to be calculated
    public static AAA_Paths.Function[] getBezierCurve(double[][] points) {
        AAA_Paths.Function[] spline = new AAA_Paths.Function[6];
        int n = points.length - 1;
        //x of point
        spline[0] = (u) -> {
            double x = 0;
            for (int i = 0; i <= n; i++) {
                x += B(n, i, u) * points[i][0];
            }
            return x;
        };
        //y of point
        spline[1] = (u) -> {
            double y = 0;
            for (int i = 0; i <= n; i++) {
                y += B(n, i, u) * points[i][1];
            }
            return y;
        };
        //x' of point
        spline[2] = (u) -> {
            double dx = 0;
            for (int i = 0; i <= n - 1; i++) {
                dx += B(n - 1, i, u) * ((n) * (points[i + 1][0] - points[i][0]));
            }
            return dx;

        };
        //y' of point
        spline[3] = (u) -> {
            double dy = 0;
            for (int i = 0; i <= n - 1; i++) {
                dy += B(n - 1, i, u) * ((n) * (points[i + 1][1] - points[i][1]));
            }
            return dy;
        };

        //x'' of point
        spline[4] = (u) -> {
            double d2x = 0;
            for (int i = 0; i <= n - 2; i++) {
                d2x += B(n - 2, i, u) * ((n) * (n - 1) * (points[i + 2][0] - 2 * points[i + 1][0] + points[i][0]));
            }
            return d2x;
        };
        //y'' of point
        spline[5] = (u) -> {
            double d2y = 0;
            for (int i = 0; i <= n - 2; i++) {
                d2y += B(n - 2, i, u) * ((n) * (n - 1) * (points[i + 2][1] - 2 * points[i + 1][1] + points[i][1]));
            }
            return d2y;
        };

        return spline;
    }

    //takes 2 points, start and end
    public static AAA_Paths.Function[] getLine(double[][] points) {
        //t goes between 0 and 1, 1 being the endpoint of the graph
        double xDiff = points[1][0] - points[0][0];
        double yDiff = points[1][1] - points[0][1];
        AAA_Paths.Function[] line = new AAA_Paths.Function[6];
        line[0] = (t) -> t * xDiff + points[0][0];
        line[1] = (t) -> t * yDiff + points[0][1];
        line[2] = (t) -> xDiff;
        line[3] = (t) -> yDiff;
        line[4] = (t) -> 0;
        line[5] = (t) -> 0;
        return line;
    }

    //takes start point, center of rotation, and fraction of circle wanted to be rotated (make sure to include direction with +/-
    public static AAA_Paths.Function[] getOrbit(double[][] points) {

        double[] center = points[0];
        double[] start = points[1];
        double fraction = points[2][0];
        double radius = Math.sqrt(Math.pow(center[0] - start[0], 2) + Math.pow(center[1] - start[1], 2));

        AAA_Paths.Function[] arc = new AAA_Paths.Function[2];

        arc[0] = (t) -> center[0] + Math.cos(t * 2 * Math.PI * fraction) * radius;
        arc[1] = (t) -> center[1] + Math.sin(t * 2 * Math.PI * fraction) * radius;

        return arc;
    }

    //faster
    private static double fact(int n) {
        switch (n) {
            case 0:
            case 1:
                return 1;
            case 2:
                return 2;
            case 3:
                return 6;
            case 4:
                return 24;
            case 5:
                return 120;
            case 6:
                return 720;
            default:
                return 0;
        }
    }

    private static double B(int n, int i, double u) {
        return fact(n) / (fact(i) * fact(n - i)) * (double) Math.pow(u, i) * (double) Math.pow(1 - u, n - i);
    }

    public interface Function {
        double get(double t);
    }
}
