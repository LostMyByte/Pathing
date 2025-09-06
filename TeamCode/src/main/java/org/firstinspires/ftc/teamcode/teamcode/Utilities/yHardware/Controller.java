package org.firstinspires.ftc.teamcode.teamcode.Utilities.yHardware;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities.MathUtils.distance2D;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities.Derivative;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities.Vector2d;

public class Controller {
	
	private final Gamepad gamepad;
	
	public Thumbstick rightStick, leftStick;
	public Button cross, circle, triangle, square, up, down, left, right, RB, LB, RT, LT, RS, LS, share, touchpad, options;
	private Gamepad.RumbleEffect rumble2m, rumble1m30s, rumble1m, rumble40s, rumble30s, rumble20s, rumble10s, rumbleCountdown, rumbleConstant, rumbleStop;
	private boolean time2m = false, time1m30s = false, time1m = false, time40s = false, time30s = false, time20s = false, timeCountdown = false;

	
	public Controller(Gamepad gamepad) {
		this.gamepad = gamepad;
		rightStick = new Thumbstick(); leftStick = new Thumbstick();
		
		cross = new Button(); circle = new Button(); triangle = new Button(); square = new Button();
		up = new Button(); down = new Button(); left = new Button(); right = new Button();
		RB = new Button(); LB = new Button(); RS = new Button(); LS = new Button();
		RT = new Button(); LT = new Button(); options = new Button();
		share = new Button(); touchpad = new Button();

		initializeRumble();
	}
	
	
	public void update(){
		rightStick.update(gamepad.right_stick_x, gamepad.right_stick_y); leftStick.update(gamepad.left_stick_x, gamepad.left_stick_y);
		
		cross.update(gamepad.cross); circle.update(gamepad.circle); triangle.update(gamepad.triangle); square.update(gamepad.square);
		up.update(gamepad.dpad_up); down.update(gamepad.dpad_down); left.update(gamepad.dpad_left); right.update(gamepad.dpad_right);
		RB.update(gamepad.right_bumper); LB.update(gamepad.left_bumper); RS.update(gamepad.right_stick_button); LS.update(gamepad.left_stick_button);
		share.update(gamepad.share); touchpad.update(gamepad.touchpad); options.update(gamepad.options);
		
		RT.update(gamepad.right_trigger); LT.update(gamepad.left_trigger);

	}
	
	
	public static class Button {
		private boolean hold = false; private boolean tap = false; private boolean toggle = false; private float rawVal = 0;
		private boolean inputYet = false;
		private int count = 0;
		public ElapsedTime timeHeld = new ElapsedTime();
		public ElapsedTime timeSinceTapped = new ElapsedTime();

		private void update(float trigger){
			rawVal = trigger;
			update(rawVal > .5);
		}

		public void update(boolean button) {
			boolean wasHeld = hold;
			tap = (hold = button) && !wasHeld;
			if (tap) { count++; inputYet = true; timeSinceTapped.reset(); }
			if(!button) timeHeld.reset();

		}

		//this doesn't fully update it's just like real quick for this one thing
		public void updateTap(boolean tap){
			this.tap = tap;
			if (tap) { count++; inputYet = true; timeSinceTapped.reset(); }
		}

		public boolean inputYet(){ return inputYet; }
		
		public float rawVal() { return rawVal; }

		public boolean hold() { return hold; }
		
		public boolean tap() { return tap; }
		
		public boolean toggle() {
			if (tap()) toggle = !toggle;
			return (toggle);
		}

		public void setToggle(boolean newValue){
			toggle = newValue;
		}

		public boolean heldFor(double seconds){
			return timeHeld.seconds() >= seconds;
		}

		public void resetCount(){
			count = 0;
		}
		public int getCount(){
			return count;
		}

		public double range(double pressed, double released){
			double range = pressed - released;
			return (rawVal() * range) + released;
		}

	}

	
	
	public class Thumbstick {
		private double rawX, rawY, shiftedX, shiftedY;

		private Derivative magnitudeDerivative = new Derivative(2);
		
		private void update(Float x, Float y) {
			rawX = x; rawY = y;
			magnitudeDerivative.update(System.currentTimeMillis() / 1000.0, vec().getMagnitude());
		}
		
		public double X() { return rawX * -1; }
		
		public double Y() { return rawY * -1; }

		public double magVelocity(){
			return magnitudeDerivative.getDerivative();
		}

		public Vector2d vec(){
			return new Vector2d(X(), Y());
		}

		public Vector2d shiftedVec(){
			return new Vector2d(shiftedX(), shiftedY());
		}
		
		public void setShift(double shiftAngle) {
			this.shiftedX = (this.rawX * Math.cos(Math.toRadians(shiftAngle))) - (this.rawY * Math.sin(Math.toRadians(shiftAngle)));
			this.shiftedY = (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
		}
		
		public double shiftedX() { return shiftedX * -1; }
		
		public double shiftedY() { return shiftedY * -1; }
		
		
		public double getAngle(){ return ((270 - (Math.atan2(0 - Y(), 0 - X())) * 180 / Math.PI) % 360); }

		public double getDistance(){return Math.abs(distance2D(0, 0, X(), Y()));}

	}



	public void stopRumble(){
		gamepad.stopRumble();
	}

	public void rumble(){
		gamepad.rumble(1, 1, 100);
	}

	public void rumbleTimer(double time){
		double remainingTime = 120 - time;
		if(remainingTime < 120 && !time2m) { gamepad.stopRumble(); gamepad.runRumbleEffect(rumble2m); time2m = true; }
		if(remainingTime < 90 && !time1m30s) { gamepad.stopRumble(); gamepad.runRumbleEffect(rumble1m30s); time1m30s = true; }
		if(remainingTime < 60 && !time1m) { gamepad.stopRumble(); gamepad.runRumbleEffect(rumble1m); time1m = true; }
		if(remainingTime < 40 && !time40s) { gamepad.stopRumble(); gamepad.runRumbleEffect(rumble40s); time40s = true; }
		if(remainingTime < 35 && !time30s) { gamepad.stopRumble(); gamepad.runRumbleEffect(rumble30s); time30s = true; }
		if(remainingTime < 20 && !time20s) { gamepad.stopRumble(); gamepad.runRumbleEffect(rumble20s); time20s = true; }
		if(remainingTime < 10 && !timeCountdown) { gamepad.stopRumble(); gamepad.runRumbleEffect(rumbleCountdown); timeCountdown = true; }
	}



	private void initializeRumble() {
		final int rumbleLong = 700;
		final int rumbleShort = 200;
		final int pauseLong = 300;
		final int pauseShort = 100;

		rumble2m = new Gamepad.RumbleEffect.Builder()
				.addStep(1,1, rumbleLong)
				.addStep(0,0, pauseLong)
				.addStep(1,1, rumbleLong)
				.build();

		rumble1m30s = new Gamepad.RumbleEffect.Builder()
				.addStep(1,1, rumbleLong)
				.addStep(0,0, pauseLong)

				.addStep(1,1, rumbleShort)
				.addStep(0,0, pauseShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0, pauseShort)
				.addStep(1,1, rumbleShort)
				.build();

		rumble1m = new Gamepad.RumbleEffect.Builder()
				.addStep(1,1, rumbleLong)
				.build();

		rumble40s = new Gamepad.RumbleEffect.Builder()
				.addStep(1,1, rumbleShort)
				.addStep(0,0, pauseShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0, pauseShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0, pauseShort)
				.addStep(1,1, rumbleShort)
				.build();

		rumble30s = new Gamepad.RumbleEffect.Builder()
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)

				.addStep(1,1,rumbleShort)
				.addStep(0,0, pauseShort)
				.addStep(1,1,rumbleShort)
				.addStep(0,0, pauseShort)
				.addStep(1,1,rumbleShort)
				.build();

		rumble20s = new Gamepad.RumbleEffect.Builder()
				.addStep(1,1,rumbleShort)
				.addStep(0,0, pauseShort)
				.addStep(1,1,rumbleShort)
				.build();

		rumbleCountdown = new Gamepad.RumbleEffect.Builder()
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1, rumbleShort)
				.addStep(0,0,1000 - rumbleShort)
				.addStep(1,1,2000)
				.build();

		rumbleConstant = new Gamepad.RumbleEffect.Builder()
				.addStep(1,1,1000)
				.build();

		rumbleStop = new Gamepad.RumbleEffect.Builder()
				.addStep(0,0,1)
				.build();
	}
}
