package control;

import viewer.*;
//import view.*;

public class LocalizationDriver2 extends Thread {
	
	private RobotLocalizationViewer l;
	private long timer;
	
	public LocalizationDriver2( long stepTime, RobotLocalizationViewer v) {
		this.l = v;
		this.timer = stepTime;
	}
	
	public void run() {
		while( !isInterrupted()) {
			
			
			try{
				l.updateContinuously();
				sleep( timer);
			} catch( InterruptedException e) {
				System.out.println( "oops");
			}

		}
	}
	
}