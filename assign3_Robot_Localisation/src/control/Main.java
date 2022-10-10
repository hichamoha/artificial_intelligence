package control;

import models.*;
//import model.*;
import viewer.RobotLocalizationViewer;

public class Main {
	
	public static void main( String[] args) {
		
		/*
		 * generate the state model based on number of rows and columns in the grid, 
		 * the number of headings is fixed to 4
		 * generate you own localiser / estimator wrapper here to plug it into the 
		 * graphics class.
		 */
		//EstimatorInterface l = new DummyLocalizer( new StateModel( 8, 8));
		//Localizer l = new Localizer( new StateModel( 8, 8));
		//EstimatorInterface l = new DummyLocalizer( new StateModel( 5, 5));
		EstimatorInterface l = new HMM_RobotLocaliser( 5, 5, 4);

		RobotLocalizationViewer viewer = new RobotLocalizationViewer( l);

		/*
		 * this thread controls the continuous update. If it is not started, 
		 * you can only click through your localisation stepwise
		 */
		new LocalizationDriver( 500, viewer).start();
	}
}	
