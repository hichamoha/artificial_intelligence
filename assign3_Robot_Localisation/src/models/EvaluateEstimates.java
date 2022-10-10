package models;
//import HMM_RobotLocaliser;

import models.HMM_RobotLocaliser;

public class EvaluateEstimates {

	/**
	  * Sets up the world and robot and enters loop of guessing.
	  * Run several instances of observations to
	  * get some more accurate measurement.
	  * n terms of robot localisation it is often not relevant 
	  * to know how often you are 100% correct with your estimate, 
	  * but rather, how far "off" your estimate is from reality 
	  * on average / how often !!!
	  */
	public static void main(String[] args) {
		int nbrObservations = 100; 
		int nbrEpisodes = 20;
		
		System.out.println("Running " + nbrEpisodes + 
					           " episodes with " + nbrObservations + 
					           " observations each");
			
		double manhattan[] = new double[nbrObservations];
		double correctRatio = 0;
		
		for(int run = 0; run < nbrEpisodes; run++) {
			System.out.println("Run\t" + (run+1) + "/" + nbrEpisodes);
			
			//  create robot localiser, which guesses location based on sensor
			HMM_RobotLocaliser robotLoc = 
					                 new HMM_RobotLocaliser( 8, 8, 4);
			double correct = 0;
				
			for(int obs = 0; obs < nbrObservations; obs++) {
				robotLoc.update();
				
				manhattan[obs] += robotLoc.getManhattanDistance() / 
						                   ((double)nbrEpisodes);
					
				if(robotLoc.correctEstimates())
					correct++;
			}
			correctRatio += (correct / 
					         nbrObservations) / 
					         nbrEpisodes;
		}
			
		System.out.println();
		System.out.println("Evaluation of the estimates:");
		System.out.printf("The correct estimates :\t%.5f\n", 
				                                correctRatio);
		System.out.println("Observation:\t\tManhattan Distance");
			
		for(int obs = 0; obs < nbrObservations; obs++)
			System.out.printf((obs+1) + 
				       "\t\t\t%.5f\n", manhattan[obs]);  // average over 20 episodes
	}

}

