package models;


public class HMMFilter {
	/*
	 *  ********************* YOUR TASK ******************
	 *  
	 *  Implement the forward filtering approach 
	 *  based on the transition and sensor models
	 *  
	 *  after one update cycle, the f-vector (probability distribution
	 *  over the states) should be updated
	 */
    public HMMFilter(){
    	//currTrueState = pickNewState();

        //SimpleMatrix OMatrix = pickOMatrix();

        //fmsg = OMatrix.mult(TTM).mult(fmsg);

        //normalizing
    	//double alpha = 1.0 / fmsg.elementSum();
    	//fmsg = fmsg.mult(new SimpleMatrix(new double[][] { { alpha } }));

    }
    
    //public void update(){}
    
}
