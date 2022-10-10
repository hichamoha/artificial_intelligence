package models;
import org.ejml.simple.SimpleMatrix;
//import ejml-simpleMatrix-libs;
//import SimpleMatrix;
import java.util.*;

import control.EstimatorInterface;

public class HMM_RobotLocaliser implements EstimatorInterface {
//public class HMM_RobotLocaliser {

    // compass from 0 to 3 as NORTH-EAST-SOUTH-WEST.
    private static final int NORTH = 0;
	private static final int EAST = 1;
	private static final int SOUTH = 2;
	private static final int WEST = 3;

    // The state is a location/heading pair based on triplets (x,y,h)
    // where x being the row, y the column and h the heading in the grid world
	private int rows, cols, head;
	private State currTrueState;   // the true location L
	
	//Probability distribution over possible locations.
	private double[] probLoc; 
	private State reading;
	
	private int[] currLoc;
	private int currHead;
	
	

	// the four states (headings) that corresponds to one position in the grid 
	// north, east, south, west
	private static final int[][] headings = {{-1,0}, {0,1}, {1,0}, {0,-1}};

    // the directly surrounding 8 fields, i.e. Ls:	
	private static final int[][] Ls = { { -1, -1 }, { -1, 0 }, 
                                        { -1, 1 }, { 0, -1 }, 
                                        { 0, 1 }, { 1, -1 }, 
                                        { 1, 0 }, { 1, 1 } };

    // the second surrounding ring with 16 fields
	private static final int[][] Ls2 = { { -2, -2 }, { -2, -1 }, 
                                         { -2, 0 }, { -2, 1 }, { -2, 2 },
                                         { -1, -2 }, { -1, 2 }, { 0, -2 }, 
                                         { 0, 2 }, { 1, -2 }, { 1, 2 }, 
                                         { 2, -2 }, { 2, -1 }, { 2, 0 }, 
                                         { 2, 1 }, { 2, 2 } };

	// The transition matrix
	private double[][] T;
	private SimpleMatrix TTM, fmsg;
	
	private State[] states;
    // The observation matrix
	private double[][] O;
	private SimpleMatrix[] OMs;
    // "Nothing" reading values
    private double[][] nullReadings;

	/* The forward message in hidden Markov model	
	 * We can think of the filtered estimate P(X_t | e_1:t ) as a 
	 * message f_1:t that is propagated forward along the sequence, 
	 * modified by each transition and updated by each new observation. 
	 * The forward equation is given by
	 * f_1:t+1 = alpha O_t+1 T' f_1:t
	 */
	private double[] f;
	private double[][] fm;
	
	private static final Random rand = new Random();
	
	// ==========================================================
	// Constructor
	// ========================================================== 
    /**
     *  constructor to create the localiser with the current location 
     *   at (0,0) being the top row and the leftmost column in the grid world
     */
    
	public HMM_RobotLocaliser(int rows, int cols, int head) {
		this.rows = rows;
		this.cols = cols;
		this.head = head;
		
		createStateVector(rows, cols);
		
		// ====== TEST states vector ===========
		/*
		System.out.println("nb of elements in states is: " + states.length); // 26
				
		// see first row in O
		for (int i = 0; i < states.length; ++i) {
			System.out.println(i + ": " + states[i]);
		}
		*/
		// generate transition matrix
		//createTmatrix();
		//calculateTmatrix();
		createTransMatrix(rows, cols);
		
		// generate observation matrices
		//createSensorVectors();
		//generateOs();
		createSensorModel();
		//System.out.println("nb of elements in O is:" + OMs.length);
		
		//createNullReadings();
		// ====== TEST nothing readings ===========
		//System.out.println("nb of elements in nullReadings is: " + nullReadings.length); // 26
		
		// see first row in O
		//for (int i = 0; i < nullReadings.length; ++i) {
		//	System.out.println(i + ": " + nullReadings[0][i]);
		//}
			
			
		// Initialize the true position (randomly)
		currLoc = new int[2];
		//currLoc[0] = 0;
		currLoc[0] = rand.nextInt(rows);
		//currLoc[1] = 0;
		currLoc[1] = rand.nextInt(cols);

		//currHead = EAST;
		currHead = rand.nextInt(head);
				
		currTrueState = new State(currLoc[0], currLoc[1], currHead);
		//System.out.println("The initial position is: " + 
		//                    currTrueState.toString());
		
		// Initialize Reading
		//reading = currentReading(currTrueState);
		
		// initialize vector of forward values f
		//init_f();
		// initialize the vector forward message
		double[][] forwardMessage = new double[rows * cols * 4][1];

		//Forward message is uniform over all states(including heading)
		for (int i = 0; i < forwardMessage.length; i++)
			forwardMessage[i][0] = (1.0 / (rows * cols * 4));
		
		// forward message in matrix form
		fmsg = new SimpleMatrix(forwardMessage);
		
		// ========== TEST forward matrix ==========================
		// =========================================================
										
		//System.out.println("nb of elements in init-f is: " + f.length); 
														
		// see first row in O
		//for (int i = 0; i < f.length; ++i) {
		//	System.out.println(i + "init-f: " + f[i]);
		//}
        // ==========================================================
		
		//fm = new double[rows][cols];
		
		//probLoc is uniform over possible locations(not including heading)
		probLoc = new double[rows * cols];
		//probLoc = new double[rows * cols*4];
		Arrays.fill(probLoc, (1.0 / (rows * cols)));
		
		
	}
	
	private void createSensorModel() {
		// Possible outputs is each square + nothing output
		OMs = new SimpleMatrix[rows * cols + 1];
		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < cols; c++) {
				// Reporting location row r col c.
				State s = new State(r, c, 0); 
				OMs[getStateNbr(s) / 4] = new SimpleMatrix(sensorMatrix(s));
			}
		}
		double[][] nothing = new double[rows * cols * 4][rows * cols * 4];
		for (int r = 0; r < rows; r++)
			for (int c = 0; c < cols; c++)
				for (int h = 0; h < 4; h++) {
					State s = new State(r, c, h);
					nothing[getStateNbr(s)][getStateNbr(s)] = computeNullSensing(s);
				}
		OMs[OMs.length - 1] = new SimpleMatrix(nothing);
		
	}

	/**
     * "Nothing" probability which consider the 
     * blind robot in limited sensing case
     * Computes the probability for the sensor
     * reporting nothing in this state.
     * @return The probability for blind sensor
     */
	private double computeNullSensing(State s) {
		return 1.0 - 0.1 - nb_Ls(s.getX(), s.getY()) * 0.05 - nb_Ls2(s.getX(), s.getY()) * 0.025;
	}

	private double[][] sensorMatrix(State s) {
		double[][] matrix = new double[states.length][states.length];
		for(int i = 0; i < matrix.length; i++)
			matrix[i][i] = sensorReports(s, states[i]);
		return matrix;
	}

	private double sensorReports(State report, State st) {
		int xdiff = Math.abs(report.getX() - st.getX());
		int ydiff = Math.abs(report.getY() - st.getY());
		if(xdiff > 2 || ydiff > 2) //Max 2:th ring
			return 0;
		else if(xdiff == 2 || ydiff == 2)
			return 0.025;
		else if(xdiff == 1 || ydiff == 1)
			return 0.05;
		else //Must be 0 ring
			return 0.1;
	}
	
	private void createTransMatrix(int rows, int cols) {
		T = new double[rows * cols * 4][rows * cols * 4];
		double[][] ttm = new double[rows * cols * 4][rows * cols * 4];
		for (int i = 0; i < states.length; i++)
			for (int j = 0; j < states.length; j++) {
				//compute probability for going from state i to state j
				//T[i][j] = computeTransProb(states[i], states[j]);
				T[i][j] = getNewStepProb(states[i].getX(), states[i].getY(), states[i].getH(),
						                 states[j].getX(), states[j].getY(), states[j].getH());
				ttm[j][i] = T[i][j]; //Transpose
			}
		TTM = new SimpleMatrix(ttm);
	}
	
	/**
	 * Returns the average manhattan distance
	 * to measure the distance between true location and estimate
	 * @return manhattan distance
	 */
	public int getManhattanDistance() {
		State estimate = getEstimate();
		
		int Xdiff = Math.abs(currTrueState.getX() - estimate.getX());
		int Ydiff = Math.abs(currTrueState.getY() - estimate.getY());
		return Xdiff + Ydiff;
	}

	/**
	 * Get the state location with the highest probability.
	 * @return best estimated state 
	 */
	private State getEstimate() {
		State optima = states[0];
		double bestValue = probLoc[0];
		
		for (int i = 0; i < probLoc.length; i++) {
			if (probLoc[i] > bestValue) {
				optima = states[4 * i];
				bestValue = probLoc[i];
			}
		}
		return optima;
	}
	
	public boolean correctEstimates() {
		State estimate = getEstimate();
		return estimate.getX() == currTrueState.getX() && estimate.getY() 
				               == currTrueState.getY();
	}

	private double computeTransProb(State st, State target) {
		// zero probability to move more than one step
		int rowdiff = Math.abs(st.getX() - target.getX());
		int coldiff = Math.abs(st.getY() - target.getY());
		int manhattan = rowdiff + coldiff;
				
		// Must move one step
		if(manhattan != 1 || !movingTowards(st, target))
			return 0;
				
		List<Integer> goodMoves = goodMoves(st);
		if(!goodMoves.contains(target.getH())) //Don't do anything stupid
			return 0;
		if(!facesWall(st)) {
			//If i don't face wall continue with possibility 0.7 and
			//distribute uniform 0.3 on other good moves
			return st.getH() != target.getH() ? 0.3 / (goodMoves.size() - 1) : 0.7;
		} else {
			//distribute uniform 1.0 on good moves
			return 1.0 / goodMoves.size();
				}
	}

	private List<Integer> goodMoves(State s) {
		List<Integer> moves = new LinkedList<Integer>();
		for(int h = 0; h < 4; h++) {
			int nrow = s.getX() + headings[h][0];
			int ncol = s.getY() + headings[h][1]; 
			if(!(nrow < 0 || nrow >= rows || ncol < 0 || ncol >= cols))
				moves.add(h);
		}
		return moves;
	}

	private boolean movingTowards(State st, State target) {
		int nrow = st.getX() + headings[target.getH()][0];
		int ncol = st.getY() + headings[target.getH()][1]; 
		return nrow == target.getX() && ncol == target.getY();
	}

	private boolean facesWall(State st) {
		int nrow = st.getX() + headings[st.getH()][0];
		int ncol = st.getY() + headings[st.getH()][1]; 
		return (nrow < 0 || nrow >= rows || ncol < 0 || ncol >= cols);
	}

	private void createStateVector(int rows, int cols) {
		states = new State[rows * cols * 4]; // 4 headings per position
		
		for (int r = 0; r < rows; r++)
			for (int c = 0; c < cols; c++)
				for (int h = 0; h < 4; h++) {
					State s = new State(r, c, h);
					states[getStateNbr(s)] = s;
				}
	}

    /**
	 * return the number of assumed rows, columns and possible headings for the grid
	 * a number of four headings makes obviously most sense... the viewer can handle 
	 * four headings at maximum in a useful way.
	 */
    public int getNumRows() {
    	return rows;
	}

	public int getNumCols() {
		return cols;
	}

	public int getNumHead() {
		return head;
	}

	// ==========================================================
	// Setting up the sensor model
	// ========================================================== 	
	
	
	public State currentReading(State Pos) {
		
		// get possible next Positions
		ArrayList<State> positions = new ArrayList<State>();
		
		int stateIndex = OstateIndex(Pos);

		for (int read = 0; read < rows * cols + 1; read ++) {
			
			double chance = O[read][stateIndex];
			
			if(chance > 0.001) {
				int percentage = Math.round((int)(chance * 100));
				for (int i = 0; i < percentage; i++) 
					positions.add(readingPosition(read));			
			}
		}
		
		// choose the next position randomly
		return positions.get( new Random().nextInt(positions.size()));		
	}
	
	
	
    /**
     * create the sensor model in matrix form, according to the noisy model:
     * P(e_t | X_t = i) for each state i.
     * For mathematical convenience we place each of these values into an S * S
     * diagonal matrix.
     */
//	
	private void generateOs() {
		int m = rows*cols+1;
		//int n = rows*cols;
		int n = rows*cols*4;
		O = new double[m][n];
		
		for (int row = 0; row < m; row ++)
			for (int col = 0; col < n; col ++)
				O[row][col] = 0.0;
		
		for (int read = 0; read < m; read++) {
			
			for (int state = 0; state < n; state++) {
				State statePos = OstatePosition(state);
				State readingPos = readingPosition(read);
//
				OsetElementAt(read, 
						      state, 
						      generateOrXY(readingPos.getX(), 
                                      readingPos.getY(),  
                                      statePos.getX(), 
                                      statePos.getY(), 
                                      statePos.getH()));
//
/*				
				OsetElementAt(read, state,
                              calculateOrXY(readingPos.getX(), 
                                            readingPos.getY(),  
                                            statePos.getX(), 
                                            statePos.getY(), 
                                            statePos.getH()));
*/
			}
		}
		// ========== TEST O matrix =========================================
		// =========================================================
		System.out.println(calculateOrXY(0, 0, 1, 0, SOUTH)); // 0.05
		System.out.println(getOrXY(0, 0, 1, 0, SOUTH)); // 0.05
								
		State pose_i = new State(0, 0, NORTH);
		State pose_j = new State(1, 0, SOUTH);
		System.out.println(OstateIndex(pose_i));  // 0
		System.out.println(OstateIndex(pose_j));  // 6
		//System.out.println(getEntryAt(pose_i, pose_j )); // 0.5
								
		System.out.println("nb of elements in O is: " + O.length); // 26
		double diagO[][] = toDiagonal(O[0]);
		System.out.println("nb of elements in O diagonal is: " +
				            diagO.length  ); 
								
		// see first row in O
		//for (int i = 0; i < O.length-1; ++i) {
		for (int i = 0; i < rows*cols; ++i) {
			System.out.println(i + ": " + O[0][i]);
		}
	}
//
	
	private double generateOrXY(int rX, int rY, int x, int y, int h) {
		// Probability that sensor reading r = (rX,rY) 
		// has been caused by respective state (x,y,h)
		
		// reading is equal
		if (isReadingCorrect(rX,rY,x,y)) return 0.1;
		
		// reading is surrounding field
		if (is_Ls(rX,rY,x,y)) return 0.05;
		
		// reading is secondary surrounding field 
		if (is_Ls2(rX,rY,x,y)) return 0.025;
		
		// reading is nothing
		if (isReadingNothing(rX,rY))
			return 1.0 - 0.1 - nb_Ls(x,y) * 0.05 - nb_Ls2(x,y) * 0.025;
		
		return 0.0;
	}
//	
	private void createSensorVectors() {
		//O = new double[rows*cols+1][rows*cols];
		O = new double[rows*cols+1][rows*cols*4];
		
		int m = rows*cols+1;
		int n = rows*cols*4;
		O = new double[m][n];
		
		for (int row = 0; row < m; row ++)
			for (int col = 0; col < n; col ++)
				O[row][col] = 0.0;

		//for (int i = 0; i < O.length - 1; ++i) {
		for (int i = 0; i < O.length ; ++i) {
			int index = 0;

			for (int x = 0; x < rows; ++x) {
			//for (int x = 0; x < m; ++x) {
				for (int y = 0; y < cols; ++y) {
				//for (int y = 0; y < n; ++y) {
					int distance = (int) Math.sqrt(Math.pow(x - i / rows, 2) + 
							                       Math.pow(y - i % rows, 2));

					switch(distance) {
                        // the true location L
						case 0:	O[i][index++] = 0.1; 
							break;

						// the directly surrounding fields Ls
						case 1: O[i][index++] = 0.05;
							break;

						// the scondary surrounding fields Ls2
						case 2: O[i][index++] = 0.025;
							break;

						default: O[i][index++] = 0.0;
					}
				}
			}
			
		}
		// ========== TEST O matrix =========================================
		// =========================================================
		System.out.println(calculateOrXY(0, 0, 1, 0, SOUTH)); // 0.05
		System.out.println(getOrXY(0, 0, 1, 0, SOUTH)); // 0.05
						
		State pose_i = new State(0, 0, NORTH);
		State pose_j = new State(1, 0, SOUTH);
		System.out.println(OstateIndex(pose_i));  // 0
		System.out.println(OstateIndex(pose_j));  // 6
		//System.out.println(getEntryAt(pose_i, pose_j )); // 0.5
		
		System.out.println("nb of elements in O is: " + O.length); // 26
		double diagO[][] = toDiagonal(O[0]);
		System.out.println("nb of elements in O diagonal is: " +
				            diagO.length  ); 
						
		// see first row in O
		for (int i = 0; i < O.length; ++i) {
			System.out.println(i + ": " + O[0][i]);
		}
	}

	/** 
	 * compute the probability to give "nothing" reading for each cell 
	 * and put that data in a matrix of the same dimensions as the grid.
     * a sensor reading of nothing is slightly 
     * more likely to get when robot is close to a wall. 
	 */
	private void createNullReadings() {
		int index = 0;
		double sum = 0;

		nullReadings = new double[rows][cols];

		for (int x = 0; x < rows; ++x) {
			for (int y = 0; y < cols; ++y) {
				double value = 0.1;
                                
                // when L is at least 2 fields away from any wall
				int yRelevantSquares = 5;

				if (x <= 1 || x >= rows-2) {
					value += 0.025 * 5;
					yRelevantSquares--;

					if (x < 1 || x > rows-2) {
						value += 0.05 * 3 + 0.025 * 2;
						yRelevantSquares--;
					}
				}

				if (y <= 1 || y >= cols-2) {
					value += 0.025 * yRelevantSquares;

					if (y < 1 || y > cols-2) {
						value = (yRelevantSquares < 4) ? value + 0.05 * 2 : 
							                             value + 0.05 * 3;
						value = (yRelevantSquares < 5) ? value + 0.025 * 2 : 
							                             value + 0.025;
					}
				}

				nullReadings[x][y] = value;

				double prob = 1 / (rows*cols) * value;
				sum += prob;
				//System.out.println("Sum null: " + sum); // 0
				O[O.length - 1][index++] = prob;
			}
		}

		double scalar = 1 / sum;

		for (int i = 0; i < O[O.length - 1].length; ++i) {
			O[O.length - 1][i] *= scalar;
		}
		
		// ========== TEST null readings matrix =========================================
		// =========================================================
						
		System.out.println("nb of elements in null is: " + nullReadings.length); // 26
										
		// see first row in O
		for (int i = 0; i < nullReadings.length; ++i) {
			System.out.println(i + "null: " + nullReadings[0][i]);
		}
	}
	
	// ==========================================================
	//	Setting up the transition model
	// ========================================================== 
	
	/**
     * Create the transition model in Matrix form:
	 * P(X_t | X_t-1)
	 * is represented by an S * S matrix T where
	 * T_ij = P(X_t = j | X_t-1 = i).
     */
	private void createTmatrix() {
		T = new double[rows * cols * head][rows * cols * head];
		int gridsize = rows * cols;

		int iHeading = NORTH;
		int jHeading = NORTH;

		// get the coordinates/states from the indexes
		// To get right column you have: i % NUMBER_ITEMS_IN_ROW
		// And to get right row: i / NUMBER_ITEMS_IN_ROW
		for (int i = 0; i < T.length; ++i) {
		//for (int i = 0; i < g; ++i) {
			int iX = i / (rows * head);      // convert index to coordinates 
			int iY = i % (rows * head);

			for (int j = 0; j < T[i].length; ++j) {
				int jX = j / (rows * head);
				int jY = j / head % (rows);
				//int jY = j % (rows * head);
				
				// see the all the targets from first square (0,0,0) 
				// in this way we get the first sequence/row in T matrix
				if (i == 2) {
					System.out.println(" FROM " + "ix: " + iX + 
							           ", iy: " + iY +
							           ", iHead: " + iHeading + " TO " + 
							           ", jx: " + jX + 
							           ", jy: " + jY + 
							           ", jHead: " + jHeading);
				}
				
				//T[i][j] = getTProb(iX, iY, iHeading, jX, jY, jHeading);
				T[i][j] = getNewStepProb(iX, iY, iHeading, jX, jY, jHeading);
				
				jHeading = (jHeading == WEST) ? NORTH : jHeading + 1;
			}

			iHeading = (iHeading == WEST) ? NORTH : iHeading + 1;
		}
		
		// ========== TEST =========================================
		// =========================================================
		System.out.println(getNewStepProb(0, 0, NORTH, 1, 0, SOUTH)); // 0.5
		System.out.println(getTProb(0, 0, NORTH, 1, 0, SOUTH)); // 0.0
		
		State pose_i = new State(0, 0, NORTH);
		State pose_j = new State(1, 0, SOUTH);
		System.out.println(stateIndex(pose_i));
		System.out.println(stateIndex(pose_j));
		System.out.println(getEntryAt(pose_i, pose_j )); // 0.0
		
		System.out.println("nb of elements in T is: " + T.length); // 100
		
		// see first row in T
		for (int i = 0; i < rows * cols * 4; ++i) {
			System.out.println(i + ": " + T[0][i]);
		}
		// deepToString() This method is designed for converting 
		// multidimensional arrays to strings.
		//System.out.println(" the transition matrix T = ");
		//System.out.println(Arrays.deepToString(T).replace("]","]\n"));
	}
	
	private void calculateTmatrix() {
		T = new double[rows * cols * head][rows * cols * head];
		int gridsize = rows * cols;

		int iHeading = NORTH;
		int jHeading = NORTH;
		
		for (int i = 0; i < T.length; i++) {
			State state = statePosition(i);
			
			for (int j = 0; j < T[i].length; ++j) {
				State target = statePosition(j);
				//System.out.println(target);
				
				
				// see the all the targets from first square (0,0,0) 
				// in this way we get the first sequence/row in T matrix
				if (i == 0) {
					System.out.println(" FROM " + "ix: " + state.getX() + 
					           ", iy: " + state.getY() +
					           ", iHead: " + state.getH() + " TO " + 
					           ", jx: " + target.getX() + 
					           ", jy: " + target.getY() + 
					           ", jHead: " + target.getH());
				}
				
				//T[i][j] = getNewStepProb(iX, iY, iHeading, jX, jY, jHeading);
				T[i][j] = getNewStepProb(state.getX(), state.getY(), state.getH(), 
						                 target.getX(), target.getY(), target.getH());
				jHeading = (jHeading == WEST) ? NORTH : jHeading + 1;
			}
			iHeading = (iHeading == WEST) ? NORTH : iHeading + 1;
		}
		
		// ========== TEST T matrix =========================================
		// =========================================================
		System.out.println(getNewStepProb(0, 0, NORTH, 1, 0, SOUTH)); // 0.5
		System.out.println(getTProb(0, 0, NORTH, 1, 0, SOUTH)); // 0.5
				
		State pose_i = new State(0, 0, NORTH);
		State pose_j = new State(1, 0, SOUTH);
		System.out.println(stateIndex(pose_i));  // 0
		System.out.println(stateIndex(pose_j));  // 22
		System.out.println(getEntryAt(pose_i, pose_j )); // 0.5
				
		System.out.println("nb of elements in T is: " + T.length); // 100
				
		// see first row in T
		for (int i = 0; i < rows * cols * 4; ++i) {
			System.out.println(i + ": " + T[0][i]);
		}
		// deepToString() This method is designed for converting 
		// multidimensional arrays to strings.
		//System.out.println(" the transition matrix T = ");
		//System.out.println(Arrays.deepToString(T).replace("]","]\n"));
		
		
	}
	
	private void init_f() {
		f = new double[rows * cols * head];
		int s = rows * cols * head;
		//f[currHead] = 1;
		//
		//Forward message is uniform over all states(including heading)
		for (int i = 0; i < f.length; i ++) {
			f[i] = 1.0/s;
			//f[i] = 1.0;
	    }
	    //
		//f[currHead] = 1.0/s;
	}
	
	public double[][] setScalarMatrix(double scalar) {
		double[][] mat = new double[rows][cols];
		for (int row = 0; row < rows; row ++) {
			for (int col = 0; col < cols; col ++) {
				mat[row][col] = scalar;
			}
		}
		return mat;
	}
	
    /**
     * Compute matrix indexes from coordinates
     * Returns the element at the specified row/col position in this grid.
     * @param i
     * @param j
     * @return
     */
	//public double getEntryAt(double[][] a, State i, State j) {
	public double getEntryAt(State i, State j) {
		int pose_i = stateIndex(i.getX(), i.getY(), i.getH());
		int pose_j = stateIndex(j.getX(), j.getY(), j.getH());
		
		//return getElementAt(a, pose_i, pose_j);
		return T[pose_i][pose_j];
				
	}
	
	/*
	 * Get a single element in the transition matrix.
	 */
	//public double getElementAt(double[][] a, int x, int y) {
		//return a[x][y];
	//}
	/**
	 * returns the probability entry (Tij) of your transition model T to go from
	 * pose i = (x, y, h) to pose j = (nX, nY, nH)
	 */
	public double getTProb(int x, int y, int h, 
			               int nX, int nY, int nH) {
		//return getEntryAt(T, new State(x,y,h), new State(nX,nY,nH));
		return getEntryAt(new State(x,y,h), new State(nX,nY,nH));
		//Position i = new Position(x,y,h);
		
		//Position j = new Position(nX,nY,nH);
		//return T[x][y];
		
	}
	
	/*
	 * returns the transition matrix T, in which each entry (Tij) contains 
	 * the probability to go from pose 
	 * i = (x, y, h) to pose j = (nX, nY, nH)
	 */	
	
	//public TransitionMatrix getTMatrix() {}
	
	/* Robot movement strategy: 
	 * For any new step pick new heading h_t+1 
	 * based on the current heading h_t according to:
	 * P( h_t+1 = h_t | not encountering a wall) = 0.7
	 * P( h_t+1 != h_t | not encountering a wall) = 0.3
	 * P( h_t+1 = h_t | encountering a wall) = 0.0
	 * P( h_t+1 != h_t | encountering a wall) = 1.0
	 */
	//public double getTProb(int x, int y, int h, int nX, int nY, int nH) {
	public double getNewStepProb(int x, int y, int h, int nX, int nY, int nH) {
		int possibleDirs = 4;
		
		// facing bottom horizontal wall
		if (!notFacingWall(x, y, -1, 0)) {
			--possibleDirs;
		} 
        
		// facing top horizontal wall
		if (!notFacingWall(x, y, 1, 0)) {
			--possibleDirs;
		}
 
		// facing left vertical wall
		if (!notFacingWall(x, y, 0, -1)) {
			--possibleDirs;
		} 

		// facing right vertical wall
		if (!notFacingWall(x, y, 0, 1)) {
			--possibleDirs;
		} 
		
		// in case of not encountering a wall
		if ((x + headings[nH][0] == nX) && (y + headings[nH][1] == nY)) {
			// and having the same direction h_t+1 = h_t
			if (nH == h) {
				return 0.7;
				
			// otherwise in case of different direction h_t+1 != h_t
			} else {
				
				if (!notFacingWall(x, y, headings[h][0], headings[h][1])) {
					return 1.0/(possibleDirs);
				}

				return 0.3/(possibleDirs - 1);
			}
		}
		
		return 0.0;
	}
	


	/**
	 * returns the probability entry of the sensor matrices O 
	 * to get reading r corresponding to position (rX, rY)
	 * when actually in position (x, y) (note that you have to take 
	 * care of potentially necessary transformations from states i = <x, y, h> to 
	 * positions (x, y)). If rX or rY (or both) are -1, the method should return the probability for 
	 * the sensor to return "nothing" given the robot is in position (x, y).
	 */	
//
	public double getOrXY(int rX, int rY, int x, int y, int h) {
		//return O[readingIndex(rX,rY)][stateIndex(x,y,h)];
	    //return OgetElementAt(readingIndex(rX,rY), OstateIndex(x,y,h));
		State r = new State(x, y);
		int rStateNbr = getStateNbr(r);
		if (rX == -1 || rY == -1)
			return OMs[OMs.length - 1].get(rStateNbr, rStateNbr);
		
		State s = new State(rX, rY, h);
		return OMs[getStateNbr(s) / 4].get(rStateNbr, rStateNbr);
	}
	
	/*
	 * returns the collection of sensor matrices O_r containing 
	 * the probabilities to get reading r corresponding 
	 * to position (rX, rY) when actually in position (x, y)
	 */
	
	//public SensorMatrices getObservationMatrices() {}
	
//
	
	public double[][] getO(State reading) {
		
		int readingIndex = readingIndex(reading.getX(), reading.getY());
		int s = rows*cols*4;
		double[] diag = new double[s];
		
		for (int i = 0; i < s; i++)
				diag[i] = OgetElementAt(readingIndex,i);
		
		return Matrix(diag);
	}
	
	public double[][] Matrix(double[] diag) {
		rows = diag.length;
		cols = diag.length;
		double[][] mat = new double[rows][cols];
		
		for (int row = 0; row < rows; row ++) {
			for (int col = 0; col < rows; col ++) {
				mat[row][col] = (row == col) ? diag[row] : 0.0;
			}
		}
		return mat;
		
	}
	
	//public double getOrXY(int rX, int rY, int x, int y, int h) {
	public double calculateOrXY(int rX, int rY, int x, int y, int h) {
		// If rX, rY = -1; nothing reading
//		
		if (rX == -1 || rY == -1) {
			//return nullReadings[x][y];
			return 1 - 0.1 - nb_Ls(x,y)*0.05 - nb_Ls2(x,y)*0.025;
		}
//
		int distance = (int) Math.sqrt(Math.pow(x - rX, 2) + 
				                       Math.pow(y - rY, 2));

		switch (distance) {
			case 0: return 0.1;
			case 1: return 0.05;
			case 2: return 0.025;
			default: return 0.0;
		}
	}
	
	private int nb_Ls2(int x, int y) {
		if (isInCorner(x,y)) return 5;
		if (isNextToCorner(x,y)) return 6;
		if (isDiagonalToCorner(x,y)) return 7;
		if (isTwoAwayFromCorner(x,y)) return 9;
		if (isTwoAndOneAwayFromWall(x,y)) return 11;
		return 16;
	}
	
	private boolean isNextToCorner(int x, int y) {
		if (isNextToWall(x,y) && (x == 1 || x == rows - 2)) return true;
		if (isNextToWall(x,y) && (y == 1 || y == cols - 2)) return true;
		return false;
	}
	
	private boolean isDiagonalToCorner(int x, int y) {
		if (x == 1 && y == 1) return true;
		if (x == 1 && y == cols - 2) return true;
		if (x == rows -2 &&  y == 1) return true;
		if (x == rows -2 && y == cols - 2) return true;
		return false;
	}
	
	private boolean isTwoAwayFromCorner(int x, int y) {
		if (isNextToWall(x,y) && (x == 2 || x == rows - 3)) return true;
		if (isNextToWall(x,y) && (y == 2 || y == cols - 3)) return true;
		return false;
	}
	
	private boolean isTwoAndOneAwayFromWall(int x, int y) {
		if (x == 1 && (y == 2 || y == cols - 3)) return true;
		if (x == 2 && (y == 1 || y == cols - 2)) return true;
		if (x == rows - 3 && (y == 1 || y == cols - 2)) return true;
		if (x == rows - 2 && (y == 2 || y == cols - 3)) return true;
		return false;
	}
	
	private int nb_Ls(int x, int y) {
		if (isInCorner(x,y)) return 3;
		if (isNextToWall(x,y)) return 5;
		//if (notFacingWall(x,y,h)) return 5;
		return 8;
	}
	
	private boolean isInCorner(int x, int y) {
		if (x == 0 && y == 0) return true;
		if (x == 0 && y == cols-1) return true;
		if (x == rows-1 && y == 0 ) return true;
		if (x == rows-1 && y == cols-1) return true;
		return false;
	}
	
	private boolean isNextToWall(int x, int y) {
		if (x == 0 	|| x == rows - 1) return true;
		if (y == 0 	|| y == cols - 1) return true;
		return false;
	}
	
	private boolean isReadingCorrect(int rX, int rY, int x, int y ) {
		return (rX == x && rY == y);
	}
	
	private boolean isReadingNothing(int rX, int rY) {
		return (rX == -1 && rY == -1);
	}
	
	private boolean is_Ls(int rX, int rY, int x, int y) {
		if (isReadingNothing(rX,rY)) return false;
		int vertDist = Math.abs(rX-x); // Manhattan = vertDist + horDist
		int horDist = Math.abs(rY-y);
		if (vertDist == 1 && horDist == 0) return true;
		if (vertDist == 1 && horDist == 1) return true;
		if (vertDist == 0 && horDist == 1) return true;
		return false;
	}
	
	private boolean is_Ls2(int rX, int rY, int x, int y) {
		if (isReadingNothing(rX,rY)) return false;
		int vertDist = Math.abs(rX - x);
		int horDist = Math.abs(rY - y);
		if (vertDist == 2 && horDist == 2) return true;
		if (vertDist == 2 && horDist == 1) return true;		
		if (vertDist == 2 && horDist == 0) return true;
		if (vertDist == 1 && horDist == 2) return true;
		if (vertDist == 0 && horDist == 2) return true;
		return false;
	}

	/**
	 * returns the currently known true position i.e., after one simulation step of
	 * the robot as (x,y)-pair.
	 */
	public int[] getCurrentTrueState() {
		//return currLoc;
		return new int[] {currTrueState.getX(), 
                          currTrueState.getY() };
	}
	
	public int getCurrentHeading() {
		return currHead;
	}

	/**
	 * returns the currently available sensor reading obtained 
	 * for the true position after the simulation step
     * returns null if the reading was "nothing" (whatever that stands for in your model)
	 */
//	
	public int[] getCurrentReading() {
		return reading == null ? null : new int[] {reading.getX(), 
				                                   reading.getY() };
	}
//	
	//public int[] getCurrentReading() {
	public int[] calculateCurrentReading() {
		// Returns a pseudorandom, uniformly distributed int value 
		// between 0 (inclusive) and the specified value (exclusive), 
		// drawn from this random number generator's sequence.
		int rand = new Random().nextInt(40);
		//System.out.println(" pseudorandom : " + rand);
		
		int[] currRead = null;

		if (rand < 4) {
			// the true position
			currRead = currLoc;
		} else if (rand < 20) {
			// the directly surrounding fields
			int cellIndex = new Random().nextInt(Ls.length);
			int dx = Ls[cellIndex][0];
			int dy = Ls[cellIndex][1];

			if (notFacingWall(currLoc[0], currLoc[1], dx, dy)) {
				currRead = new int[2];
				currRead[0] = currLoc[0] + dx;
				currRead[1] = currLoc[1] + dy;
			} // else keep currRead = null

		} else if (rand < 36) {
			// the second surrounding fields
			int cellIndex = new Random().nextInt(Ls2.length);
			int dx = Ls2[cellIndex][0];
			int dy = Ls2[cellIndex][1];

			if (notFacingWall(currLoc[0], currLoc[1], dx, dy)) {
				currRead = new int[2];
				currRead[0] = currLoc[0] + dx;
				currRead[1] = currLoc[1] + dy;
			} 
		} 
		return currRead;
	}

	/**
	 * returns the currently estimated (summed) probability for the robot to be in
	 * position (x,y) in the grid. The different headings are not considered, as it
	 * makes the view somewhat unclear.
	 */
	public double getCurrentProb(int x, int y) {
		//System.out.println(fm[x][y]);
		//return fm[x][y];
		//return probForPos(new State(x,y));
		State s = new State(x, y, 0);
		return probLoc[getStateNbr(s) / 4];
		//return probForPos(s);
	}
	
	/*
	 * returns the currently estimated (summed) probability for the robot to be in position
	 * (x,y) in the grid. The different headings are not considered, as it makes the 
	 * view somewhat unclear.
	 */
	//public double[] getCurrentProbDist() {}
	
	/**
	 * Indexing of the state
	 * @return unique nbr enumerating this state
	 */
	public int getStateNbr(State s) {
		//return s.getX() * cols * 4 + s.getY() * 4 + s.getH();
		return s.getX() * 4 + s.getY() * rows * 4 + s.getH();
	}
	
	/**
     * The matrix based Forward Filtering algorithm
     * using simple matrix-vector operations.
     * function to perform forward calculation of probability 
     * using matrix algorithm
     * v[t] = normlize(sensor*transitionTranspose*v[t-1])
     */ 
	private void HMMforward() {
		
		// update reading
		//int[] reading = getCurrentReading();
		//reading = currentReading(currTrueState);
		//int index = rows * cols;
        /*
		if (reading != null) {
			 //index = reading[0] / rows + reading[1] % rows;
			index = reading.getX() / rows + reading.getY() % rows;
			 
		}
		*/
		
		// Compute probability to be in (x, y) for all x, y. 
		// Summing over possible headings
		computeProbLoc();
		//reshape_f();
		
	}
	
	
	/*
	 * NOTE: the filtering algorithm only knows sensor readings, 
	 * which only represent positions in the grid, i.e. 
	 * one sensor reading is equally likely for the four states (headings) 
	 * that correspond to one position in the grid. 
	 * thus, the values within each bundle of four consecutive entries 
	 * along the observation matrix diagonal should be the same.
	 * */
	private void reshape_f() {
		int x = 0;
		int y = 0;
 
		for (int i = 0; i < f.length; i += 4) {
			for (int j = 0; j < 4; ++j) {
				
				//fm[x][y] += f[i+j];
				fm[x][y] += f[i+j];
			}

			x = (x < cols - 1) ? x + 1 : 0;

			if (x == 0) {
				++y;
			}
		}
		
		//normalize(fm);
		
		// Compute probability to be in (x, y) for all x, y. 
		// Summing over possible headings
		//computeProbLoc();
		
		// ========== TEST forward matrix =========================================
		// =========================================================
		/*								
		System.out.println("nb of elements in fm is: " + fm.length); 
														
		// see first row in O
		for (int i = 0; i < fm.length; ++i) {
			System.out.println(i + "fm: " + fm[i][1]);
		}
		*/
	}
	
	/**
	 * should trigger one step of the estimation, i.e., true position, sensor
	 * reading and the probability distribution for the position estimate should be
	 * updated one step after the method has been called once.
	 */
	public void update() {
	//public void aggiornare() {
		// Update to new state
		currTrueState = pickNewState();
		//System.out.println("True Position: " + currTrueState.toString());
				
		// Select O Matrix from the sensor input
		//double[] oMatrix = pickOMatrix();
		SimpleMatrix OMatrix = pickOMatrix();
		
		
		// Compute forward message
		//f = multiplyM(multiplyM(toDiagonal(oMatrix), transposeM(T)), f);
		//f = multiplyM(multiplyM(getO(reading), transposeM(T)), f);
		fmsg = OMatrix.mult(TTM).mult(fmsg);
		
		// ========== TEST forward matrix =========================================
		// =========================================================
		/*								
		System.out.println("nb of elements in f is: " + fmsg.numRows()); 
														
		// see first row in O
		for (int i = 0; i < fmsg.numRows(); ++i) {
			System.out.println(i + "f: " + fmsg.get(i));
		}
		*/
		
		//normalizing
		double alpha = 1.0 / fmsg.elementSum();
		fmsg = fmsg.mult(new SimpleMatrix(new double[][] { { alpha } }));
		
		// Compute probability to be in (x, y) for all x, y. 
		// Summing over possible headings
		computeProbLoc();
	}
	
	//public boolean update( int[] trueState, int[] sensedPos) {return false;}
//	
	public void aggiornare() {
	//public void update() {
		// make move and update true position
		//currTrueState = nextPosition(currTrueState);
		 System.out.println("True Position: " + currTrueState.toString());
		
		// update reading
		reading = currentReading(currTrueState);
		// System.out.println("Current Reading: " + Reading.toString());
		
		// update probability distribution for the position estimate
		//f.forwardPrediction(getO(reading), getT());
		// Compute forward message
		f = multiplyM(multiplyM(getO(reading), transposeM(T)), f);
	}

//
	
	public State nextPosition(State oldPosition) {
		
		// initialize ArrayList for possible next Positions
		ArrayList<State> positions = new ArrayList<State>();
		
		int oldindex = stateIndex(oldPosition);
		
		// add positions according to probability to ArrayList
		for (int nextindex = 0; nextindex < rows*cols*4; nextindex++) {
			
			double chance = T[oldindex][nextindex];
			
			if(chance > 0.001) {
				int percentage = Math.round((int)(chance * 100));
				for (int i = 0; i < percentage; i++) 
					positions.add(statePosition(nextindex));			
			}
			
		}
		System.out.println("position list size is: " + positions.size());
		
		// choose the next position according to the possibilities
		return positions.get( new Random().nextInt(positions.size()));
	}
	
	/**
	 * Returns the new true state
	 * @return new true state
	 */
	private State pickNewState() {
		double[] prob = new double[states.length];

		for (int s = 0; s < states.length; s++)
			prob[s] = T[getStateNbr(currTrueState)][s];

		return states[randomPick(prob)];
	}
	
	/**
	 * Return random i = 0..pos.length-1 with probability
	 * according to the element pos[i].
	 * @param pos Array with probabilities for earch index
	 * @return random index
	 */
	private int randomPick(double[] pos) {
		double rand = Math.random();
		double cumProb = 0;

		for (int i = 0; i < pos.length; i++) {
			cumProb += pos[i];
			
			if(rand <= cumProb)
				return i;
		}
		return pos.length;
	}

	
	private void computeProbLoc() {
		Arrays.fill(probLoc, 0);

		for (int i = 0; i < probLoc.length; i++)
			for (int h = 0; h < 4; h++) {
				probLoc[i] += fmsg.get(i * 4 + h);
				//probLoc[i] += f[i * 4 + h];
		        //System.out.println(" prob loc at: " + i +" " +  probLoc[i] );
			}
				
	}
	
	public double probForPos(State st) {
		double sum = 0.0;
		for (int dir = 0; dir < head; dir ++) {
			//int index = getStateNbr(st);
			//
			//sum += getElementAt(stateIndex(new State(pos.getX(),
            //                                           pos.getY(),
            //                                           heading)   ), 0);
			State pos = new State(st.getX(),st.getY(),dir);
			System.out.println("state s: " + pos.getX() + 
					                         pos.getY() + 
					                         dir);
			
            int index = stateIndex(pos);
            System.out.println("f index: " + index);
            //
            sum += f[index];
            //sum += fm[index][0];
			
		}
		return sum;
	}
	
	//private double[] pickOMatrix() {
	private SimpleMatrix pickOMatrix() {
		reading = getReading();
		//reading = currentReading(currTrueState);
		int index = reading == null ? OMs.length - 1 : 
			                         getStateNbr(reading) / 4;
		return OMs[index];
	}
	
	/**
	 * Returns the sensor reading.
	 * @return sensor reading(location not heading)
	 */
	private State getReading() {
		double probReading[] = new double[OMs.length];
		int ts_index = getStateNbr(currTrueState);
		//int ts_index = stateIndex(currTrueState);
		
		for(int sm = 0; sm < OMs.length; sm++) {
			
			//probReading[sm] = OgetElementAt(sm, ts_index);
			probReading[sm] = OMs[sm].get(ts_index, ts_index); //Diagonal matrix
		}
		int index = randomPick(probReading);
		return (index < OMs.length - 1) ? states[index * 4] : null;
	}

	
	/*
     * In case a new heading is to be found, the new one is randomly chosen 
     * from the possible ones. 
     */
	private int getNewHeading() {
		ArrayList<Integer> possHeads = new ArrayList<Integer>();

		for (int i = 0; i < 4; ++i) {
			if (i != currHead && notFacingWall(currLoc[0], 
					                           currLoc[1], 
                                               headings[i][0], 
                                               headings[i][1])) {
				possHeads.add(i);
			}
		}
		//System.out.println("CurrentHeading: " + currHead + 
                //                     " : " + headings[currHead][0] + 
                //                     " " + headings[currHead][1]);
		//System.out.println("SIZE OF POSSIBLEHEADINGS: " 
                //                     + possHeads.size());
		return possHeads.get(new Random().nextInt(possHeads.size()));
	}

    /**
      * Check if any direction does not point to a wall
      */
	private boolean notFacingWall(int x, int y, int dx, int dy) {
		return (x + dx) >= 0 && (x + dx) < rows 
                       && (y + dy) >= 0 && (y + dy) < cols;
	}
	
	// generate diagonal matrix from vector
	private double[][] toDiagonal(double[] vec) {
		//int length = vec.length * head; 
		int rows = vec.length;
		int cols = vec.length;
		//double[][] matrix = new double[length][length];
		double[][] matrix = new double[rows][cols];

		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				//matrix[i][i] = 1;
				matrix[i][j] = (i == j) ? vec[i] : 0.0;
				//matrix[i][j] = (i == j) ? 1 : 0.0;        // XXXXXXX
			}
		}
		
		return matrix;	
	}
	
	/**
     * normalize
     * function to normalize matrix
     * @param matrix
     */
	public void normalize(double[][] matrix) {
		double sum = 0.0;
		
		// sum up all the elements
		for (int row = 0; row < rows; row ++)
			for (int col = 0; col < cols; col ++) 
				sum += matrix[row][col];
		
		// normalize by the sum
		for (int row = 0; row < rows; row ++)
			for (int col = 0; col < cols; col ++)
				matrix[row][col] /= sum;
	}
	
	/**
     * normalize
     * function to normalize vector
     * @param vect
     */
    public void normalize(double[] vect){
        double[] result = new double[vect.length];

        double sum = 0;
        for(int i = 0; i < vect.length; i++){
            sum += vect[i];
        }

        for(int i = 0; i < vect.length; i++){
            result[i] = vect[i] / sum;
        }

    }


	// ==========================================================
	// Private methods - TM
	// ========================================================== 
	
	private int stateIndex(int x, int y, int h) {
		return h + x * head + y * head * rows;
		//return h + y * head + x * head * rows;
		
	}

	private int stateIndex(State pos) {
		return pos.getH() + pos.getX() * head + pos.getY() * head * rows;
		//return pos.getH() + pos.getY() * head + pos.getX() * head * rows;
	}
	
	private State statePosition(int index) {
		int y = index / (head * rows);
		//int x = index / (head * rows);
	    //index -= (y * head * rows);
	    index = index % (head * rows);
	    //index -= (x * head * rows);
	    int x = index / head;
	    //int y = index / head;
	    int h = index % head;
	    //System.out.println("Position at index: " + index + " is: " + x+y+h);
	    
	    return new State(x,y,h);
	    
		
	}
	
	public void setElementAt(int x , int y, double value) {
		T[x][y] = value;
	}

	// ==========================================================
		// Private methods - SM
		// ========================================================== 
		
		private int OstateIndex(int x, int y, int h) {
			return h + x * head + y * head * rows;
			//return h + y * head + x * head * rows;
			
		}

		private int OstateIndex(State pos) {
			return pos.getH() + pos.getX() * head + pos.getY() * head * rows;
			//return pos.getH() + pos.getY() * head + pos.getX() * head * rows;
		}
		
		private State OstatePosition(int index) {
			int y = index / (head * rows);
			//int x = index / (head * rows);
		    //index -= (y * head * rows);
		    index = index % (head * rows);
		    //index -= (x * head * rows);
		    int x = index / head;
		    //int y = index / head;
		    int h = index % head;
		    return new State(x,y,h);
			
		}
		
		public void OsetElementAt(int x , int y, double value) {
			O[x][y] = value;
		}
		
		public double OgetElementAt(int x , int y) {
			return O[x][y];
		}
		
		private int readingIndex(int rX, int rY) {
			if (rX == -1 && rY == -1) return rows * cols;
			return rX + rY * rows;
			//return rX * rows + rY;
		}
		
		private State readingPosition(int index) {
			if (index == rows * cols) return new State(-1,-1);
			
			int rY = index / rows;
			int rX = index % rows;
			return new State(rX,rY);			
			
		}
	// ==========================================================
	// Utility functions - Matrix Algebra
	// ========================================================== 
	
	// return c = a * b
    private double[][] multiplyM(double[][] a, double[][] b) {
        int m1 = a.length;
        int n1 = a[0].length;
        int m2 = b.length;
        int n2 = b[0].length;

        if (n1 != m2) throw new RuntimeException("Illegal matrix dimensions.");
        double[][] c = new double[m1][n2];

        for (int i = 0; i < m1; i++)
            for (int j = 0; j < n2; j++)
            	
                for (int k = 0; k < n1; k++)
                    c[i][j] += a[i][k] * b[k][j];
        return c;
    }
    
    // matrix-vector multiplication (y = A * x)
    private double[] multiplyM(double[][] a, double[] x) {
        int m = a.length;
        int n = a[0].length;
        if (x.length != n) 
        	throw new RuntimeException("Illegal matrix dimensions.");
        
        double[] y = new double[m];
        
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                y[i] += a[i][j] * x[j];
        return y;
    }
    
    // return B = A^T
    private double[][] transposeM(double[][] a) {
        int m = a.length;
        int n = a[0].length;
        double[][] b = new double[n][m];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++)
                b[j][i] = a[i][j];
        return b;
    }
}