package models;

public class TransitionMatrix {
	private int rows, cols;
	private int head;
	private int dim;
	private StateModel stateModel;
	
	private double[][] matrix;
	
	
	public TransitionMatrix( StateModel stateModel) {
		this.stateModel = stateModel;
		int[] help = stateModel.getDimensions();
		
		rows = help[0];
		cols = help[1];
		
		head = help[2];
		dim = rows * cols * head;
		
		matrix = new double[dim][dim];
		for( int i=0; i<dim; i++)
			for( int j=0; j<dim; j++)
				matrix[i][j] = 0.0;
		
	
		initMatrix();
		
	}

	private void initMatrix() {
	
		int x, y, h, nx, ny, nh; 
		int i, j;
		int[] help;
		
		x = 0; y = 0; h = 0; nx = 0; ny = 0; nh = 0;
		
		for( i = 0; i < dim; i++) {
			help = stateModel.robotStateToXYH( i);
			h = help[2];
			x = help[0];
			y = help[1];
		
			for( j = 0; j < dim; j++) {
				help = stateModel.robotStateToXYH( j);
				nh = help[2];
				nx = help[0];
				ny = help[1];
				
				
				if( Math.abs( x-nx) + Math.abs( y-ny) == 1 && 
					( nh == 0 && nx == x-1 || nh == 1 && ny == y+1 ||
					  nh == 2 && nx == x+1 || nh == 3 && ny == y-1)) {
					if( nh == h) {
						matrix[i][j] = 0.7;
					} else {
						if( x != 0 && x != rows-1 && y != 0 && y != cols-1) { matrix[i][j] = 0.1;}
						else if( h == 0 && x == 0 && y != 0 && y != cols-1 ||
							     h == 1 && x != 0 && x != rows-1 && y == cols-1 ||
							     h == 2 && x == rows-1 && y != 0 && y != cols-1 ||
							     h == 3 && x != 0 && x != rows-1 && y == 0) { matrix[i][j] = 1.0 / 3.0;}
						else if( h != 0 && x == 0 && y != 0 && y != cols-1 ||
							     h != 1 && x != 0 && x != rows-1 && y == cols-1 ||
							     h != 2 && x == rows-1 && y != 0 && y != cols-1 ||
							     h != 3 && x != 0 && x != rows-1 && y == 0) { matrix[i][j] = 0.15;}
						else if( ( h == 0 || h == 3) && ( nh == 1 || nh == 2) && x == 0 && y == 0 || 
								 ( h == 0 || h == 1) && ( nh == 2 || nh == 3) && x == 0 && y == cols-1 ||
								 ( h == 1 || h == 2) && ( nh == 0 || nh == 3) && x == rows-1 && y == cols-1 ||
								 ( h == 2 || h == 3) && ( nh == 0 || nh == 1) && x == rows-1 && y == 0) { matrix[i][j] = 0.5;}
						else if( ( h == 1 && nh == 2 || h == 2 && nh == 1) && x == 0 && y == 0 ||
								 ( h == 2 && nh == 3 || h == 3 && nh == 2) && x == 0 && y == cols-1 ||
								 ( h == 0 && nh == 1 || h == 1 && nh == 0) && x == rows-1 && y == 0 ||
								 ( h == 0 && nh == 3 || h == 3 && nh == 0) && x == rows-1 && y == cols-1) { matrix[i][j] = 0.3;}
						
					}
				}
//				System.out.println( "testing state " 
//						+ i + " = (" +
//						x + ", " + y + ", " + h + ") against "
//						+ j + " = (" +
//						nx + ", " + ny + ", " + nh + "): " + matrix[i][j]);
				
			}
		}
	
	}
	
	public int getNrOfStates() {
		return dim;
	}
	
	public double getTij( int i, int j) {
		return matrix[i][j];
	}
	
	public double getTxyhToXYH( int x, int y, int h, int X, int Y, int H) {
		return matrix[stateModel.xyhToRobotState( x, y, h)][stateModel.xyhToRobotState( X, Y, H)];
	}
	
	
	public double[][] getT() {
		double[][] copy = new double[dim][dim];
		
		for( int i=0; i<dim; i++)
			for( int j=0; j<dim; j++)
				copy[i][j] = matrix[i][j];
		return copy;
	}

	public double[][] getT_transp() {
		double[][] copy = new double[dim][dim];
		
		for( int i=0; i<dim; i++)
			for( int j=0; j<dim; j++)
				copy[i][j] = matrix[j][i];
		return copy;
	}

	public void printMatrix() {
		for ( int i=0; i<dim; i++) {
			for( int j = 0; j<dim; j++)
				System.out.print( matrix[i][j] + "  ");
			System.out.println();
		}

	}
	

}