#include<stdio.h>
#include<conio.h>
#include<math.h>
#include<graphics.h>
#include<stdlib.h>
#include<dos.h>

#define R 24 // radius of the wheel
#define PI 3.1415

void putPathParameter( int type );
void drawPath();
void drawCycle();
int getPathY( int x );
void initCycleParameter();
void initPathParameter( int type );
void transform( float* tMat[3], int* points[3], int* tempMat[3], int n );
void toMatrixForm();
void toPointForm();
void initialize();
void destroy();
void doTheEffect( int type );
void setTranslation( int tX, int tY );
void pedalToMatrix();
void matrixToPedal();
void setRotation( float angle, int rX, int rY );
void copyMat( int* source[3], int* dest[3], int n );
void changeCycleParameter();

int* pathX; // holds x-coordinate where the y changes for a path
int* pathY; // holds y-coordinate for correspoding x
int frontWheel[2]; // front wheel center
int rearWheel[2]; // rear wheel center coordinates
int midJoint[2]; // middle joint coordinates
int frontJoint[2]; // front joint coordinates
int rearJoint[2]; // rear joint coordinates
int handleJoint[2]; // handle joint coordinates
int shoulder[2]; // shoulder coordinates
int hand[2]; // hand coordinates
int neck[2]; // neck coordinates
int head[2]; // head center coordinates
int pedalOne[2]; // one end of the pedal
int pedalTwo[2]; // another end of the pedal
int numPathPairs; // holds the number of pairs of point to define path
int numPoints = 12; // number of points in the graphics
float prevSlope = 0; // slope of the path
int* points[3]; // holds points in matrix form
int* tempMat[3]; // holds points for temporary for processing
int* pedalMat[3]; // holds the matrix of pedal vertices
float* tTranslateMat[3]; // holds translate matrix
float* tRotateMat[3]; // holds the rotation matrix


void main() {
	int gd=DETECT; // graphics driver
	int gm; // graphics mode
	int randomNum;
	clrscr();
	initgraph( &gd, &gm, "C:\\TC\\BGI" );
	while( !kbhit() ) {
		randomNum = rand() % 5 + 1; // gives value from 1 to 5
		doTheEffect( randomNum );
	}
	closegraph();
}

// performs necessary allocations
void initialize() {
	int i;
	points[0]= (int*) calloc(numPoints, sizeof(int) );
	points[1]= (int*) calloc(numPoints, sizeof(int) );
	points[2]= (int*) calloc(numPoints, sizeof(int) );
	tempMat[0]= (int*) calloc(numPoints, sizeof(int) );
	tempMat[1]= (int*) calloc(numPoints, sizeof(int) );
	tempMat[2]= (int*) calloc(numPoints, sizeof(int) );
	tTranslateMat[0]= (float*) calloc(3, sizeof(float) );
	tTranslateMat[1]= (float*) calloc(3, sizeof(float) );
	tTranslateMat[2]= (float*) calloc(3, sizeof(float) );
	tRotateMat[0]= (float*) calloc(numPoints, sizeof(float) );
	tRotateMat[1]= (float*) calloc(numPoints, sizeof(float) );
	tRotateMat[2]= (float*) calloc(numPoints, sizeof(float) );
	pedalMat[0] = (int*) calloc( 2, sizeof(int) );
	pedalMat[1] = (int*) calloc( 2, sizeof(int) );
	for( i=0; i<numPoints; i++ )
		points[2][i] = 1;
	for( i=0; i<2; i++ )
		pedalMat[2][i] = 1;
}

// release the memory allocated
void destroy() {
	int i;
	for (i=0; i<3; i++ ) {
		free( points[i] );
		free( tempMat[i] );
		free( tTranslateMat[i] );
		free( tRotateMat[i] );
		free( pedalMat[i] );
	}
	prevSlope = 0;
}

// populate the pathX and pathY arrays with appropriate values
// set the number of values in pathX and pathY to numPathPairs
void initPathParameter( int type ) {
	int i=0;
	pathX = (int*) calloc( type+1, sizeof(int) );
	pathY = (int*) calloc( type+1, sizeof(int) );
	pathY[0] = getmaxy() - 100;
	numPathPairs = type+1;
	// populate pathX
	for( i=0; i<numPathPairs; i++ ) {
		pathX[i] = getmaxx() / type * i;
	}
	if( type==1 ) {
		pathY[1] = pathY[0];
	} else if( type == 2 ) {
		pathY[1] = pathY[0];
		pathY[2] = pathY[1] - 100;
	} else if( type == 3 ) {
		pathY[1] = pathY[0];
		pathY[2] = pathY[1] - 120;
		pathY[3] = pathY[2];
	} else if( type == 4 ) {
		pathY[1] = pathY[0];
		pathY[2] = pathY[1] - 80;
		pathY[3] = pathY[2];
		pathY[4] = pathY[3] + 100;
	} else if ( type == 5 ) {
		pathY[1] = pathY[0];
		pathY[2] = pathY[1] - 80;
		pathY[3] = pathY[2];
		pathY[4] = pathY[3] + 100;
		pathY[5] = pathY[4];
	} else
		numPathPairs = 0;
}

// returns value of y-coordinate for provided x in path
int getPathY( int x ) {
	int i = 1;
	float ratio;
	float y;
	if( x <= pathX[0] )
		return pathY[0];
	if( x >= pathX[numPathPairs-1] )
		return pathY[ numPathPairs-1];
	while( i < numPathPairs ) {
		if( x < pathX[i] ) {
			ratio = (float) (x-pathX[i-1]) / (pathX[i]-x);
			y = (ratio*pathY[i]+pathY[i-1]) / (ratio+1);
			return (int)y;
		}
		i++;
	}
	return -1;
}

// populate the coordinates for the joints in the cycle
void initCycleParameter() {
	rearWheel[0] = -3*R;
	rearWheel[1] = getPathY( rearWheel[0] ) - R;
	frontWheel[0] = rearWheel[0] + 3*R;
	frontWheel[1] = rearWheel[1];
	midJoint[0] = rearWheel[0] + 1.5 * R;
	midJoint[1] = rearWheel[1];
	rearJoint[0] = rearWheel[0] + 1.5 * R * cos( PI / 3 );
	rearJoint[1] = rearWheel[1] - 1.5 * R * sin( PI / 3 );
	frontJoint[0] = rearJoint[0] + 1.5 * R;
	frontJoint[1] = rearJoint[1];
	shoulder[0] = rearJoint[0] + R * cos( PI / 2.3 );
	shoulder[1] = rearJoint[1] - R;
	neck[0] = shoulder[0];
	neck[1] = shoulder[1] - 0.16 * R;
	head[0] = neck[0];
	head[1] = neck[1] - 0.3 * R;
	handleJoint[0] = frontJoint[0] - 0.5 * R * cos( PI / 3 );
	handleJoint[1] = frontJoint[1] - 0.5 * R * sin( PI / 3 );
	hand[0] = handleJoint[0] - 0.2 * R;
	hand[1] = handleJoint[1];
	pedalOne[0] = midJoint[0] + 0.5 * R * cos( PI/6 );
	pedalOne[1] = midJoint[1] - 0.5 * R * sin( PI/6 );
	pedalTwo[0] = midJoint[0] - 0.5 * R * cos( PI/6 );
	pedalTwo[1] = midJoint[1] + 0.5 * R * sin( PI/6 );
}

// draw the path according the value set
void drawPath() {
	int i=0;
	int defaultColor;
	defaultColor = getcolor();
	setcolor(5); // set the color for the path
	if( pathX == NULL || pathY == NULL )
		initPathParameter( 1 ); // set the default path to straight line
	for( i=0; i<numPathPairs-1;  i++ ) {
		line( pathX[i], pathY[i], pathX[i+1], pathY[i+1] );
	}
	setcolor( defaultColor ); // restore default color
}

// draw a cycle based upon available values
void drawCycle() {
	int defaultColor = getcolor();
	// rotate the pedal
	pedalToMatrix();
	setRotation( PI/9, midJoint[0], midJoint[1] );
	transform( tRotateMat, pedalMat, tempMat, 2 );
	copyMat( tempMat, pedalMat, 2 );
	matrixToPedal();
	// draw graphical cycle
	circle( rearWheel[0], rearWheel[1], R );
	circle( rearWheel[0], rearWheel[1], 0.98*R );
	circle( frontWheel[0], frontWheel[1], R );
	circle( frontWheel[0], frontWheel[1], 0.98*R );
	line( rearWheel[0], rearWheel[1], rearJoint[0], rearJoint[1] );
	line( rearWheel[0], rearWheel[1], midJoint[0], midJoint[1] );
	line( rearJoint[0], rearJoint[1],  midJoint[0], midJoint[1] );
	line( midJoint[0], midJoint[1], frontJoint[0], frontJoint[1] );
	line( rearJoint[0], rearJoint[1], frontJoint[0], frontJoint[1] );
	line( frontJoint[0], frontJoint[1], frontWheel[0], frontWheel[1] );
	line( frontJoint[0], frontJoint[1], handleJoint[0], handleJoint[1] );
	line( handleJoint[0], handleJoint[1], hand[0], hand[1] );
	line( pedalOne[0], pedalOne[1], pedalTwo[0], pedalTwo[1] );
	setcolor(10);
	line( rearJoint[0], rearJoint[1], shoulder[0], shoulder[1] );
	line( hand[0], hand[1], shoulder[0], shoulder[1] );
	line( shoulder[0], shoulder[1], neck[0], neck[1] );
	line( pedalOne[0], pedalOne[1], rearJoint[0], rearJoint[1] );
	line( pedalTwo[0], pedalTwo[1], rearJoint[0], rearJoint[1] );
	fillellipse( head[0], head[1], 0.16 * R, 0.3  * R );
	setcolor( defaultColor );
}

// put the points into a matrix for transformation
void toMatrixForm() {
	int i;
	for( i=0; i<2; i++ ) {
		points[i][0] = rearWheel[i];
		points[i][1] = frontWheel[i];
		points[i][2] = midJoint[i];
		points[i][3] = rearJoint[i];
		points[i][4] = frontJoint[i];
		points[i][5] = handleJoint[i];
		points[i][6] = hand[i];
		points[i][7] = shoulder[i];
		points[i][8] = neck[i];
		points[i][9] = head[i];
		points[i][10] = pedalOne[i];
		points[i][11] = pedalTwo[i];
	}
}

// put the vertices in matrix points into vertices
void toPointForm() {
	int i;
	for( i=0; i<2; i++ ) {
		rearWheel[i] = points[i][0];
		frontWheel[i] =	points[i][1];
		midJoint[i] = points[i][2];
		rearJoint[i] = points[i][3];
		frontJoint[i] =	points[i][4];
		handleJoint[i] = points[i][5];
		hand[i] = points[i][6];
		shoulder[i] = points[i][7];
		neck[i] = points[i][8];
		head[i] = points[i][9];
		pedalOne[i] = points[i][10];
		pedalTwo[i] = points[i][11];
	}
}

// put pedal vertices in matrix form
void pedalToMatrix() {
	int i;
	for( i=0; i<2; i++ ) {
		pedalMat[i][0] = pedalOne[i];
		pedalMat[i][1] = pedalTwo[i];
	}
}

// put vertices in matrix pedalMatrix in pedal
void matrixToPedal() {
	int i;
	for( i=0; i<2; i++) {
		pedalOne[i] = pedalMat[i][0];
		pedalTwo[i] = pedalMat[i][1];
	}
}

// copy elements of a matrix to another
void copyMat( int* source[3], int* dest[3], int n ) {
	int i, j;
	for( i=0; i<3; i++ )
		for( j=0; j<n; j++ )
			dest[i][j] = source[i][j];
}

// modifies the tTranslateMat matrix to do translation
void setTranslation( int tX, int tY ) {
	tTranslateMat[0][0] = 1;
	tTranslateMat[0][1] = 0;
	tTranslateMat[0][2] = tX;
	tTranslateMat[1][0] = 0;
	tTranslateMat[1][1] = 1;
	tTranslateMat[1][2] = tY;
	tTranslateMat[2][0] = 0;
	tTranslateMat[2][1] = 0;
	tTranslateMat[2][2] = 1;
}

// modifies the tRotateMat matrix to do rotation
void setRotation( float angle, int rX, int rY ) {
	tRotateMat[0][0] = cos(angle);
	tRotateMat[0][1] = - sin(angle);
	tRotateMat[0][2] = rX*(1-cos(angle)) + rY*sin(angle);
	tRotateMat[1][0] = sin(angle);
	tRotateMat[1][1] = cos(angle);
	tRotateMat[1][2] = rY*(1-cos(angle)) - rX*sin(angle);
	tRotateMat[2][0] = 0;
	tRotateMat[2][1] = 0;
	tRotateMat[2][2] = 1;
}

// performs tranformation operation by matrix multiplication
void transform( float* t[3], int* input[3], int* output[3], int n ) {
	int i, j, k;
	float temp=0;
	// for each column in 2nd matrix
	for( k=0; k<n; k++ ) {
		// for each row in 1st matrix
		for( i=0; i<3; i++ ) {
			temp = 0;
			// do matrix multiplication
			for( j=0; j<3; j++ ) {
				temp += t[i][j] * input[j][k];
			}
			output[i][k] = (int)temp;
		}
	}
}

// checks for the sloppy rode and rotate if necessary
void changeCycleParameter() {
	int rearY;
	int rearX;
	int frontX;
	int frontY;
	int deltaX;
	int deltaY;
	float slope = 0;
	float slopeAngle = 0;
	rearX = rearWheel[0];
	rearY = getPathY( rearX );
	frontX = frontWheel[0];
	frontY = getPathY( frontX );
	deltaX = frontX - rearX;
	deltaY = frontY - rearY;
	while(  deltaX*deltaX + deltaY*deltaY > 9*R*R ) {
		frontX--;
		frontY = getPathY( frontX );
		deltaX = frontX - rearX;
		deltaY = frontY - rearY;
	}
	toMatrixForm();
	setTranslation( 0, rearY-R-rearWheel[1] );
	transform( tTranslateMat, points, tempMat, numPoints );
	copyMat( tempMat, points, numPoints );
	// determine slope and angle to rotate
	slope = (float) (frontY-rearY)/(frontX-rearX);
	slopeAngle = atan( slope-prevSlope );
	prevSlope = slope;
	// rotate the graphics in slope angle about rear wheel point
	setRotation( slopeAngle, rearWheel[0], rearWheel[1] );
	transform( tRotateMat, points, tempMat, numPoints );
	copyMat( tempMat, points, numPoints );
	toPointForm();
}

// give the moving effect to the object by clearing delaying
void doTheEffect(int type) {
	initialize();
	initPathParameter( type );
	initCycleParameter();
	do {
		clearviewport();
		drawPath();
		changeCycleParameter();
		drawCycle();
		delay(100);
		toMatrixForm();
		setTranslation( .5*R, 0 ); // translate to the right
		transform( tTranslateMat, points, tempMat, numPoints );
		copyMat( tempMat, points, numPoints );
		toPointForm();
	} while( frontWheel[0] <= pathX[numPathPairs-1] );
	destroy();
}
