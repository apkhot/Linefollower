//You are allowed to define your own function to fulfill the requirement of tasks
//Dont change the name of following functions

#include "CB_Task_1_Sandbox.h"

double K[3] = { 15, 0.068, 0.08 };
//double K[3] = { 0, 0, 0 };
//20,0.05,6
//10,0.03,0.1
//10,0.03,0.08
//10,0.07,0.06



unsigned char isConverged = 0;

//Read Sensor Position

int readPosition() {
	int position;

	int x1=0, x2=0, x3=0, x4=0, x5=0, x6=0;
	int a1, a2, a3, a4, a5, a6;
	int b1, b2, b3, b4, b5, b6;

	unsigned char left_sensor, center_sensor, right_sensor;

	left_sensor = ADC_Conversion(1);
	center_sensor = ADC_Conversion(2);
	right_sensor = ADC_Conversion(3);

	/*printf("\n %d %d %d", left_sensor, center_sensor, right_sensor);*/

	if (left_sensor == 255 && center_sensor == 0 && right_sensor == 0) {
		x1 = 1, x2 = 0, x3 = 0, x4 = 0, x5 = 0, x6 = 0;
	}
	if (left_sensor == 255 && center_sensor == 255 && right_sensor == 0) {
		x1 = 0, x2 = 1, x3 = 0, x4 = 0, x5 = 0, x6 = 0;
	}
	if (left_sensor == 0 && center_sensor == 255 && right_sensor == 0) {
		x1 = 0, x2 = 0, x3 = 1, x4 = 0, x5 = 0, x6 = 0;
	}
	if (left_sensor == 255 && center_sensor == 255 && right_sensor == 255) {
		x1 = 0, x2 = 0, x3 = 0, x4 = 1, x5 = 0, x6 = 0;
	}
	if (left_sensor == 0 && center_sensor == 255 && right_sensor == 255) {
		x1 = 0, x2 = 0, x3 = 0, x4 = 0, x5 = 1, x6 = 0;
	}
	if (left_sensor == 0 && center_sensor == 0 && right_sensor == 255) {
		x1 = 0, x2 = 0, x3 = 0, x4 = 0, x5 = 0, x6 = 1;
	}
	// Monotonic error function
	a1 = -2, a2 = -1, a3 = 0, a4 = 0, a5 = 1, a6 = 2;
	b1 = b2 = b3 = b4 = b5 = b6 = 0.5;

	position = a1 * (x1 - b1) + a2 * (x2 - b2) + a3 * (x3 - b3) + a4 * (x4 - b4) + a5 * (x5 - b5) + a6 * (x6 - b6);

	return(position);
}

// PID Control
unsigned int PIDcontrol(unsigned char node, double K[3], unsigned char isConverged) {
	int position;

	unsigned char nodeIdx = 0;

	unsigned char left_sensor, center_sensor, right_sensor;

	double Kp = K[0];
	double Ki = K[1];
	double Kd = K[2];

	int  P, I = 0, D;
	int previousError = 0;
	double PIDvalue;


	unsigned char nodeFlag = 1;

	position = readPosition();
	//position *=127;	// to scale up with 8-bit ADC readings (0-255)

	P = position;
	I = position + previousError;
	D = position - previousError;

	PIDvalue = (Kp * (double)P + Ki * (double)I + Kd * (double)D);

	previousError = position;

	//PIDvalue *= 50;
	//PIDvalue *= 127;

	// Print PID value
	printf("%lf %f %f %f %d %d %d \n", PIDvalue,Kp,Ki,Kd,P,I,D);
	

	if (PIDvalue > 127.0)
		PIDvalue = 127;
	else if (PIDvalue < -127.0)
		PIDvalue = -127;

	// Proceed forward till you're not on node
	forward();

	// PIDvalue +ve: move right;
	// PIDvalue -ve: move left;
	velocity(150 + (int)PIDvalue, 150 - (int)PIDvalue);

	if (isConverged == 1) {

		while (1) {

			position = readPosition();
			//position *= 127;	// to scale up with 8-bit ADC readings (0-255)

			P = position;
			I = position + previousError;
			D = position - previousError;

			PIDvalue = (Kp * (double)P + Ki * (double)I + Kd * (double)D);

			previousError = position;

			//PIDvalue *= 50;
			//PIDvalue *= 127;
			// Print PID value
			printf("%lf %f %f %f %d %d %d \n", PIDvalue, Kp , Ki , Kd , P, I, D);


			if (PIDvalue > 127.0)
				PIDvalue = 127;
			else if (PIDvalue < -127.0)
				PIDvalue = -127;

			// Proceed forward till you're not on node
			forward();

			// PIDvalue +ve: move right;
			// PIDvalue -ve: move left;
			velocity(150 + (int)PIDvalue, 150 - (int)PIDvalue);

			// Fetch sensor position
			left_sensor = ADC_Conversion(1);
			center_sensor = ADC_Conversion(2);
			right_sensor = ADC_Conversion(3);

			/*printf("\n %d %d %d", left_sensor, center_sensor, right_sensor);*/

			// if bot on node, then nodeIdx++
			if (left_sensor == 255 && center_sensor == 255 && right_sensor == 255 && nodeFlag == 1) {
				nodeIdx++;
				nodeFlag = 0;
			}

			if (left_sensor == 0 && center_sensor == 255 && right_sensor == 0) {
				nodeFlag = 1;
			}

			// to move a bit from node 
			if (nodeIdx >= node) {
				//printf("\n%d nodes crossed", node);
				while (1) {
					forward();
					velocity(150 + (int)PIDvalue, 150 - (int)PIDvalue);//100,100
					left_sensor = ADC_Conversion(1);
					center_sensor = ADC_Conversion(2);
					right_sensor = ADC_Conversion(3);
					printf("\n %d %d %d %d", left_sensor, center_sensor, right_sensor, nodeIdx);
					if (left_sensor == 255 && center_sensor == 255 && right_sensor == 255) {
						_delay_ms(450);//650
						//stop();
						break;
					}
				}

				break; // to come out of while(1) when # of nodes have crossed

			} // end node detection

		} // end while(1)

	} // end isConverged code ends

	position = readPosition();
	//position *= 127;	// to scale up with 8-bit ADC readings (0-255)

	return abs(position);
} // end PIDcontrol


/*
*
* Function Name: forward_wls
* Input: node
* Output: void
* Logic: Uses white line sensors to go forward by the number of nodes specified
* Example Call: forward_wls(2); //Goes forward by two nodes
*
*/
void forward_wls(unsigned char node)
{
	

	isConverged = 1;
	PIDcontrol(node, K, isConverged);

} //end forward
/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls(); //Turns right until black line is encountered
*
*/
void left_turn_wls(void)
{
	unsigned char left_sensor, center_sensor, right_sensor;
	while (1) {
		left_sensor = ADC_Conversion(1);
		center_sensor = ADC_Conversion(2);
		right_sensor = ADC_Conversion(3);

		if (left_sensor == 0 && center_sensor == 255 && right_sensor == 0) {
			stop();
			break;
		}
		left();

	}
}

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls(); //Turns right until black line is encountered
*/
void right_turn_wls(void)
{
	unsigned char left_sensor, center_sensor, right_sensor;

	while (1) {
		left_sensor = ADC_Conversion(1);
		center_sensor = ADC_Conversion(2);
		right_sensor = ADC_Conversion(3);
		printf("\n %d %d %d", left_sensor, center_sensor, right_sensor);
		if (left_sensor == 0 && center_sensor == 255 && right_sensor == 0) {
			stop();
			break;
		}
		right();
	}

	/*printf("\n %d %d %d", left_sensor, center_sensor, right_sensor);*/
	//_delay_ms(100);
}

void twiddle(unsigned char node) 
{
	double dp[3] = { 1, .01, 1 };

	unsigned int best_err;
	unsigned int err;

	double sum = (dp[0] + dp[1] + dp[2]);


	//forward();
	//velocity(200, 100);
	//_delay_ms(250);
	//velocity(100, 200);
	//_delay_ms(250);

	best_err = PIDcontrol(node, K, isConverged);

	while (sum > 1e-18) {

		for (int j = 0; j < 3; j++) {

			K[j] += dp[j];

			for(int k=0;k<5;k++) {
				err = PIDcontrol(node, K, isConverged);
			}

			if (err < best_err) {
				best_err = err;
				dp[j] *= 1.1;
			}
			else {
				K[j] -= 2 * dp[j];
				if (K[j] < 0)
					K[j] = 0;

				for(int k=0;k<5;k++) {
					err = PIDcontrol(node, K, isConverged);
				}

				if (err < best_err) {
					best_err = err;
					dp[j] *= 1.1;
				}
				else {
					K[j] += dp[j];
					dp[j] *= 0.9;
				}
			}
		}

		sum = (dp[0] + dp[1] + dp[2]);

	} // end while sum
}
/*
*
* Function Name: e_shape
* Input: void
* Output: void
* Logic: Use this function to make the robot trace a e shape path on the arena
* Example Call: e_shape();
*/
void  e_shape(void)
{
	//twiddle(1);

	forward_wls(1);
	right_turn_wls();

	forward_wls(2);
	right_turn_wls();

	forward_wls(1);
	right_turn_wls();

	forward_wls(1);
	right_turn_wls();

	forward_wls(1);

	stop();


}


/*
*
* Function Name: Task_1_1
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.1 logic
* Example Call: Task_1_1();
*/
void Task_1_1(void)
{
	forward_wls(1);
	left_turn_wls();
	forward_wls(1);
	right_turn_wls();
	forward_wls(1);




	// Write your task 1.1 Logic here
}

/*
*
* Function Name: Task_1_2
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.2 logic
* Example Call: Task_1_2();
*/
void Task_1_2(void)
{
	//write your task 1.2 logic here
}