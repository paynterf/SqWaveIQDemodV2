//#include <avr/sleep.h> //needed for debugging
#pragma warning(disable : 4996)
//Program to implement John Jenkins I/Q sq wave demod scheme
//see his sync_filter_2c.xlsx document

//Purpose (v3):
//	Produce a running estimate of the magnitude of a square-wave modulated IR
//	signal in the presence of ambient interferers.  The estimate is computed by 
//	adding the absolute values of the running sums of the I & Q outputs from a 
//	digital bandpass filter centered at the square-wave frequency (nominally 520Hz).
//	The filter is composed of two N-element circular buffers, one for the 
//	I 'channel' and one for the Q 'channel'. Each element in each circular buffer 
//	represents one full-cycle sum of the I or Q 'channels' of the sampler.  Each 
//	full-cycle sum is composed of four 1/4 cycle groups of samples (nominally 5 
//	samples/group), where each group is first summed and then multiplied by the 
//	appropriate sign to form the I and Q 'channels'.  The output is updated once 
//	per input cycle, i.e. approximately once every 2000 uSec.  The size of the 
//	nominally 64 cycle circular determines the bandwidth of the filter.

//Plan:
//	Step1: Collect a 1/4 cycle group of samples and sum them into a single value
//	Step2: Assign the appropriate multiplier to form I & Q channel values, and
//		   add the result to the current cycle sum I & Q variables respectively
//	Step3: After 4 such groups have been collected and processed into full-cycle
//		   I & Q sums, add them to their respective N-cycle running totals
//	Step4: Subtract the oldest cycle sums from their respective N-cycle totals, and
//		   then overwrite these values with the new ones
//	Step5: Compute the final demodulate sensor value by summing the absolute values
//		   of the I & Q running sums.
//	Step6: Update buffer indicies so that they point to the new 'oldest value' for 
//		   the next cycle
//Notes:
//	step 1 is done each acquisition period
//	step 2 is done each SAMPLES_PER_GROUP acquisition periods
//	step 3-6 are done each SAMPLES_PER_CYCLE acquisition periods

#include <ADC.h> //Analog-to-Digital library
#include <SD.h> //SD card library

ADC *adc = new ADC(); // adc object;


#pragma region ProgConsts
const int OUTPUT_PIN = 32; //lower left pin
const int SQWV_OUTPUT_PIN = 31; //next to bottom pin, left side
const int IRDET1_PIN = A0; //aka pin 14

//06/26/17 added for freq matching with xmit board
//const double SQWAVE_FREQ_HZ = 518.5; //stopped!!!
const double SQWAVE_FREQ_HZ = 51.85; //stopped!!! 07/04/17 rev for debug
//const double SQWV_HALF_PERIOD_US = 500000 / SQWAVE_FREQ_HZ; //half-period in Usec
const double SQWV_PERIOD_US = 1000000 / SQWAVE_FREQ_HZ; //period in Usec

//const int USEC_PER_SAMPLE = 1E6 / (SQWAVE_FREQ_HZ * SAMPLES_PER_CYCLE);
const int SAMPLES_PER_CYCLE = 20;
const int GROUPS_PER_CYCLE = 4; 
const int SAMPLES_PER_GROUP = SAMPLES_PER_CYCLE / GROUPS_PER_CYCLE;
//const float USEC_PER_SAMPLE = 95.7; //value that most nearly zeroes beat-note
const float USEC_PER_SAMPLE = SQWV_PERIOD_US / SAMPLES_PER_CYCLE; //sample period, Usec
const int RUNNING_SUM_LENGTH = 64;
const int SAMPLE_CAPTURE_LENGTH = RUNNING_SUM_LENGTH*SAMPLES_PER_CYCLE;
const int SENSOR_PIN = A0;
const int aMultVal_I[GROUPS_PER_CYCLE] = { 1, 1, -1, -1 };
const int aMultVal_Q[GROUPS_PER_CYCLE] = { -1, 1, 1, -1 };
const int FULLSCALE_FINALVAL = 273584; //07/02/17 full-scale final value --> 3.3v DAC output
#pragma endregion Program Constants

#pragma region ProgVars
int SampleSum = 0;//sample sum for each channel
int CycleGroupSum_Q = 0;//cycle sum for each channel, Q component
int CycleGroupSum_I = 0;//cycle sum for each channel, I component
int aCycleSum_Q[RUNNING_SUM_LENGTH];//running cycle sums for each channel, Q component
int aCycleSum_I[RUNNING_SUM_LENGTH];//running cycle sums for each channel, I component
int RunningSumInsertionIndex = 0;
int RunningSum_Q = 0;//overall running sum for each channel, Q component
int RunningSum_I = 0;//overall running sum for each channel, I component
int FinalVal = 0;//final value = abs(I)+abs(Q) for each channel


//debugging variables
//int aRawData[RUNNING_SUM_LENGTH*SAMPLES_PER_CYCLE];//1280 elements per sensor
int aRawData[SAMPLE_CAPTURE_LENGTH];//12800 elements
int aSampleGroupSums_I[RUNNING_SUM_LENGTH*SAMPLES_PER_CYCLE];//1280 total, 256 non-zero elements 
int aSampleGroupSums_Q[RUNNING_SUM_LENGTH*SAMPLES_PER_CYCLE];//1280 total, 256 non-zero elements
int sample_count = 0; //this counts from 0 to 1279
int numpassess = 0;
elapsedMicros sinceLastSqWvTrans = 0; //06/26/17 added for freq matching with xmit board
int num_sample_pulses = 0; //used for square-wave generation


#pragma endregion Program Variables

elapsedMicros sinceLastSample;
int SampleSumCount; //sample sums taken so far. range is 0-4
int CycleGroupSumCount; //cycle group sums taken so far. range is 0-3

//07/02/17 Sinewave output variables
float phase = 0.0;
float twopi = 3.14159 * 2;

char incomingByte = 0; //used for incoming command chars via serial port
File myFile; //used for SD card I/O
const int chipSelect = BUILTIN_SDCARD;



void setup()
{
	Serial.begin(115200);
	pinMode(OUTPUT_PIN, OUTPUT);
	pinMode(SQWV_OUTPUT_PIN, OUTPUT); //added 06/26/17 for frequency matching with xmit board
	pinMode(IRDET1_PIN, INPUT);
	digitalWrite(OUTPUT_PIN, LOW);

//DEBUG!!
	Serial.print("USEC_PER_SAMPLE = "); Serial.println(USEC_PER_SAMPLE);
	Serial.print("SQWV_PERIOD_US = "); Serial.println(SQWV_PERIOD_US);

	//decreases conversion time from ~15 to ~5uSec
	adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
	adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
	adc->setResolution(12);
	adc->setAveraging(1);


	//init aCycleSum I & Q arrays
	for (size_t m = 0; m < RUNNING_SUM_LENGTH; m++)
	{
		aCycleSum_I[m] = 0;
		aCycleSum_Q[m] = 0;
	}

	//init debugging arrays
	for (int k = 0; k < RUNNING_SUM_LENGTH*SAMPLES_PER_CYCLE; k++)
	{
		aSampleGroupSums_I[k] = 0;
		aSampleGroupSums_Q[k] = 0;
	}

	//init sample capture array
	for (int i = 0; i < SAMPLE_CAPTURE_LENGTH; i++)
	{
		aRawData[i] = 0;
	}

	//Serial.println("GSumI\tGSumQ\tCSumI\tCSumQ\tRSI\tOldRSI\tOldRSQ\tCurRSI\tCurRQ\tNewRSI\tNewRSQ\tFinVal");
	//Serial.println("RSI\tCSumI\tCSumQ\tOldRSI\tOldRSQ\tCurRSI\tCurRQ\tNewRSI\tNewRSQ\tFinVal");
	//Serial.println("SI\tSamp\tGSumI\tGSumQ\tCGSumI\tCGSumQ\tOldI\tOldQ\tRSumI\tRSumQ\tFinVal");

	//07/01/17 bugfix - need to init elapsedMicros variables
	sinceLastSqWvTrans = 0; //06/26/17 added for freq matching with xmit board
	sinceLastSample = 0;

	//07/02/17 for sinewave output
	analogWriteResolution(12);

////DEBUG!! ----------------  Micro-SD Logging ---------------------------
//
//	Serial.print("Initializing SD card...");
//
//	if (!SD.begin(chipSelect)) {
//		Serial.println("initialization failed!");
//		return;
//	}
//	Serial.println("initialization done.");
//
//	// open the file. note that only one file can be open at a time,
//	// so you have to close this one before opening another.
//	myFile = SD.open("test.txt", FILE_WRITE);
//
//	// if the file opened okay, write to it:
//	if (myFile) 
//	{
//		Serial.print("Writing to test.txt...");
//		long startUsec = micros();
//		for (size_t i = 0; i < 1000; i++)
//		{
//			myFile.println("testing 1, 2, 3.");
//		}
//		// close the file:
//		myFile.close();
//		long endUsec = micros();
//		Serial.print("done. Time req for 1000 writes was "); 
//		Serial.print(endUsec - startUsec);
//		Serial.println(" Usec");
//	}
//	else {
//		// if the file didn't open, print an error:
//		Serial.println("error opening test.txt");
//	}
//
//	// re-open the file for reading:
//	myFile = SD.open("test.txt");
//	if (myFile) {
//		Serial.println("test.txt:");
//
//		// read from the file until there's nothing else in it:
//		while (myFile.available()) {
//			Serial.write(myFile.read());
//		}
//		// close the file:
//		myFile.close();
//	}
//	else {
//		// if the file didn't open, print an error:
//		Serial.println("error opening test.txt");
//	}
////DEBUG!! ----------------  Micro-SD Logging ---------------------------

}

void loop()
{
	if (Serial.available() > 0) 
	{
		incomingByte = Serial.read();
		Serial.print("I received: "); Serial.print(incomingByte, DEC);Serial.print(", ");Serial.println(incomingByte);

		if (incomingByte == 'q')
		{
			Serial.println("Exiting - Bye!");

			//close SD card logging file
			//enter infinite loop

		}
	}


	//this runs every 95.7uSec
	if (sinceLastSample > USEC_PER_SAMPLE)
	{
		/*RECORD sinceLastSample HERE!*/

		//sinceLastSample = 0;
		sinceLastSample -= USEC_PER_SAMPLE;

		//start of timing pulse
		digitalWrite(OUTPUT_PIN, HIGH);

		//	Step1: Collect a 1/4 cycle group of samples and sum them into a single value
		// this section executes each USEC_PER_SAMPLE period
		int samp = adc->analogRead(SENSOR_PIN);
		SampleSum += samp;
		SampleSumCount++; //goes from 0 to SAMPLES_PER_GROUP-1
		sample_count++; //checked in step 6 to see if data capture is complete

////DEBUG!!
//		Serial.print(sample_count); Serial.print("\t"); Serial.print(samp);
//		if (SampleSumCount != SAMPLES_PER_GROUP && CycleGroupSumCount != GROUPS_PER_CYCLE)
//		{
//			Serial.println();
//		}
//		else
//		{
//			Serial.print("\t");
//		}
////DEBUG!!

		//	Step2: Every 5th acquisition cycle, assign the appropriate multiplier to form I & Q channel values, and
		//		   add the result to the current cycle sum I & Q variables respectively
		if(SampleSumCount == SAMPLES_PER_GROUP) //an entire sample group sum from each channel is ready for further processing
		{
			SampleSumCount = 0; //starts a new sample group

			//compute new group sum.  Sign depends on which quarter cycle the sample group is from 
			//CycleGroupSumCount ranges from 0 to 3
			int groupsum_I = SampleSum * aMultVal_I[CycleGroupSumCount];
			int groupsum_Q = SampleSum * aMultVal_Q[CycleGroupSumCount];
			SampleSum = 0;

			//Serial.print(groupsum_I); Serial.print("\t"); Serial.println(groupsum_Q);

			CycleGroupSum_I += groupsum_I; //add new I comp to cycle sum
			CycleGroupSum_Q += groupsum_Q; //add new Q comp to cycle sum

////DEBUG!!
//			Serial.print(groupsum_I); Serial.print("\t"); Serial.print(groupsum_Q); Serial.print("\t");
//			Serial.print(CycleGroupSum_I); Serial.print("\t"); Serial.print(CycleGroupSum_Q);
//
//			if (CycleGroupSumCount != GROUPS_PER_CYCLE)
//			{
//				Serial.println();
//			}
//			else
//			{
//				Serial.print("\t");
//			}
////DEBUG!!

			CycleGroupSumCount++;
		}//if(SampleSumCount == SAMPLES_PER_GROUP)

		//	Step3: After 4 such groups have been collected and processed into full-cycle
		//		   I & Q sums, add them to their respective N-cycle running totals
		if(CycleGroupSumCount == GROUPS_PER_CYCLE) //now have a complete cycle sum for I & Q - load into running sum circ buff
		{
			CycleGroupSumCount = 0; //start a new cycle group next time

			//	Step4: Subtract the oldest cycle sum from the N-cycle total, and
			//		   then overwrite this values with the new one
			int oldestvalue_I = aCycleSum_I[RunningSumInsertionIndex];
			int oldestvalue_Q = aCycleSum_Q[RunningSumInsertionIndex];
////DEBUG!!
//			Serial.print(RunningSumInsertionIndex); Serial.print("\t");
//			Serial.print(CycleGroupSum_I); Serial.print("\t");
//			Serial.print(CycleGroupSum_Q); Serial.print("\t");
//			Serial.print(oldestvalue_I); Serial.print("\t");
//			Serial.print(oldestvalue_Q); Serial.print("\t");
//			Serial.print(RunningSum_I); Serial.print("\t");
//			Serial.print(RunningSum_Q); Serial.print("\t");
////DEBUG!!

			RunningSum_I = RunningSum_I + CycleGroupSum_I - oldestvalue_I;
			RunningSum_Q = RunningSum_Q + CycleGroupSum_Q - oldestvalue_Q;

			//	Step5: Compute the final demodulate sensor value by summing the absolute values
			//		   of the I & Q running sums
			int RS_I = RunningSum_I; int RS_Q = RunningSum_Q;
			FinalVal = abs((int)RS_I) + abs((int)RS_Q);

////DEBUG!!
			//Serial.print(RunningSum_I); Serial.print("\t");
			//Serial.print(RunningSum_Q); Serial.print("\t");

			//Serial.print(FinalVal);
			//Serial.println();
////DEBUG!!

			aCycleSum_I[RunningSumInsertionIndex] = CycleGroupSum_I;
			aCycleSum_Q[RunningSumInsertionIndex] = CycleGroupSum_Q;

			//reset cycle group sum values for next group
			CycleGroupSum_I = 0;
			CycleGroupSum_Q = 0;

			////	Step6: Update buffer indicies so that they point to the new 'oldest value' for 
			////		   the next cycle
			RunningSumInsertionIndex++;
		}

		//06/20/17 revised for debugging
		if (RunningSumInsertionIndex >= RUNNING_SUM_LENGTH)
		{
			RunningSumInsertionIndex = 0;
			sample_count = 0; 
			numpassess++;
			//Serial.print(FinalVal);
			//Serial.println();
		}//if (RunningSumInsertionIndex >= RUNNING_SUM_LENGTH)

		num_sample_pulses++; //used for square-wave generation

		if (num_sample_pulses == SAMPLES_PER_CYCLE/2)
		{
			digitalWrite(SQWV_OUTPUT_PIN, !digitalRead(SQWV_OUTPUT_PIN));
			num_sample_pulses = 0;
		}

		//end of timing pulse
		digitalWrite(OUTPUT_PIN, LOW);

	}//if (sinceLastSample > 95.7)

	 //added 06/26/17 for frequency matching with xmit board
	//07/02/17 rev to output sinewave on A14
	//if (sinceLastSqWvTrans > SQWV_HALF_PERIOD_US)
	//{
	//	sinceLastSqWvTrans -= SQWV_HALF_PERIOD_US;
	//	float val = 4096*FinalVal/FULLSCALE_FINALVAL;
	//	analogWrite(A21, (int)val);
	//}
	//if (sinceLastSqWvTrans > USEC_PER_SAMPLE*10)
	//{
	//	sinceLastSqWvTrans -= USEC_PER_SAMPLE * 10;
	//	digitalWrite(SQWV_OUTPUT_PIN, !digitalRead(SQWV_OUTPUT_PIN));
	//	//float val = 4096*FinalVal/FULLSCALE_FINALVAL;
	//	//analogWrite(A21, (int)val);
	//}
}//loop
