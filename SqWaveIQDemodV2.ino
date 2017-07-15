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
//07/11/17 rev to synch with sweep generator module
//07/12/17 rev to go back to free-run mode - no printouts, no data capture, analog output

#include <ADC.h> //Analog-to-Digital library
//#include <SD.h> //SD card library

ADC *adc = new ADC(); // adc object;


#pragma region ProgConsts
const int OUTPUT_PIN = 32; //lower left pin
const int SQWV_OUTPUT_PIN = 31; //next to bottom pin, left side
const int IRDET1_PIN = A0; //aka pin 14
const int DAC_PIN = A21; //DAC0 added 07/11/17

//07/11/17 added sweep gen synch pins
const int DEMOD_SYNCH_IN_PIN = 1;
const int DEMOD_SYNCH_OUT_PIN = 2;

//const int FINVAL_CAPTURE_LENGTH = 3000; //rev 07/12/17
const int FINVAL_CAPTURE_LENGTH = 12000; //rev 07/12/17
float aFinalVals[FINVAL_CAPTURE_LENGTH]; //debugging array

//06/26/17 added for freq matching with xmit board
//const double SQWAVE_FREQ_HZ = 518.5; //stopped!!!
const double SQWAVE_FREQ_HZ = 51.85; //stopped!!! 07/04/17 rev for debug
//const double SQWV_HALF_PERIOD_US = 500000 / SQWAVE_FREQ_HZ; //half-period in Usec
const double SQWV_PERIOD_US = 1000000 / SQWAVE_FREQ_HZ; //period in Usec

const int SAMPLES_PER_CYCLE = 20;
const int GROUPS_PER_CYCLE = 4; 
const int SAMPLES_PER_GROUP = SAMPLES_PER_CYCLE / GROUPS_PER_CYCLE;
//const float USEC_PER_SAMPLE = 95.7; //value that most nearly zeroes beat-note
const float USEC_PER_SAMPLE = SQWV_PERIOD_US / SAMPLES_PER_CYCLE; //sample period, Usec
//const int RUNNING_SUM_LENGTH = 64;
const int RUNNING_SUM_LENGTH = 32; //07/15/17
const int SENSOR_PIN = A0;
const int aMultVal_I[GROUPS_PER_CYCLE] = { 1, 1, -1, -1 };
const int aMultVal_Q[GROUPS_PER_CYCLE] = { -1, 1, 1, -1 };
const int FULLSCALE_FINALVAL_COUNT = 2621440; //07/02/17 full-scale final value --> 3.3v DAC output
const float FULLSCALE_DAC_COUNT = 4096; //07/02/17 full-scale final value --> 3.3v DAC output
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
//int aSampleGroupSums_I[RUNNING_SUM_LENGTH*SAMPLES_PER_CYCLE];//1280 total, 256 non-zero elements 
//int aSampleGroupSums_Q[RUNNING_SUM_LENGTH*SAMPLES_PER_CYCLE];//1280 total, 256 non-zero elements
//int sample_count = 0; //this counts from 0 to 1279
int finvalidx = 0;
//elapsedMicros sinceLastSqWvTrans = 0; //06/26/17 added for freq matching with xmit board
//int num_sample_pulses = 0; //used for square-wave generation
bool bDoneRecordingFinVals = false;


#pragma endregion Program Variables

elapsedMicros sinceLastSample;
int SampleSumCount; //sample sums taken so far. range is 0-4
int CycleGroupSumCount; //cycle group sums taken so far. range is 0-3


void setup()
{
	Serial.begin(115200);
	pinMode(OUTPUT_PIN, OUTPUT);
	pinMode(SQWV_OUTPUT_PIN, OUTPUT); //added 06/26/17 for frequency matching with xmit board
	pinMode(IRDET1_PIN, INPUT);
	digitalWrite(OUTPUT_PIN, LOW);

	//07/11/17 added sweep gen synch pins
	pinMode(DEMOD_SYNCH_IN_PIN, INPUT_PULLDOWN);
	pinMode(DEMOD_SYNCH_OUT_PIN, OUTPUT);
	digitalWrite(DEMOD_SYNCH_OUT_PIN, LOW);

	pinMode(DAC_PIN, OUTPUT); //analog output pin

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

	sinceLastSample = 0;

	//07/02/17 for final value analog output
	analogWriteResolution(12);
	analogReference(0); //default - 3.3VDC

}

void loop()
{
	//this runs every 95.7uSec
	if (sinceLastSample > USEC_PER_SAMPLE)
	{
//DEBUG!!

		//aSampTimes[sample_count] = sinceLastSample;
		//if (sample_count > SAMPTIMES_CAPTURE_LENGTH-1)
		//{
		//	Serial.print("Sample times capture length exceeded.  Print y/n (y)? ");
		//	while (Serial.available() == 0); //waits for input
		//	String res = Serial.readString();
		//	Serial.println(res.substring(0));
		//	res.trim();
		//	if (!res.equalsIgnoreCase('N'))
		//	{
		//		Serial.println("Elapsed Usec between samples");
		//		for (size_t i = 0; i < SAMPTIMES_CAPTURE_LENGTH; i++)
		//		{
		//			Serial.println(aSampTimes[i]);
		//		}

		//		Serial.println("Final Values");
		//		for (size_t i = 0; i < FINVAL_CAPTURE_LENGTH; i++)
		//		{
		//			Serial.println(aFinalVals[i]);
		//		}
		//	}

		//	Serial.println("Exiting - Bye!");
		//	while (1);
		//}
		//sample_count++;
//DEBUG!!
		//sinceLastSample = 0;
		sinceLastSample -= USEC_PER_SAMPLE;

		//start of timing pulse
		digitalWrite(OUTPUT_PIN, HIGH);

		//	Step1: Collect a 1/4 cycle group of samples and sum them into a single value
		// this section executes each USEC_PER_SAMPLE period
		int samp = adc->analogRead(SENSOR_PIN);
		SampleSum += samp;
		SampleSumCount++; //goes from 0 to SAMPLES_PER_GROUP-1
		//sample_count++; //checked in step 6 to see if data capture is complete

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
			//07/11/17 rev to synch with sweepgen
			if (digitalRead(DEMOD_SYNCH_IN_PIN) == HIGH && !bDoneRecordingFinVals)
			{
				if (finvalidx == 0)
				{
					Serial.println("Finval Recording Started");
					//Serial.print("Cycle\tFinval");
					digitalWrite(DEMOD_SYNCH_OUT_PIN, HIGH); //handshake with sweepgen
				}

				if (finvalidx < FINVAL_CAPTURE_LENGTH)
				{
					aFinalVals[finvalidx] = FinalVal;
					finvalidx++;
					float FinalValAnalogOut = FULLSCALE_DAC_COUNT * ((float)FinalVal / (float)FULLSCALE_FINALVAL_COUNT);
					//Serial.print(finvalidx);Serial.print(": FinalValAnalogOut = "); Serial.println((int)FinalValAnalogOut);
					//Serial.print(finvalidx);Serial.print("\t"); Serial.println((int)FinalValAnalogOut);
					//Serial.print(finvalidx);Serial.print("\t"); 
					//Serial.print(FinalVal);Serial.print("\t"); 
					//Serial.println((int)FinalValAnalogOut);
					analogWrite(DAC_PIN, (int)FinalValAnalogOut); //added 07/11/17
				}
				else
				{
					digitalWrite(DEMOD_SYNCH_OUT_PIN, LOW); //handshake with sweepgen
					finvalidx = 0;
					bDoneRecordingFinVals = true;
					Serial.print("Finval Recording Stopped.  Print y/n (y)? ");

					//wait for input
					while (Serial.available() == 0)
					{
						Serial.println("Waiting for input...");
						delay(1000);
					}
					String res = Serial.readString();
					Serial.println(res.substring(0));
					res.trim();
					if (!res.equalsIgnoreCase('N'))
					{
						Serial.println("Final Values");
						for (size_t i = 0; i < FINVAL_CAPTURE_LENGTH; i++)
						{
							Serial.println(aFinalVals[i]);
						}
					}
					Serial.println("Exiting - Bye!!");
				}
		
			}
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
			//sample_count = 0; 
			//Serial.print(FinalVal);
			//Serial.println();
		}//if (RunningSumInsertionIndex >= RUNNING_SUM_LENGTH)
	}//if (sinceLastSample > 95.7)
}//loop
