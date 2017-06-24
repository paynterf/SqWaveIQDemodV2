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

#include <ADC.h>
ADC *adc = new ADC(); // adc object;


#pragma region ProgConsts
const int OUTPUT_PIN = 32; //lower left pin
const int IRDET1_PIN = A0; //aka pin 14

//const int SQWAVE_FREQ_HZ = 520; //approximate
//const int USEC_PER_SAMPLE = 1E6 / (SQWAVE_FREQ_HZ * SAMPLES_PER_CYCLE);
const int SAMPLES_PER_CYCLE = 20;
const int GROUPS_PER_CYCLE = 4; 
const int SAMPLES_PER_GROUP = SAMPLES_PER_CYCLE / GROUPS_PER_CYCLE;
const float USEC_PER_SAMPLE = 95.7; //value that most nearly zeroes beat-note
const int RUNNING_SUM_LENGTH = 64;
const int NUMSENSORS = 4;
const int aSensorPins[NUMSENSORS] = { A0, A1, A2, A3 };
const int aMultVal_I[GROUPS_PER_CYCLE] = { 1, 1, -1, -1 };
const int aMultVal_Q[GROUPS_PER_CYCLE] = { -1, 1, 1, -1 };
#pragma endregion Program Constants

#pragma region ProgVars
int aSampleSum[NUMSENSORS];//sample sum for each channel
int aCycleGroupSum_Q[NUMSENSORS];//cycle sum for each channel, Q component
int aCycleGroupSum_I[NUMSENSORS];//cycle sum for each channel, I component
int aCycleSum_Q[NUMSENSORS][RUNNING_SUM_LENGTH];//running cycle sums for each channel, Q component
int aCycleSum_I[NUMSENSORS][RUNNING_SUM_LENGTH];//running cycle sums for each channel, I component
int RunningSumInsertionIndex = 0;
int aRunningSum_Q[NUMSENSORS];//overall running sum for each channel, Q component
int aRunningSum_I[NUMSENSORS];//overall running sum for each channel, I component
int aFinalVal[NUMSENSORS];//final value = abs(I)+abs(Q) for each channel

//debugging variables
int aRawData[NUMSENSORS][RUNNING_SUM_LENGTH*SAMPLES_PER_CYCLE];//1280 elements per sensor
int aSampleGroupSums_I[NUMSENSORS][RUNNING_SUM_LENGTH*SAMPLES_PER_CYCLE];//1280 total, 256 non-zero elements 
int aSampleGroupSums_Q[NUMSENSORS][RUNNING_SUM_LENGTH*SAMPLES_PER_CYCLE];//1280 total, 256 non-zero elements
int sample_count = 0; //this counts from 0 to 1279


#pragma endregion Program Variables

elapsedMicros sinceLastOutput;
int SampleSumCount; //sample sums taken so far. range is 0-4
int CycleGroupSumCount; //cycle group sums taken so far. range is 0-3

void setup()
{
	Serial.begin(115200);
	pinMode(OUTPUT_PIN, OUTPUT);
	pinMode(IRDET1_PIN, INPUT);
	digitalWrite(OUTPUT_PIN, LOW);

	//decreases conversion time from ~15 to ~5uSec
	adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
	adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
	adc->setResolution(12);
	adc->setAveraging(1);

	//init aSampleSum & RunningSum arrays
	for (int i = 0; i < NUMSENSORS; i++)
	{
		aSampleSum[i] = 0;
		aRunningSum_I[i] = 0;
		aRunningSum_Q[i] = 0;
	}

	//init aCycleSum I & Q arrays
	for (int i = 0; i < NUMSENSORS; i++)
	{
		for (size_t m = 0; m < RUNNING_SUM_LENGTH; m++)
		{
			aCycleSum_I[i][m] = 0;
			aCycleSum_Q[i][m] = 0;
		}
	}

	////now print them aCycleSum I & Q arrays
	//Serial.println("#\tCSumI1\tCSumQ1\tCSumI2\tCSumQ2\tCSumI3\tCSumQ3\tCSumI4\tCSumQ4");
	//for (size_t m = 0; m < RUNNING_SUM_LENGTH; m++)
	//{
	//	Serial.print(m); Serial.print("\t");
	//	for (int i = 0; i < NUMSENSORS; i++)
	//	{
	//		Serial.print(aCycleSum_I[i][m]); Serial.print("\t");
	//		Serial.print(aCycleSum_Q[i][m]); Serial.print("\t");
	//	}
	//	Serial.println();
	//}

	//init debugging arrays
	for (int i = 0; i < NUMSENSORS; i++)
	{
		for (int k = 0; k < RUNNING_SUM_LENGTH*SAMPLES_PER_CYCLE; k++)
		{
			aSampleGroupSums_I[i][k] = 0;
			aSampleGroupSums_Q[i][k] = 0;
		}
	}

}

void loop()
{
	//this runs every 95.7uSec
	if (sinceLastOutput > 95.7)// stopped
	{
		sinceLastOutput = 0;

		//start of timing pulse
		digitalWrite(OUTPUT_PIN, HIGH);

		//	Step1: Collect a 1/4 cycle group of samples for each channel and sum them into a single value
		// this section executes each USEC_PER_SAMPLE period
		//Serial.print("SampleSumCount = "); Serial.println(SampleSumCount);
		//Serial.print("sample_count = "); Serial.println(sample_count); //counts samples *per sensor channel*
		for (int i = 0; i < NUMSENSORS; i++)
		{
			//07/20/17 rev to capture raw data for later readout
			int samp = adc->analogRead(aSensorPins[i]);
////DEBUG!!
//			aRawData[i][sample_count] = samp; //debugging array
////DEBUG!!
			aSampleSum[i] += samp;
		}
		SampleSumCount++; //goes from 0 to SAMPLES_PER_GROUP-1
		sample_count++; //checked in step 6 to see if data capture is complete

		//	Step2: Every 5th acquisition cycle, assign the appropriate multiplier to form I & Q channel values, and
		//		   add the result to the current cycle sum I & Q variables respectively
		if(SampleSumCount == SAMPLES_PER_GROUP) //an entire sample group sum from each channel is ready for further processing
		{
			SampleSumCount = 0; //starts a new sample group

			//Serial.print("CycleGroupSumCount = "); Serial.println(CycleGroupSumCount);
			//process all 4 sensor channels at the same time
			for (int j = 0; j < NUMSENSORS; j++)
			{
				//compute new group sum for each channel.  Sign depends on which  
				//quarter cycle the sample group is from, but all four channels get same sign set
				//CycleGroupSumCount ranges from 0 to 3
				int groupsum_I = aSampleSum[j] * aMultVal_I[CycleGroupSumCount];
				int groupsum_Q = aSampleSum[j] * aMultVal_Q[CycleGroupSumCount];

////DEBUG!!
//				aSampleGroupSums_I[j][sample_count-1] = groupsum_I; //capture for debug
//				aSampleGroupSums_Q[j][sample_count-1] = groupsum_Q; //capture for debug
////DEBUG!!

				//reset aSampleSum
				aSampleSum[j] = 0;

				aCycleGroupSum_I[j] += groupsum_I; //add new I comp to cycle sum
				aCycleGroupSum_Q[j] += groupsum_Q; //add new Q comp to cycle sum
			}

			CycleGroupSumCount++;
		}//if(SampleSumCount == SAMPLES_PER_GROUP)

		//	Step3: After 4 such groups have been collected and processed into full-cycle
		//		   I & Q sums, add them to their respective N-cycle running totals
		if(CycleGroupSumCount == GROUPS_PER_CYCLE) //now have a complete cycle sum for I & Q - load into running sum circ buff
		{
			CycleGroupSumCount = 0; //start a new cycle group next time

			////now print them aCycleSum I & Q arrays
			//Serial.println("#\tCSumI1\tCSumQ1\tCSumI2\tCSumQ2\tCSumI3\tCSumQ3\tCSumI4\tCSumQ4");
			//for (size_t m = 0; m < RUNNING_SUM_LENGTH; m++)
			//{
			//	Serial.print(m); Serial.print("\t"); Serial.print("s#: "); Serial.print(sample_count); Serial.print("\t"); Serial.print("RSI: "); Serial.print(RunningSumInsertionIndex); Serial.print("\t");
			//	for (int i = 0; i < NUMSENSORS; i++)
			//	{
			//		Serial.print(aCycleSum_I[i][m]); Serial.print("\t");
			//		Serial.print(aCycleSum_Q[i][m]); Serial.print("\t");
			//	}
			//	Serial.println();
			//}

			Serial.print(RunningSumInsertionIndex); Serial.print("\t");
			for (int k = 0; k < NUMSENSORS; k++)
			{
				//	Step4: Subtract the oldest cycle sums from their respective N-cycle totals, and
				//		   then overwrite these values with the new ones
				int oldestvalue_I = aCycleSum_I[k][RunningSumInsertionIndex];
				int oldestvalue_Q = aCycleSum_Q[k][RunningSumInsertionIndex];
				aRunningSum_I[k] = aRunningSum_I[k] + aCycleGroupSum_I[k] - oldestvalue_I;
				aRunningSum_Q[k] = aRunningSum_Q[k] + aCycleGroupSum_Q[k] - oldestvalue_Q;

				aCycleSum_I[k][RunningSumInsertionIndex] = aCycleGroupSum_I[k];
				aCycleSum_Q[k][RunningSumInsertionIndex] = aCycleGroupSum_Q[k];

				//reset cycle group sum values for next group
				aCycleGroupSum_I[k] = 0;
				aCycleGroupSum_I[k] = 0;

				//	Step5: Compute the final demodulate sensor value by summing the absolute values
				//		   of the I & Q running sums
				int RS_I = aRunningSum_I[k]; int RS_Q = aRunningSum_Q[k];
				aFinalVal[k] = abs((int)RS_I) + abs((int)RS_Q);
				Serial.print(aFinalVal[k]); Serial.print("\t");
			}
			Serial.println();

			//	Step6: Update buffer indicies so that they point to the new 'oldest value' for 
			//		   the next cycle
			RunningSumInsertionIndex++;
			//Serial.print("RunningSumInsertionIndex = "); Serial.println(RunningSumInsertionIndex);

			//06/20/17 revised for debugging
			if (RunningSumInsertionIndex >= RUNNING_SUM_LENGTH)
			{

				//Serial.println("samp#\traw\tSSumI1\tSSumQ1\tCSumI1\tCSumQ1\tRSumI\tRSumQ\tFVal");

				RunningSumInsertionIndex = 0;

			//	//print out raw data array for channel 1
			//	//for (int j = 0; j < RUNNING_SUM_LENGTH*SAMPLES_PER_CYCLE;j++)
			//	for (int j = 0; j < 100;j++)
			//	{
			//		//for (int i = 0; i < NUMSENSORS; i++)
			//		//{
			//		//	Serial.print(aRawData[i][j]); Serial.print("\t");
			//		//}

			//		//06/21/17 now just printing ch1
			//		Serial.print(j); Serial.print(":\t");
			//		Serial.print(aRawData[0][j]); Serial.print("\t");
			//		Serial.print(aSampleGroupSums_I[0][j]); Serial.print("\t");
			//		Serial.print(aSampleGroupSums_Q[0][j]); Serial.print("\t");

			//		if (j > 0 && (j+1) % SAMPLES_PER_CYCLE == 0)
			//		{
			//			//Serial.print("RSI = "); Serial.print(RunningSumInsertionIndex); Serial.print("\t");
			//			Serial.print(aCycleSum_I[0][RunningSumInsertionIndex]); Serial.print("\t");
			//			Serial.print(aCycleSum_Q[0][RunningSumInsertionIndex]); Serial.print("\t");
			//			Serial.print(aRunningSum_I[0]); Serial.print("\t");Serial.print(aRunningSum_Q[0]); Serial.print("\t");
			//			Serial.print(aFinalVal[0]);
			//			RunningSumInsertionIndex++;

			//		}
			//		//Serial.print(aSampleGroupSums_I[2][j]); Serial.print("\t");
			//		//Serial.print(aSampleGroupSums_Q[2][j]); Serial.print("\t");
			//		//Serial.print(aSampleGroupSums_I[3][j]); Serial.print("\t");
			//		//Serial.print(aSampleGroupSums_Q[3][j]);

			//		Serial.println();
			//	}

			//	//print out aCycleSum_I & Q arrays
			//	Serial.println();
			//	Serial.println("aCycleSum array value for sensor channel 0"); Serial.println();
			//	Serial.println("#\tCSumI1\tCSumQ1\tCSumI2\tCSumQ2");
			//	for (size_t m = 0; m < RUNNING_SUM_LENGTH; m++)
			//	{
			//		Serial.print(m); Serial.print("\t");
			//		for (size_t i = 0; i < 2; i++)
			//		{
			//			Serial.print(aCycleSum_I[i][m]); Serial.print("\t");
			//			Serial.print(aCycleSum_Q[i][m]);; Serial.print("\t");
			//		}
			//		Serial.println();
			//	}

			//	//stop the processor
			//	while (1)
			//	{

			//	}
			//	Serial.println("this should never print");

			}//if (RunningSumInsertionIndex >= RUNNING_SUM_LENGTH)
		}

		//end of timing pulse
		digitalWrite(OUTPUT_PIN, LOW);
	}//else
}//loop
