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
const int TIMING_PULSE_OUT_PIN = 32; //lower left pin
const int TIMING_SQWAVE_OUT_PIN = 31; //up one more
const int IRDET1_IN_PIN = A0; //aka pin 14
const int IRDET2_IN_PIN = A1; //aka pin 15
const int CH1_ANALOG_OUT_PIN = A21; //DAC0 added 07/11/17
const int CH2_ANALOG_OUT_PIN = A22; //DAC1 added 07/19/17

//07/11/17 added sweep gen synch pins
const int DEMOD_SYNCH_IN_PIN = 1;
const int DEMOD_SYNCH_OUT_PIN = 2;

//06/26/17 added for freq matching with xmit board
const double SQWAVE_FREQ_HZ = 518.5; //stopped!!!
//const double SQWAVE_FREQ_HZ = 51.85; //stopped!!! 07/04/17 rev for debug
const double SQWV_PERIOD_US = 1000000 / SQWAVE_FREQ_HZ; //period in Usec

const int SAMPLES_PER_CYCLE = 20;
const int GROUPS_PER_CYCLE = 4; 
const int SAMPLES_PER_GROUP = SAMPLES_PER_CYCLE / GROUPS_PER_CYCLE;
const float USEC_PER_SAMPLE = SQWV_PERIOD_US / SAMPLES_PER_CYCLE; //sample period, Usec
const int RUNNING_SUM_LENGTH = 64;
const int SENSOR_PIN = A0;
const int aMultVal_I[GROUPS_PER_CYCLE] = { 1, 1, -1, -1 };
const int aMultVal_Q[GROUPS_PER_CYCLE] = { -1, 1, 1, -1 };
const int FULLSCALE_FinalVal_COUNT = 2621440; //07/02/17 full-scale final value --> 3.3v DAC output
const float FULLSCALE_DAC_COUNT = 4096; //07/02/17 full-scale final value --> 3.3v DAC output
#pragma endregion Program Constants

#pragma region ProgVars
//Channel 1
int SampleSum1 = 0;//sample sum for channel1
int CycleGroupSum1_Q = 0;//cycle sum for channel1, Q component
int CycleGroupSum1_I = 0;//cycle sum forchannel1, I component
int aCycleSum1_Q[RUNNING_SUM_LENGTH];//running cycle sums for channel1, Q component
int aCycleSum1_I[RUNNING_SUM_LENGTH];//running cycle sums for channel1, I component
int RunningSum1_Q = 0;//overall running sum for channel1, Q component
int RunningSum1_I = 0;//overall running sum for channel1, I component
int FinalVal1 = 0;//final value = abs(I)+abs(Q) for channel1

//Channel 2
int SampleSum2 = 0;//sample sum for channel2
int CycleGroupSum2_Q = 0;//cycle sum for channel2, Q component
int CycleGroupSum2_I = 0;//cycle sum forchannel2, I component
int aCycleSum2_Q[RUNNING_SUM_LENGTH];//running cycle sums for channel2, Q component
int aCycleSum2_I[RUNNING_SUM_LENGTH];//running cycle sums for channel2, I component
int RunningSum2_Q = 0;//overall running sum for channel1, Q component
int RunningSum2_I = 0;//overall running sum for channel1, I component
int FinalVal2 = 0;//final value = abs(I)+abs(Q) for channel2

//common to both channels
int RunningSumInsertionIndex = 0;
elapsedMicros sinceLastSample;
int SampleSumCount; //sample sums taken so far. range is 0-4
int CycleGroupSumCount; //cycle group sums taken so far. range is 0-3
#pragma endregion Program Variables


//debugging variables
#pragma region DebugVars
//const int FINVAL_CAPTURE_LENGTH = 6000; //rev 07/12/17
//const int FINVAL_CAPTURE_LENGTH = 12000; //rev 07/12/17
//float aFinalVals1[FINVAL_CAPTURE_LENGTH]; //debugging array for Ch1
//float aFinalVals2[FINVAL_CAPTURE_LENGTH]; //debugging array for Ch2
//const int SAMPTIMES_CAPTURE_LENGTH = SAMPLES_PER_CYCLE * RUNNING_SUM_LENGTH *3;
//float aSampTimes[SAMPTIMES_CAPTURE_LENGTH];
//long aTimeStamps[FINVAL_CAPTURE_LENGTH]; //added 07/17/17
long startMicroSec = 0; //added 07/17/17
int sample_count = 0; //this counts from 0 to 1279
int finvalidx = 0;
int num_sample_pulses = 0; //used for square-wave generation
bool bDoneRecordingFinVals = false;
#pragma endregion Program Variables


void setup()
{
	Serial.begin(115200);
	pinMode(TIMING_PULSE_OUT_PIN, OUTPUT);
	pinMode(TIMING_SQWAVE_OUT_PIN, OUTPUT); //added 06/26/17 for frequency matching with xmit board
	pinMode(IRDET1_IN_PIN, INPUT);
	pinMode(IRDET2_IN_PIN, INPUT);
	digitalWrite(TIMING_PULSE_OUT_PIN, LOW);

	//07/11/17 added sweep gen synch pins
	pinMode(DEMOD_SYNCH_IN_PIN, INPUT_PULLDOWN);
	pinMode(DEMOD_SYNCH_OUT_PIN, OUTPUT);
	digitalWrite(DEMOD_SYNCH_OUT_PIN, LOW);
	digitalWrite(DEMOD_SYNCH_IN_PIN, LOW);

	pinMode(CH1_ANALOG_OUT_PIN, OUTPUT); //analog output pin
	pinMode(CH2_ANALOG_OUT_PIN, OUTPUT); //analog output pin

//DEBUG!!
	Serial.print("USEC_PER_SAMPLE = "); Serial.println(USEC_PER_SAMPLE);
	Serial.print("SQWV_PERIOD_US = "); Serial.println(SQWV_PERIOD_US);

	//Initialize both ADC's

	//ADC_0
	adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED, ADC_0);
	adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_0);
	adc->setResolution(12, ADC_0);
	adc->setAveraging(1, ADC_0);

	//ADC_1
	adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED, ADC_1);
	adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_1);
	adc->setResolution(12, ADC_1);
	adc->setAveraging(1, ADC_1);

	//init aCycleSum I & Q arrays
	for (size_t m = 0; m < RUNNING_SUM_LENGTH; m++)
	{
		//Channel 1
		aCycleSum1_I[m] = 0;
		aCycleSum1_Q[m] = 0;

		//Channel 2
		aCycleSum2_I[m] = 0;
		aCycleSum2_Q[m] = 0;
	}

	////init Final Value & Timestamp arrays
	//for (size_t m = 0; m < FINVAL_CAPTURE_LENGTH; m++)
	//{
	//	aFinalVals1[m] = 0;
	//	aFinalVals2[m] = 0;
	//	aTimeStamps[m] = 0;
	//}

	//07/02/17 for final value analog output
	analogWriteResolution(12);
	analogReference(0); //default - 3.3VDC

	//07/16/17 halt here until triggered by sweep gen
	while (digitalRead(DEMOD_SYNCH_IN_PIN) == LOW)
	{
		Serial.println("Waiting for trigger...");
		delay(1000);
	}
	Serial.println("Starting...");

	sinceLastSample = 0; //need to init this var, or it can be a VERY big number to start
}

void loop()
{
	//this runs every 95.7uSec
	if (sinceLastSample > USEC_PER_SAMPLE)
	{
		//if (sample_count < SAMPTIMES_CAPTURE_LENGTH)
		//{
		//	aSampTimes[sample_count] = sinceLastSample;
		//}
		//sample_count++;
		sinceLastSample -= USEC_PER_SAMPLE;

		//start of timing pulse
		digitalWrite(TIMING_PULSE_OUT_PIN, HIGH);

		//	Step1: Collect a 1/4 cycle group of samples and sum them into a single value
		// this section executes each USEC_PER_SAMPLE period
		int samp1 = adc->analogRead(IRDET1_IN_PIN);
		int samp2 = adc->analogRead(IRDET2_IN_PIN);

		SampleSum1 += samp1;
		SampleSum2 += samp2;

		SampleSumCount++; //goes from 0 to SAMPLES_PER_GROUP-1

		//	Step2: Every 5th acquisition cycle, assign the appropriate multiplier to form I & Q channel values, and
		//		   add the result to the current cycle sum I & Q variables respectively
		if(SampleSumCount == SAMPLES_PER_GROUP) //an entire sample group sum from each channel is ready for further processing
		{
			SampleSumCount = 0; //starts a new sample group

			//compute new group sum.  Sign depends on which quarter cycle the sample group is from 
			//CycleGroupSumCount ranges from 0 to 3

			//Ch1
			int groupsum1_I = SampleSum1 * aMultVal_I[CycleGroupSumCount];
			int groupsum1_Q = SampleSum1 * aMultVal_Q[CycleGroupSumCount];
			CycleGroupSum1_I += groupsum1_I; //add new I comp to cycle sum
			CycleGroupSum1_Q += groupsum1_Q; //add new Q comp to cycle sum

			//Ch2
			int groupsum2_I = SampleSum2 * aMultVal_I[CycleGroupSumCount];
			int groupsum2_Q = SampleSum2 * aMultVal_Q[CycleGroupSumCount];
			CycleGroupSum2_I += groupsum2_I; //add new I comp to cycle sum
			CycleGroupSum2_Q += groupsum2_Q; //add new Q comp to cycle sum

			//common
			SampleSum1 = 0;
			SampleSum2 = 0;
			CycleGroupSumCount++;
		}//if(SampleSumCount == SAMPLES_PER_GROUP)

		//	Step3: After 4 such groups have been collected and processed into full-cycle
		//		   I & Q sums, add them to their respective N-cycle running totals
		if(CycleGroupSumCount == GROUPS_PER_CYCLE) //now have a complete cycle sum for I & Q - load into running sum circ buff
		{
			CycleGroupSumCount = 0; //start a new cycle group next time

			//	Step4: Subtract the oldest cycle sum from the N-cycle total, and
			//		   then overwrite this values with the new one

			//Ch1
			int oldestvalue1_I = aCycleSum1_I[RunningSumInsertionIndex];
			int oldestvalue1_Q = aCycleSum1_Q[RunningSumInsertionIndex];
			RunningSum1_I = RunningSum1_I + CycleGroupSum1_I - oldestvalue1_I;
			RunningSum1_Q = RunningSum1_Q + CycleGroupSum1_Q - oldestvalue1_Q;

			//Ch2
			int oldestvalue2_I = aCycleSum2_I[RunningSumInsertionIndex];
			int oldestvalue2_Q = aCycleSum2_Q[RunningSumInsertionIndex];
			RunningSum2_I = RunningSum2_I + CycleGroupSum2_I - oldestvalue2_I;
			RunningSum2_Q = RunningSum2_Q + CycleGroupSum2_Q - oldestvalue2_Q;

			//	Step5: Compute the final demodulate sensor value by summing the absolute values
			//		   of the I & Q running sums

			//ch1
			int RS1_I = RunningSum1_I; int RS1_Q = RunningSum1_Q;
			FinalVal1 = abs((int)RS1_I) + abs((int)RS1_Q);
			int RS2_I = RunningSum2_I; int RS2_Q = RunningSum2_Q;
			FinalVal2 = abs((int)RS2_I) + abs((int)RS2_Q);

			float FinalVal1AnalogOut = FULLSCALE_DAC_COUNT * ((float)FinalVal1 / (float)FULLSCALE_FinalVal_COUNT);
			analogWrite(CH1_ANALOG_OUT_PIN, (int)FinalVal1AnalogOut); //added 07/11/17

			float FinalVal2AnalogOut = FULLSCALE_DAC_COUNT * ((float)FinalVal2 / (float)FULLSCALE_FinalVal_COUNT);
			analogWrite(CH2_ANALOG_OUT_PIN, (int)FinalVal2AnalogOut); //added 07/11/17

//DEBUG!!
			////07/11/17 rev to synch with sweepgen
			//if (finvalidx < FINVAL_CAPTURE_LENGTH)
			//{
			//	if (finvalidx == 0)
			//	{
			//		Serial.println("Finval Recording Started");
			//		digitalWrite(DEMOD_SYNCH_OUT_PIN, HIGH); //handshake with sweepgen
			//		startMicroSec = micros();
			//	}

			//	//capture final value & timestamp
			//	aFinalVals1[finvalidx] = FinalVal1;
			//	aFinalVals2[finvalidx] = FinalVal2;
			//	aTimeStamps[finvalidx] = micros() - startMicroSec;
			//	finvalidx++;

				//float FinalVal1AnalogOut = FULLSCALE_DAC_COUNT * ((float)FinalVal1 / (float)FULLSCALE_FinalVal_COUNT);
				//analogWrite(CH1_ANALOG_OUT_PIN, (int)FinalVal1AnalogOut); //added 07/11/17

				//float FinalVal2AnalogOut = FULLSCALE_DAC_COUNT * ((float)FinalVal2 / (float)FULLSCALE_FinalVal_COUNT);
				//analogWrite(CH2_ANALOG_OUT_PIN, (int)FinalVal2AnalogOut); //added 07/11/17
			//}
			//else
			//{
			//	digitalWrite(DEMOD_SYNCH_OUT_PIN, LOW); //handshake with sweepgen
			//	finvalidx = 0;
			//	bDoneRecordingFinVals = true;
			//	Serial.print("Finval Recording Stopped.  Print y/n (y)? ");

			//	//wait for input
			//	while (Serial.available() == 0)
			//	{
			//		Serial.println("Waiting for input...");
			//		delay(1000);
			//	}

			//	String res = Serial.readString();
			//	Serial.println(res.substring(0));
			//	res.trim();
			//	if (!res.equalsIgnoreCase('N'))
			//	{
			//		Serial.println("TStamp\tFinValue1\tFinValue2");
			//		for (size_t i = 0; i < FINVAL_CAPTURE_LENGTH; i++)
			//		{
			//			if (aFinalVals1[i] > 2000)
			//			{
			//				Serial.print(aTimeStamps[i]); 
			//				Serial.print("\t");
			//				Serial.print(aFinalVals1[i]);
			//				Serial.print("\t");
			//				Serial.println(aFinalVals2[i]);
			//			}
			//		}

			//		//Serial.println("SampIdx\tTStamp");
			//		//for (size_t i = 0; i < SAMPTIMES_CAPTURE_LENGTH; i++)
			//		//{
			//		//	Serial.print(i); 
			//		//	Serial.print("\t");
			//		//	Serial.print(aSampTimes[i]);
			//		//	Serial.println();
			//		//}
			//	}
			//	Serial.println("Exiting - Bye!!");
			//	while (1); //start infinite loop
			//}
//DEBUG!!

			aCycleSum1_I[RunningSumInsertionIndex] = CycleGroupSum1_I;
			aCycleSum1_Q[RunningSumInsertionIndex] = CycleGroupSum1_Q;
			aCycleSum2_I[RunningSumInsertionIndex] = CycleGroupSum2_I;
			aCycleSum2_Q[RunningSumInsertionIndex] = CycleGroupSum2_Q;

			//reset cycle group sum values for next group
			CycleGroupSum1_I = 0;
			CycleGroupSum1_Q = 0;
			CycleGroupSum2_I = 0;
			CycleGroupSum2_Q = 0;

			////	Step6: Update buffer indicies so that they point to the new 'oldest value' for 
			////		   the next cycle
			RunningSumInsertionIndex++;
		}

		//06/20/17 revised for debugging
		if (RunningSumInsertionIndex >= RUNNING_SUM_LENGTH)
		{
			RunningSumInsertionIndex = 0;
		}

		//generate square wave output for synch with sweep gen
		num_sample_pulses++; //used for square-wave generation

		if (num_sample_pulses == SAMPLES_PER_CYCLE / 2)
		{
			digitalWrite(TIMING_SQWAVE_OUT_PIN, !digitalRead(TIMING_SQWAVE_OUT_PIN));
			num_sample_pulses = 0;
		}

		//end of timing pulse
		digitalWrite(TIMING_PULSE_OUT_PIN, LOW);

	}//if (sinceLastSample > 95.7)
}//loop
