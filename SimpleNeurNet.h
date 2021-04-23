/******************************************************************
 * Generic NN
 * All basic settings can be controlled via the Network Configuration
 * section.
 * See robotics.hobbizine.com/arduinoann.html for details.
 ******************************************************************/

//#include <stdio.h>
#include <stdlib.h>  
#include <math.h>

int initNeurNet();
long trainNeurNet(long MaxTrainingCycles);
void calcNeurNet();
float actFunc(float input);
void normTrainingData();
void printTrainingData();

/* Network Configuration - customized per network */
const int NrPattern = 7;
const int NrInputNodes = 7;
const int NrHiddenNodes = 8;
const int NrOutputNodes = 4;

const float LearningRate = 0.3;
const float Momentum = 0.5;
const float InitialWeightMax = 0.5;
const float Success = 0.0004;

float Input[NrPattern][NrInputNodes] = {
  { 1, 1, 1, 1, 1, 1, 0 },  // 0
  { 0, 1, 1, 0, 0, 0, 0 },  // 1
  { 1, 1, 0, 1, 1, 0, 1 },  // 2
  { 1, 1, 1, 1, 0, 0, 1 },  // 3
  { 0, 1, 1, 0, 0, 1, 1 },  // 4
  { 1, 1, 0, 0, 1, 1, 0 },  // 5
  { 0, 0, 1, 0, 1, 0, 1 },  // 6

};

float Output[NrPattern][NrOutputNodes] = {
  { 0, 0, 0, 0 },  
  { 0, 0, 0, 1 }, 
  { 0, 0, 1, 0 }, 
  { 0, 0, 1, 1 }, 
  { 0, 1, 0, 0 },
  { 0, 1, 0, 1 },
  { 0, 1, 1, 0 },
};

float InputMin[NrInputNodes];
float OutputMin[NrOutputNodes];
float InputMax[NrInputNodes];
float OutputMax[NrOutputNodes];

/* End Network Configuration */
//int i, j, p, q, r;
bool FlgReport1000;
bool FlgNeurNetTrained = 0;
bool FlgNeurNetInitialized = 0;
int RandomizedIndex[NrPattern];
long  TrainingCycle = 0;
float Rando;
float Error;
float Accum;

float HiddenNodes[NrHiddenNodes];
float OutputNodes[NrOutputNodes];
float HiddenNodesWeights[NrInputNodes+1][NrHiddenNodes];
float OutputWeights[NrHiddenNodes+1][NrOutputNodes];
float ChangeHiddenNodesWeights[NrInputNodes+1][NrHiddenNodes];
float ChangeOutputWeights[NrHiddenNodes+1][NrOutputNodes];
 
/*
int main()
{
	
	initNeurNet();
  
	trainNeurNet(20000);
	
	calcNeurNet();

	Serial.print("\n\n");
	Serial.print("Training Set Solved! \n");
	Serial.print("--------\n");
	Serial.print("\n\n");

	return(1);
}
*/
 
int initNeurNet()
{
    if(FlgNeurNetInitialized) return(0);
    
    FlgReport1000 = 1;
    TrainingCycle = 0;
    
	for(int p = 0 ; p < NrPattern ; p++ ) 
	{   RandomizedIndex[p] = p;
	}

	/* Initialize HiddenNodesWeights and ChangeHiddenNodesWeights */
	for(int i = 0 ; i < NrHiddenNodes ; i++ )
	{    
		for(int j = 0 ; j <= NrInputNodes ; j++ ) 
		{   ChangeHiddenNodesWeights[j][i] = 0.0 ;
			Rando = float(rand()%100)/100;
			HiddenNodesWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
		}
	}
  
	/* Initialize OutputWeights and ChangeOutputWeights */
	for(int i = 0 ; i < NrOutputNodes ; i ++ )
	{    
		for(int j = 0 ; j <= NrHiddenNodes ; j++ )
		{
			ChangeOutputWeights[j][i] = 0.0 ;
			Rando = float(rand()%100)/100;        
			OutputWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
		}
	}
    Serial.println("NN initialized.");
    
    FlgNeurNetInitialized = 1;
    return(FlgNeurNetInitialized);
}


long trainNeurNet(long MaxTrainingCycles)
{
    if(TrainingCycle != 0) return(TrainingCycle);
    
	float HiddenNodesDelta[NrHiddenNodes];
	float OutputDelta[NrOutputNodes];

	/* Begin training */
    for(TrainingCycle = 1 ; TrainingCycle < MaxTrainingCycles; TrainingCycle++)
    {
		/* Randomize order of training patterns */
        for(int p = 0 ; p < NrPattern ; p++) 
        {   int q = rand()%NrPattern;
            int r = RandomizedIndex[p] ; 
            RandomizedIndex[p] = RandomizedIndex[q] ; 
            RandomizedIndex[q] = r ;
        }
        Error = 0.0 ;

		/* Cycle through each training pattern in the randomized order */
        for(int q = 0 ; q < NrPattern ; q++ ) 
        {    
            int p = RandomizedIndex[q];

			/* Compute HiddenNodes layer activations */
            for(int i = 0 ; i < NrHiddenNodes ; i++ )
            {   Accum = HiddenNodesWeights[NrInputNodes][i];
                for(int j = 0 ; j < NrInputNodes ; j++ ) 
                {   Accum += Input[p][j] * HiddenNodesWeights[j][i] ;
                }
                HiddenNodes[i] = actFunc(Accum);
            }

			/* Compute OutputNodes layer activations and calculate errors */
			for(int i = 0 ; i < NrOutputNodes ; i++ )
			{   Accum = OutputWeights[NrHiddenNodes][i];
				for(int j = 0 ; j < NrHiddenNodes ; j++ )
				{	Accum += HiddenNodes[j] * OutputWeights[j][i] ;
				}
				OutputNodes[i] = actFunc(Accum);
				OutputDelta[i] = (Output[p][i] - OutputNodes[i]) * OutputNodes[i] * (1.0 - OutputNodes[i]) ;
				Error += 0.5 * (Output[p][i] - OutputNodes[i]) * (Output[p][i] - OutputNodes[i]) ;
			}

			/* Backpropagate errors to HiddenNodes layer */
			for(int i = 0; i < NrHiddenNodes; i++)
			{   Accum = 0.0 ;
				for(int j = 0 ; j < NrOutputNodes ; j++ )
				{	Accum += OutputWeights[i][j] * OutputDelta[j] ;
				}
				HiddenNodesDelta[i] = Accum * HiddenNodes[i] * (1.0 - HiddenNodes[i]) ;
			}

			/* Update Inner-->HiddenNodes Weights */
			for(int i = 0; i < NrHiddenNodes; i++)
			{     
				ChangeHiddenNodesWeights[NrInputNodes][i] = LearningRate * HiddenNodesDelta[i] + Momentum * ChangeHiddenNodesWeights[NrInputNodes][i];
                
				HiddenNodesWeights[NrInputNodes][i] += ChangeHiddenNodesWeights[NrInputNodes][i];
                
				for(int j = 0; j < NrInputNodes; j++)
				{	ChangeHiddenNodesWeights[j][i] = LearningRate * Input[p][j] * HiddenNodesDelta[i] + Momentum * ChangeHiddenNodesWeights[j][i];
					HiddenNodesWeights[j][i] += ChangeHiddenNodesWeights[j][i] ;
				}
			}

			/* Update HiddenNodes-->OutputNodes Weights */
			for(int i = 0 ; i < NrOutputNodes ; i ++ )
			{    
				ChangeOutputWeights[NrHiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[NrHiddenNodes][i];
                
				OutputWeights[NrHiddenNodes][i] += ChangeOutputWeights[NrHiddenNodes][i];
                
				for(int j = 0 ; j < NrHiddenNodes ; j++ )
				{   ChangeOutputWeights[j][i] = LearningRate * HiddenNodes[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i];
                    
					OutputWeights[j][i] += ChangeOutputWeights[j][i];
				}	
			}
		}

		/* Every 1000 cycles send data to terminal for display */
		if (FlgReport1000 && (TrainingCycle%1000 == 0))
		{
			Serial.print("TrainingCycle: ");
			Serial.print(TrainingCycle);
			Serial.print("  Error = ");
			Serial.print(Error,5);
			Serial.print("\n");
		}
		
		/* If error rate is less than pre-determined threshold then end */
		if(Error <= Success )
        {   FlgNeurNetTrained = 1;
            break;
        }
    }      
	
    Serial.print("TrainingCycle: ");
    Serial.print(TrainingCycle);
    Serial.print("  Error = ");
    Serial.print(Error,5);
    Serial.print(" (done)\n");

    return(TrainingCycle);
}

void calcNeurNet()
{
    for(int p = 0 ; p < NrPattern ; p++ ) 
    { 
        Serial.print("  Training Pattern: ");
        Serial.println(p);
        Serial.print("  Input ");
        
        for(int i = 0 ; i < NrInputNodes ; i++ ) 
        {
          Serial.print(Input[p][i]);
        }
        Serial.print("\n  Output ");
        for(int i = 0 ; i < NrOutputNodes ; i++ )
        {
          Serial.print(Output[p][i]);
          Serial.print(" ");
        }
		
		/* Compute HiddenNodes layer activations */
        for(int i = 0 ; i < NrHiddenNodes ; i++ )
        {    
            Accum = HiddenNodesWeights[NrInputNodes][i];
            for(int j = 0 ; j < NrInputNodes ; j++ ) 
            {
                Accum += Input[p][j] * HiddenNodesWeights[j][i] ;
            }
            HiddenNodes[i] = actFunc(Accum);
        }

		/* Compute OutputNodes layer activations and calculate errors */
        for(int i = 0 ; i < NrOutputNodes ; i++ )
        {    
            Accum = OutputWeights[NrHiddenNodes][i];
            for(int j = 0 ; j < NrHiddenNodes ; j++ )
            {
                Accum += HiddenNodes[j] * OutputWeights[j][i] ;
            }
            OutputNodes[i] = actFunc(Accum);
        }
        Serial.print("\n  OutputNodes ");
        for(int i = 0 ; i < NrOutputNodes ; i++ )
        {       
            Serial.print(OutputNodes[i],3);
            Serial.print(" ");
        }
    }
}

float actFunc(float input)
{	/* Activation function for each neuron*/
	return(1.0/(1.0 + exp(-input)));
}

void normTrainingData()
{
    float max, min, temp;
    
    /* Calculate input range */
    for(int i=0; i<NrInputNodes; i++)
    {   max = Input[0][i];
        min = Input[0][i];
        for(int p=1; p<NrPattern; p++)
        {   /* find minimum and maximum */
            temp = Input[p][i];
            if(temp > max) max = temp;
            if(temp < min) min = temp;
        }
        InputMin[i] = min;
        InputMax[i] = max;
    }
    
    /* Calculate output range */
    for(int i=0; i<NrOutputNodes; i++)
    {   max = Output[0][i];
        min = Output[0][i];
        for(int p=1; p<NrPattern; p++)
        {   /* find minimum and maximum */
            temp = Output[p][i];
            if(temp > max) max = temp;
            if(temp < min) min = temp;
        }
        OutputMin[i] = min;
        OutputMax[i] = max;
    }
    
    float range;
    /* Scale down input data to 0...1 */
    for(int i=0; i<NrInputNodes; i++)
    {
        range = InputMax[i]-InputMin[i];
        min = InputMin[i];
        for(int p=0; p<NrPattern; p++)
        {   Input[p][i] -= min; /* shift to 0 */
            
            if(range != 0) Input[p][i] = Input[p][i] / range; /* scale down */
        }
    }
    
    /* Scale down outut data to 0...1 */
    for(int i=0; i<NrOutputNodes; i++)
    {
        range = OutputMax[i]-OutputMin[i];
        min = OutputMin[i];
        for(int p=0; p<NrPattern; p++)
        {   Output[p][i] -= min; /* shift to 0 */
            if(range != 0) Output[p][i] = Output[p][i] / range; /* scale down */
        }
    }
    
}

void printTrainingData()
{
    Serial.println("Inputs:");
    for(int p=0; p<NrPattern; p++)
    {   for(int i=0; i<NrInputNodes; i++)
        {   Serial.print(Input[p][i],3);
            Serial.print(" ");
        }
        Serial.println();
    }

    Serial.println("Outputs:");
    for(int p=0; p<NrPattern; p++)
    {   for(int i=0; i<NrOutputNodes; i++)
        {   Serial.print(Output[p][i],3);
            Serial.print(" ");
        }
        Serial.println();
    }
}

