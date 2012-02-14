from numpy import *
from pylab import *
from scipy import *
from os.path import *
from random import *

def Evolve(numGenes,numGenerations,mutProb,numUpdates):

    parent = Matrix_Create(numGenes,numGenes);    
    parent = Matrix_Randomize(parent)*2-1;

    v = parent[9,:];
    print v;
    raw_input('');
    
    NeuralNetwork_DrawBehavior(1,parent,numUpdates);
    
    fits = Matrix_Create(1,numGenerations);
    
#    parentFit = Fitness1_Get(parent,numUpdates);
    parentFit = Fitness2_Get(parent,numUpdates);

    for g in range(0,numGenerations):
        
        child = Matrix_Copy(parent);
        child = Matrix_Perturb(child,0.05);

#        childFit = Fitness1_Get(child,numUpdates);
        childFit = Fitness2_Get(child,numUpdates);
        
        if ( childFit > parentFit ):
            parent = child;
            parentFit = childFit;

        fits[0,g] = parentFit;

        print g,parentFit;

    NeuralNetwork_DrawBehavior(2,parent,numUpdates);
    
    return fits;

def Fitness1_Get(synapses,numUpdates):

    numNeurons = size(synapses,0);
    
    neuronValues    = Matrix_Create(numUpdates,numNeurons);
    for j in range(0,numNeurons):
        neuronValues[0,j] = 0.5;

    for i in range(1,numUpdates): 
        neuronValues = NeuralNetwork_Update(numNeurons,neuronValues,synapses,i);

    actualValues = neuronValues[numUpdates-1,:];
    
    desiredValues = Vector_Create(numNeurons);
    for j in range(0,numNeurons,2):
        desiredValues[j] = 1;
    
    fitness = Vector_MeanSquaredError(actualValues,desiredValues);
    
    return( fitness );

def Fitness2_Get(synapses,numUpdates):

    numNeurons = size(synapses,0);
    
    neuronValues    = Matrix_Create(numUpdates,numNeurons);
    for j in range(0,numNeurons):
        neuronValues[0,j] = 0.5;

    for i in range(1,numUpdates): 
        neuronValues = NeuralNetwork_Update(numNeurons,neuronValues,synapses,i);
    
    fitness = Matrix_MaxDifferenceBetweenNeighbors(neuronValues);
    
    return( fitness );

def Matrix_Copy(m):

    ln = size(m,0);
    wd = size(m,1);

    copyOfM = zeros((ln,wd),dtype='f');

    for i in range(0,ln):
        for j in range(0,wd):
            copyOfM[i,j] = m[i,j];

    return copyOfM;

def Matrix_Create(i,j):

    m = zeros((i,j),dtype='f');

    return m;

def Matrix_MaxDifferenceBetweenNeighbors(m):

    diffs = 0.0;
    numUpdates = 0;
    
    for i in range(0,size(m,0)-1):
        for j in range(0,size(m,1)-1):
            diffs = diffs + abs( m[i,j] - m[i,j+1] );
            diffs = diffs + abs( m[i,j] - m[i+1,j] );
            numUpdates = numUpdates + 2;

    return( diffs / numUpdates );

def Matrix_Perturb(m,mutateProb):

    for i in range(0,size(m,0)):
        for j in range(0,size(m,1)):
            
            if ( rand() < mutateProb ):
                    m[i,j] = rand()*2-1;

    return m;

def Matrix_Randomize(m):

    for i in range(0,size(m,0)):
        for j in range(0,size(m,1)):
            m[i,j] = rand();
            
    return m;

def NeuralNetwork_DrawBehavior(figNo,synapses,numUpdates):

    numNeurons = size(synapses,0);
    
    neuronValues    = Matrix_Create(numUpdates,numNeurons);
    for j in range(0,numNeurons):
        neuronValues[0,j] = 0.5;

    for i in range(1,numUpdates): 
        neuronValues = NeuralNetwork_Update(numNeurons,neuronValues,synapses,i);

    figure(figNo);
    imshow(neuronValues,cmap=cm.gray,aspect='auto',interpolation='nearest');
    
def NeuralNetwork_Update(numNeurons,neuronValues,synapses,i):

    for j in range(0,numNeurons):

        newValue = 0.0;
        for sourceNeuron in range(0,numNeurons):
            sourceNeuronValue = neuronValues[i-1,sourceNeuron];
            synapseWeight = synapses[j,sourceNeuron];
            
            newValue = newValue + sourceNeuronValue*synapseWeight;

        if ( newValue > 1 ):
            newValue = 1;
        elif ( newValue < 0 ):
            newValue = 0;
            
        neuronValues[i,j] = newValue;
        
    return neuronValues;

def Vector_Create(wd):

    v = zeros((wd),dtype='f');

    return v;

def Vector_MeanSquaredError(m,n):
    
    error = 0.0;
    
    for i in range(0,len(m)):
        
        error = error + abs(m[i] - n[i]);
            
    return error/len(m);

# -------------------------- Private ------------------------

numGenes = 10;
numGenerations = 1000;
mutProb = 0.05;
numUpdates = 10;

fits = Evolve(numGenes,numGenerations,mutProb,numUpdates);

figure(3);
plot(fits[0,:],'k-');

print fits;

show();
