from numpy import *
from pylab import *
from scipy import *
from os.path import *
from random import *

def Evolve(numGenes,numGenerations,mutProb):

    parent = Matrix_Create(numGenes,1);
    parent = Matrix_Randomize(parent);

    fits = Matrix_Create(1,numGenerations);
    genes = Matrix_Create(numGenes,numGenerations);
    
    parentFit = Fitness_Get(parent);

    for g in range(0,numGenerations):

        genes = Matrix_CopyColumn(genes,g,parent,0);
        
        fits,parent,parentFit = UpdateOneGeneration(fits,g,parent,parentFit,mutProb);

    return fits,genes;

def Fitness_Get(m):

    total = 0.0;
    num = 0;
    
    for i in range(0,size(m,0)):
        for j in range(0,size(m,1)):
            total = total + m[i,j];
            num = num + 1;

    return( total/num );

def Matrix_Copy(m):

    ln = size(m,0);
    wd = size(m,1);

    copyOfM = zeros((ln,wd),dtype='f');

    for i in range(0,ln):
        for j in range(0,wd):
            copyOfM[i,j] = m[i,j];

    return copyOfM;

def Matrix_CopyColumn(targetM,targetMCol,sourceM,sourceMCol):

    for i in range(0,size(targetM,0)):
        targetM[i,targetMCol] = sourceM[i,sourceMCol];

    return targetM;

def Matrix_Create(i,j):

    m = zeros((i,j),dtype='f');

    return m;

def Matrix_Perturb(m,mutateProb):

    for i in range(0,size(m,0)):
        for j in range(0,size(m,1)):
            
            if ( rand() < mutateProb ):
                    m[i,j] = rand();

    return m;

def Matrix_Randomize(m):

    for i in range(0,size(m,0)):
        for j in range(0,size(m,1)):
            m[i,j] = rand();
            
    return m;

def Matrix_Randomize_Row(m,i):

    for j in range(0,size(m,1)):
        m[i,j] = rand();
            
    return m;

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

def Neurons_ComputePositions(numNeurons,neuronPositions):

    angle = 0.0;
    angleUpdate = (2.0 * 3.14159) / numNeurons;
    
    for j in range(0,numNeurons):

        x = 1 * sin(angle);
        y = 1 * cos(angle);

        angle = angle + angleUpdate;

        neuronPositions[0,j] = x;
        neuronPositions[1,j] = y;

    return neuronPositions;

def Neurons_Draw(numNeurons,neuronPositions):

    for j in range(0,numNeurons):

        plot(neuronPositions[0,j],neuronPositions[1,j],'ko',mfc=[1,1,1],markersize=18);
             
def Plot_MatrixAsSquares(m,xLabel,yLabel):

    imshow(m,cmap=cm.gray,aspect='auto',interpolation='nearest');

    xlabel(xLabel);
    ylabel(yLabel);

            
def Plot_RowAsLine(m,i):

    plot( m[i,:] );

def Step_One(numNeurons,numUpdates):

    neuronValues    = Matrix_Create(numUpdates,numNeurons);
    neuronPositions = Matrix_Create(2,numNeurons);

    neuronPositions = Neurons_ComputePositions(numNeurons,neuronPositions);

    Neurons_Draw(numNeurons,neuronPositions);

    return neuronValues, neuronPositions;

def Step_Two(numNeurons, neuronPositions):

    synapses        = Matrix_Create(numNeurons,numNeurons);
    synapses        = Matrix_Randomize(synapses);
    synapses        = 2*synapses - 1;
    
    Synapses_Draw(numNeurons,neuronPositions,synapses);

    Neurons_Draw(numNeurons,neuronPositions);
    
    return synapses;

def Step_Three(numNeurons, neuronPositions,synapses):

    Synapses_Draw_InColor(numNeurons,neuronPositions,synapses);

    Neurons_Draw(numNeurons,neuronPositions);
    
    return synapses;

def Step_Four(numNeurons, neuronPositions,synapses):

    Synapses_Draw_InColor_Widths(numNeurons,neuronPositions,synapses);

    Neurons_Draw(numNeurons,neuronPositions);
    
    return synapses;

def Synapses_Draw(numNeurons,neuronPositions,synapses):

    for i in range(0,numNeurons):
        for j in range(0,numNeurons):

            x1 = neuronPositions[0,i];
            x2 = neuronPositions[0,j];
            y1 = neuronPositions[1,i];
            y2 = neuronPositions[1,j];
            
            plot([x1,x2],[y1,y2],'k-');

def Synapses_Draw_InColor(numNeurons,neuronPositions,synapses):

    for i in range(0,numNeurons):
        for j in range(0,numNeurons):

            x1 = neuronPositions[0,i];
            x2 = neuronPositions[0,j];
            y1 = neuronPositions[1,i];
            y2 = neuronPositions[1,j];

            if ( synapses[i,j] < 0 ):
                c = [0.8,0.8,0.8];
            else:
                c = [0,0,0];
                
            plot([x1,x2],[y1,y2],'k-',color=c);

def Synapses_Draw_InColor_Widths(numNeurons,neuronPositions,synapses):

    for i in range(0,numNeurons):
        for j in range(0,numNeurons):

            x1 = neuronPositions[0,i];
            x2 = neuronPositions[0,j];
            y1 = neuronPositions[1,i];
            y2 = neuronPositions[1,j];

            if ( synapses[i,j] < 0 ):
                c = [0.8,0.8,0.8];
            else:
                c = [0,0,0];

            w = int(10*abs(synapses[i,j]))+1;
            
            plot([x1,x2],[y1,y2],'k-',color=c,linewidth=w);
            
def UpdateOneGeneration(fits,g,parent,parentFit,mutProb):

        fits[0,g] = parentFit;
        
        child = Matrix_Copy(parent);

        child = Matrix_Perturb(child,mutProb);
        childFit = Fitness_Get(child);
        
        if ( childFit > parentFit ):
            parent = child;
            parentFit = childFit;

        return fits, parent, parentFit;
        
# -------------------------- Private ------------------------

numNeurons = 10;
numUpdates = 50;

figure(1);
neuronValues, neuronPositions = Step_One(numNeurons,numUpdates);
savefig('Fig1.png'); savefig('Fig1.eps');

figure(2);
synapses = Step_Two(numNeurons,neuronPositions);
savefig('Fig2.png'); savefig('Fig2.eps');

figure(3);
synapses = Step_Three(numNeurons,neuronPositions,synapses);
savefig('Fig3.png'); savefig('Fig3.eps');

figure(4);
synapses = Step_Four(numNeurons,neuronPositions,synapses);
savefig('Fig4.png'); savefig('Fig4.eps');

figure(5);
neuronValues = Matrix_Randomize_Row(neuronValues,0);
for i in range(1,numUpdates):
    neuronValues = NeuralNetwork_Update(numNeurons,neuronValues,synapses,i);
print neuronValues;
Plot_MatrixAsSquares(neuronValues,'Neuron','Time Step');
savefig('Fig5.png'); savefig('Fig5.eps');
    
show();
