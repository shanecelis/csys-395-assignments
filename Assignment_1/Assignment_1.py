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

def Plot_MatrixAsSquares(m):

    imshow(m,cmap=cm.gray,aspect='auto',interpolation='nearest');

            
def Plot_RowAsLine(m,i):

    plot( m[i,:] );
    
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

numGenes = 50;
numGenerations = 5000;
mutProb = 0.05;

for i in range(0,5):
    fits,genes = Evolve(numGenes,numGenerations,mutProb);
    figure(1);
    Plot_RowAsLine(fits,0);
    xlabel('Generation');
    ylabel('Fitness');
    
figure(2);
Plot_MatrixAsSquares(genes);
xlabel('Generation');
ylabel('Gene');
    
show();
