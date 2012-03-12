#!/opt/local/bin/python2.5
# a10.py

from scipy import *
from numpy import *
from matplotlib import *
from pylab import *
import os
import sys
import time 
def MatrixCreate(m, n):
  return zeros((m,n), float)

def pause():
  draw()
  print 'Paused.  Please press CTRL-D.'
  sys.stdin.readlines()

def VectorCreate(m):
  return zeros((m,), dtype='f')

def MatrixRandomize(v):
  fn = vectorize(lambda x: random())
  return 2 * fn(v) - 1

def MeanSquaredError(v1, v2):
  v = v1 - v2
  (count) = shape(v)
  return dot(v, v)/count

def Fitness1(synapses):
  numNeurons = 10
  numTimeSteps = 10
  neuronValues = MatrixCreate(numTimeSteps,numNeurons)
  neuronValues[0] = neuronValues[0] + 0.5
  neuronValues[0,0] = 1
  for i in range(0,numTimeSteps - 1):
    neuronValues[i + 1] = Update(neuronValues, synapses, i)
  plotNeuronValues(neuronValues)
  actualNeuronValues = neuronValues[numTimeSteps - 1, :]
  desiredNeuronValues = VectorCreate(numNeurons)
  for j in range(0, numNeurons, 2):
    desiredNeuronValues[j] = 1
  return MeanSquaredError(actualNeuronValues, desiredNeuronValues)

def Fitness2(synapses):
  numNeurons = 10
  numTimeSteps = 10
  neuronValues = MatrixCreate(numTimeSteps,numNeurons)
  neuronValues[0] = neuronValues[0] + 0.5 
  for i in range(0,numTimeSteps - 1):
    neuronValues[i + 1] = Update(neuronValues, synapses, i)
  plotNeuronValues(neuronValues)
  actualNeuronValues = neuronValues[numTimeSteps - 1, :]
  diff = 0.0
  for i in range(0, numNeurons - 1):
    for j in range(0, numNeurons - 1):
      diff = diff + abs(neuronValues[i,j] - neuronValues[i, j + 1])
      diff = diff + abs(neuronValues[i + 1,j] - neuronValues[i, j])
  diff = diff/(2*(numNeurons - 2)*(numNeurons - 2))
  return diff

def FitnessRobot(synapses):
  synapses.tofile('weights.dat', ' ', '%s')
  fileName = 'fit.dat'
  while (os.path.exists(fileName) == False):
    time.sleep(0.2)
  f =  open(fileName)
  try:
    a = loadtxt(f)
    #print a
  finally:
    f.close()
  os.remove(fileName)
  return a

Fitness = FitnessRobot

def MatrixPerturb(p, prob):
  def f(x):
    if (random() < prob): 
      return 2 * random() - 1
    else:
      return x
  fn = vectorize(f) 
  return fn(p)

def PlotVectorAsLine(v):
  cols = v.shape[0]
  xs = arange(0,cols)
  plot(xs, v)

def plotNeuronValues(neuronValues):
  imshow(neuronValues, cmap=cm.gray, aspect='auto', interpolation='nearest')
  xlabel('Neuron')
  ylabel('Time')
  #draw()

def runNN(neuronValues, synapses):
  (numTimeSteps, numNeurons) = shape(neuronValues)
  for i in range(0,numTimeSteps - 1):
    neuronValues[i + 1] = Update(neuronValues, synapses, i)
  return neuronValues

def plotNeuronEdge(neuronPositions, synapses):
  (rows, cols) = shape(neuronPositions)
  for i in range(0, cols):
    for j in range(i + 1, cols):
      p1 = neuronPositions[:, i]
      p2 = neuronPositions[:, j]
      #plot([p1[0], p2[0]], [p1[1], p2[1]])
      if synapses[i, j] > 0:
        color = [0.8,0.8,0.8]
      else:
        color = [0.,0.,0.]
      #plot([p1[0], p2[0]], [p1[1], p2[1]], color)
      w = int(10*abs(synapses[i,j])) + 1
      plot([p1[0], p2[0]], [p1[1], p2[1]], color = color, linewidth = w)

def Update(neuronValues, synapses, i):
  (rows, cols) = shape(neuronValues)
  output = dot(neuronValues[i], synapses)
  # 1 x 10 = 1 x 10 * 10 x 10 = 1 x 10
  for j in range(0, cols):
    if output[j] < 0.0:
      output[j] = 0.0
    elif output[j] > 1.0:
      output[j] = 1.0
  return output


runCount = 1
generationCount = 100
fits = MatrixCreate(runCount, generationCount)
inputCount = 4
outputCount = 8
# Genes = MatrixCreate(geneCount, generationCount)
for currentRun in range(0, runCount):
  # currentRun = 0
  # if True:
  #figure(1)
  parent = MatrixCreate(inputCount,outputCount)
  parent = MatrixRandomize(parent) 
  parentFitness = Fitness(parent)
  # draw()
  # figure(2)
  for currentGeneration in range(0, generationCount):
    fits[currentRun, currentGeneration] = parentFitness
    #Genes[:, currentGeneration] = parent[0]
    child = MatrixPerturb(parent,0.05) 
    childFitness = Fitness(child) 
    print currentGeneration, parentFitness, childFitness 
    if ( childFitness > parentFitness ):
      parent = child 
      parentFitness = childFitness
      parent.tofile('best-weights.dat', ' ', '%s')
  Fitness(parent)
  # draw()
  # figure(3)
  # PlotVectorAsLine(fits[currentRun])
  # draw()

quit()

numNeurons = 10
numTimeSteps = 50
neuronValues = MatrixCreate(numTimeSteps,numNeurons)
neuronValues[0] = MatrixRandomize(neuronValues[0])

neuronPositions = MatrixCreate(2,numNeurons)
angle = 0.0
angleUpdate = 2 * pi / numNeurons
for i in range(0, numNeurons):
  neuronPositions[0,i] = sin(angle)
  neuronPositions[1,i] = cos(angle)
  angle = angle + angleUpdate

plot(neuronPositions[0], neuronPositions[1], 'ko', markerfacecolor = [1,1,1], markersize=18)

synapses = MatrixCreate(numNeurons, numNeurons)

synapses = 2 * MatrixRandomize(synapses) - 1


figure(1)
clf()
plotNeuronEdge(neuronPositions, synapses)
figure(2)
clf()
imshow(neuronValues, cmap=cm.gray, aspect='auto', interpolation='nearest')
figure(3)
clf()
for i in range(0,numTimeSteps - 1):
  neuronValues[i + 1] = Update(neuronValues, synapses, i)
  #print neuronValues[i + 1]

imshow(neuronValues, cmap=cm.gray, aspect='auto', interpolation='nearest')
draw()
