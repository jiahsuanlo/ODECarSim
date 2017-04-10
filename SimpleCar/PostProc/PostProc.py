import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def plotDataFrame(df,varList=None,hFig=None):
    """ plot dataframe
    
    input:
        df: dataframe with time index
        varList: varable list, default is all
        hFig: figure handle, default is Nonw-> make new figure
        
    """
    if varList==None:
        cols= df.columns
        nCol= df.shape[1]
    else:
        cols= varList
        nCol= len(cols)
   
    n1= np.ceil(np.sqrt(nCol))
    n2= np.ceil(nCol/ n1)
    
    
    if hFig==None:
        plt.figure()
    else:
        plt.figure(hFig.number)
    ct= 1
    for c in cols:
        plt.subplot(n2,n1,ct)
        plt.plot(df.index, df[c])
        plt.hold(True)
        plt.title(c)
        ct+=1

dataDir="../SimpleCar"
fn= "vehiclePos_01.csv"
fileV= os.path.join(dataDir,fn)
fn= "vehiclePos_01b.csv"
fileV1= os.path.join(dataDir,fn)

fn= "wheelForce_01.csv"
fileW= os.path.join(dataDir,fn)
fn= "wheelForce_01b.csv"
fileW1= os.path.join(dataDir,fn)

dfV= pd.read_csv(fileV)
dfV1= pd.read_csv(fileV1)
dfV= dfV.set_index("Time")
dfV1= dfV1.set_index("Time")

dfW= pd.read_csv(fileW)
dfW1= pd.read_csv(fileW1)
dfW= dfW.set_index("Time")
dfW1= dfW1.set_index("Time")

hf= plt.figure()
plotDataFrame(dfV,hFig= hf)
plotDataFrame(dfV1,hFig= hf)

hf1= plt.figure()
plotDataFrame(dfW, hFig= hf1)  
plotDataFrame(dfW1, hFig= hf1)     
plt.show()   
