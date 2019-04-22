import SMG_ADS1256PyLib
import numpy as np
import matplotlib.pyplot as plt
import scipy.fftpack
import time
import pandas as pd


def save_data(data):
    data = pd.DataFrame(data)
    data.to_csv('Test.csv')

def plot_result(Length,data):
    
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(data)

    N= Length
    T = 1/(1/0.000240)
    yf = scipy.fftpack.fft(data)
    xf = np.linspace(0.0, 1.0 / (2.0 * T),N/2)

    plt.subplot(2,1,2)
    plt.plot(xf,2.0/N*np.abs(yf[:N//2]))
    plt.xlabel('frequecny')
    plt.ylabel('Amplifier')
    plt.show()


if __name__ == '__main__':
    buffer = False
    PGA = 0x00
    samplingRate = 0xF0
    sampleCount = 3000
    InitialSPI = SMG_ADS1256PyLib.initialSPI() #Initial RPI SPI I/O pin
    SetParameter = SMG_ADS1256PyLib.setADC1256BaseParameter(buffer,PGA,samplingRate) #Args(Buffer, PGA, SamplingRate)
    while(1):
        start_time = time.time()
        data = SMG_ADS1256PyLib.readDiffChannelVolts(sampleCount) #read AIN0/AIN1 differential channel Args(Sample_count)   
        data = np.array(data) / 1670000
        end_time = time.time()
        EST = end_time - start_time
        print('spent time : ',EST)
    plot_result(sampleCount,data)
    endflag = SMG_ADS1256PyLib.endSPIfunc()
    #save_data(data)
