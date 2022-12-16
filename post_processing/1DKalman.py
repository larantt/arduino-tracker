import numpy as np
from scipy.optimize import curve_fit
import pandas as pd
import matplotlib.pyplot as plt


def denoise(data):

    def oavar(data, rate, numpoints=1793):

        x = np.cumsum(data)

        max_ratio = 1/179
        
        ms = np.unique(
            np.logspace(0, np.log10(len(x) * max_ratio), numpoints
           ).astype(int))        

        oavars = np.empty(len(ms))
        for i, m in enumerate(ms):
            oavars[i] = (
                (x[2*m:] - 2*x[m:-m] + x[:-2*m])**2
            ).mean() / (2*m**2)

        return ms / rate, oavars

    def ln_NKfit(ln_tau, ln_N, ln_K):
        tau = np.exp(ln_tau)
        N, K = np.exp([ln_N, ln_K])
        oadev = N**2 / tau + K**2 * (tau/3)
        return np.log(oadev)

    def get_NK(data, rate):
        taus, oavars = oavar(data, rate)

        ln_params, ln_varmatrix = (
            curve_fit(ln_NKfit, np.log(taus), np.log(oavars))
        )
        return np.exp(ln_params)    

    # Initialize state and uncertainty
    state = data[0]
    output = np.empty(len(data))

    rate = 1 # We can set this to 1, if we're calculating N, K internally
    # N and K will just be scaled relative to the sampling rate internally
    dt = 1/rate

    N, K = get_NK(data, rate)

    process_noise = K**2 * dt
    measurement_noise = N**2 / dt

    covariance = measurement_noise

    for index, measurement in enumerate(data):
        # 1. Predict state using system's model

        covariance += process_noise

        # Update
        kalman_gain = covariance / (covariance + measurement_noise)

        state += kalman_gain * (measurement - state)
        covariance = (1 - kalman_gain) * covariance

        output[index] = state

    return output



f2 = pd.read_csv("/Users/laratobias-tarsh/Documents/Climate324/kalmann/liftaltimeter.txt")
unfiltered = f2["unfiltered"].to_numpy()
ardukal = f2["simplekalman"].to_numpy()
pykal = denoise(unfiltered)


#unfiltered, = plt.plot(unfiltered,c='g',alpha=0.4,lw=4)
#ardukal, = plt.plot(ardukal,c='k',lw=0.5)
#pykal, = plt.plot(pykal,c='r',lw=0.5)

#plt.title(" Barometer Lift Test data")
#plt.legend([unfiltered,ardukal,pykal],['No kalman filter', 'Arduino Kalman Filter','Python Kalman Filter'])

f = pd.read_csv("/Users/laratobias-tarsh/Documents/Climate324/kalmann/Team06Flight.txt")
dataP = f[" Pressure (atm)"].to_numpy()
dataP = dataP * 1013.25 # convert data to mb
denoised_output = denoise(dataP)
#pres_raw, = plt.plot(dataP,alpha=0.2)
#pres_denoised, =plt.plot(denoised_output,lw=0.5)
#alt_raw, =plt.plot(f[" GPS Altitude"],lw=0.5)
#alt_denoised, = plt.plot(denoise(f[" GPS Altitude"].to_numpy()),alpha=0.2)


dataT = f[" Outside Temperature (Thermistor) (C)"].to_numpy()
denoised_outputT = denoise(dataT)
internal, = plt.plot(dataT,alpha=0.2)
external, = plt.plot(denoised_outputT,lw=0.5,c='k')

dataH = abs(f[" Humidity"].to_numpy())
denoised_outputH = denoise(dataH)
hum_raw, = plt.plot(dataH,alpha=0.2)
hum_filt, =plt.plot(denoised_outputH,lw=0.5)
plt.title("Humidity and Temperature over April 08th Ballon Flight")
plt.legend([hum_raw,hum_filt,internal,external],['Raw Humidity', 'Filtered Humidity','Raw Temperature','Filtered Humidity'])

dataTi = f[" Internal Temperature (TMP36) (C)"].to_numpy()
denoised_outputT = denoise(dataTi)

plt.show()
"""
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import metpy.calc as mpcalc
from metpy.plots import add_metpy_logo, SkewT
from metpy.units import units
plt.rcParams['figure.figsize'] = (9, 9)

p = dataP[3977:-1] * units.hPa
T = dataT[3977:-1] * units.degC
H = dataH[3977:-1] * units.percent
Td = mpcalc.dewpoint_from_relative_humidity(T, H)

skew = SkewT()

# Plot the data using normal plotting functions, in this case using
# log scaling in Y, as dictated by the typical meteorological plot
skew.plot(p, T, 'r',alpha=0.35)
skew.plot(p, Td, 'g',alpha=0.35)

# Add the relevant special lines
#skew.plot_dry_adiabats()
#skew.plot_moist_adiabats()
#skew.plot_mixing_lines()
#skew.ax.set_ylim(1000, 100)


p1 = denoised_output[3977:-1] * units.hPa
T1 = denoised_outputT[3977:-1] * units.degC
H1 = denoised_outputH[3977:-1] * units.percent
Td1 = mpcalc.dewpoint_from_relative_humidity(T1, H1)
skew.plot(p1, T1, 'r')
skew.plot(p1, Td1, 'g')

# Add the relevant special lines
skew.plot_dry_adiabats()
skew.plot_moist_adiabats()
skew.plot_mixing_lines()
skew.ax.set_ylim(1000, 100)
skew.ax.set_title("Kalman filtered Sounding(ish) from Arduino Balloon Launch, April 08 2021")


# external humidity sensor probably necesary

#plt.show()"""