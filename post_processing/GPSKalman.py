import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import utm
import cartopy.feature as cfeature
import cartopy.crs as ccrs

coords = pd.read_csv("/Users/laratobias-tarsh/Documents/Climate324/kalmann/Team06Flight.txt")
ax = plt.axes(projection=ccrs.PlateCarree())
ax.set_global()
ax.coastlines(resolution='50m', # alternative: '110m', '50m', '10m' 
              color='grey')
ax.add_feature(cfeature.STATES.with_scale('10m'), zorder=2, linewidth=0.2, edgecolor='b')
ax.set_extent([-80, -85, 45, 40], crs=ccrs.PlateCarree())
ax.gridlines(draw_labels=True)


plt.plot((coords[" Longitude"]*-1),coords[" Latitude"],transform=ccrs.Geodetic(),zorder=10)

def lat_log_posx_posy(coords):

     px, py = [], []
     for i in range(len(coords[" Latitude"])):
         dx = utm.from_latlon(coords[" Latitude"][i], coords[" Longitude"][i]*-1)
         px.append(dx[0])
         py.append(dx[1])
     return px, py

def kalman_xy(x, P, measurement, R,
              Q = np.array(np.eye(4))):

    return kalman(x, P, measurement, R, Q,
                  F=np.array([[1.0, 0.0, 1.0, 0.0],
                              [0.0, 1.0, 0.0, 1.0],
                              [0.0, 0.0, 1.0, 0.0],
                              [0.0, 0.0, 0.0, 1.0]]),
                  H=np.array([[1.0, 0.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0, 0.0]]))

def kalman(x, P, measurement, R, Q, F, H):

    y = np.array(measurement).T - np.dot(H,x)
    S = H.dot(P).dot(H.T) + R  # residual convariance
    K = np.dot((P.dot(H.T)), np.linalg.pinv(S))
    x = x + K.dot(y)
    I = np.array(np.eye(F.shape[0]))  # identity matrix
    P = np.dot((I - np.dot(K,H)),P)

    # PREDICT x, P
    x = np.dot(F,x)
    P = F.dot(P).dot(F.T) + Q

    return x, P

def demo_kalman_xy():

    px, py = lat_log_posx_posy(coords)
    #plt.plot(px[::18], py[::18], 'ro')
    #plt.show()

    x = np.array([px[0], py[0], 0.01, 0.01]).T
    P = np.array(np.eye(4))*1000 # initial uncertainty
    result = []
    R = 0.01**2
    for meas in zip(px, py):
        x, P = kalman_xy(x, P, meas, R)
        result.append((x[:2]).tolist())
    kalman_x, kalman_y = zip(*result)
    plt.plot(px[::180], py[::180], 'ro')
    plt.plot(kalman_x, kalman_y, 'g-')
    return kalman_x,kalman_y


kalman_x, kalman_y = demo_kalman_xy()
plt.plot(kalman_x,kalman_y,transform=ccrs.UTM(16),zorder=10,c='r',lw=0.8)
plt.title("April 8th Balloon Flight + Kalman Filter")
plt.show()