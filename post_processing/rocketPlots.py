import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
from scipy.stats import linregress
from mpl_toolkits.axes_grid.inset_locator import inset_axes


def is_outlier(points, thresh=1.1):
   
    if len(points.shape) == 1:
        points = points[:,None]
    median = np.median(points, axis=0)
    diff = np.sum((points - median)**2, axis=-1)
    diff = np.sqrt(diff)
    med_abs_deviation = np.median(diff)

    modified_z_score = 0.6745 * diff / med_abs_deviation

    return modified_z_score > thresh


################################################################################

# current directory csv files
csvs = [x for x in os.listdir('.') if x.endswith('.csv')]
fns = [os.path.splitext(os.path.basename(x))[0] for x in csvs]

# read in csv files
d = {}
for i in range(len(fns)):
    d[i] = pd.read_csv(csvs[i])

# plot launch data for all launches by parameter
ax1 = plt.subplot(311)
for i in range(len(d)):
    plt.plot( d[i]["Pressure"][0:-1])
plt.tick_params('x', labelsize=6)
plt.tick_params('x', labelbottom=False)
ax1.set_ylabel("Altitude (m)")
plt.title("Parameters recorded from all launches")

# share x only
ax2 = plt.subplot(312, sharex=ax1)
for i in range(len(d)):
    plt.plot( d[i]["Altitude"][0:-1])
# make these tick labels invisible
plt.tick_params('x', labelbottom=False)
ax2.set_ylabel("Pressure (Pa)")

ax3 = plt.subplot(313, sharex=ax1)
for i in range(len(d)):
    plt.plot(d[i]["Acceleration x"][0:-1])
    plt.plot(d[i]["Acceleration y"][0:-1])
    plt.plot(d[i]["Acceleration z"][0:-1])
ax3.set_ylabel("Acceleration (ms)")  
ax3.set_ylim(-45, 45) 
plt.tick_params('x')
#plt.show()

################################################################################

# plot correlation of altitude  data and pressure data
x=d[1]["Altitude"][0:-1]
y=d[1]["Pressure"][0:-1]
# Keep only the "good" points
# "~" operates as a logical not operator on boolean numpy arrays
filtered = x[~is_outlier(x)]
filteredy = y[~is_outlier(y)]
res = list(filter(lambda c : c > 0, filtered))
resy = list(filter(lambda c : c > 0, filteredy))
# Plot the results
fig, (ax4, ax5) = plt.subplots(nrows=2,figsize=(8,8))

ax4.scatter(x,y)
ax4.set_title('Raw Data',fontsize=15)
ax4.set_xlabel("Altitude (m)")
ax4.set_ylabel("Pressure (Pa)")
fig.suptitle("Example of outlier exclusion using modified Z-Score for launch 1", fontsize=17)

ax5.scatter(filtered,filteredy)
ax5.set_title('Without Outliers',fontsize=15)

ax1.tick_params(axis='both', labelsize=10)
ax5.tick_params(axis='both', labelsize=10)
ax5.set_xlabel("Altitude (m)")
ax5.set_ylabel("Pressure (Pa)")


fig.tight_layout(pad=2.0)


################################################################################

fig2 = plt.figure(figsize=(10,10))

ax7 = fig2.add_subplot(projection='3d')
colors = range(len(d))
customlines = []

for c, k in zip(colors, d):
    # Generate the random data for the y=k 'layer'.
    x=d[k]["Altitude"][0:310]
    y=d[k]["Pressure"][0:310]
    # Keep only the "good" points
    # "~" operates as a logical not operator on boolean numpy arrays
    filtered = x[~is_outlier(x)]
    filteredy = y[~is_outlier(y)]
    res = list(filter(lambda c : c > 0, filtered))
    resy = list(filter(lambda c : c > 0, filteredy))

    # Plot the bar graph given by xs and ys on the plane y=k with 80% opacity.
    ax7.plot(res, resy, zs=k, zdir='y', alpha=0.8,marker='x')
    slope, intercept, r_value, p_value, std_err = linregress(res, resy)
    customlines.append(f'{round(slope,4)}x + {round(intercept,4)}')

ax7.set_yticklabels(customlines, rotation=5,verticalalignment='baseline',
                horizontalalignment='left')
ax7.set_yticks(range(len(customlines)))
ax7.set_xlabel('Altitude (m)')
ax7.set_zlabel('Pressure (m)') 
#ax7.legend(customlines, loc='best',fontsize=10)
fig2.suptitle("Plot of pressure against altitude", fontsize=17)

################################################################################

fig3 = plt.figure()
ax8 = fig3.add_subplot()
ax8b=ax8.twinx()
colors = range(len(d))

for c, k in zip(colors, d):
    # Generate the random data for the y=k 'layer'.
    x=d[k]["Altitude"][0:300]
    y=d[k]["Pressure"][0:300]
    # Keep only the "good" points
    # "~" operates as a logical not operator on boolean numpy arrays
    filtered = x[~is_outlier(x)]
    filteredy = y[~is_outlier(y)]
    res = list(filter(lambda c : c > 0, filtered))
    resy = list(filter(lambda c : c > 0, filteredy))

    # Plot the bar graph given by xs and ys on the plane y=k with 80% opacity.
    ax8.plot(res, alpha=0.8)
    ax8b.plot(resy, alpha=0.8)

ax8.set_xlabel('Altitude (m)')
ax8b.set_xlabel('Pressure (Pa)')

fig3.suptitle("Relationship between pressure and altitude", fontsize=17)

#ax8.set_zlim(98, 102)
fig3.tight_layout(pad=2.0)

################################################################################


subpos = [0.2,0.6,0.3,0.3]
fig5, axs4 = plt.subplots(2, 7, constrained_layout=True, figsize=(6.4, 3.2))
def hatches_plot(ax,dat,idx):
    xx = dat[idx]["Acceleration x"][0:50]
    yy = dat[idx]["Acceleration y"][0:50]
    zz = dat[idx]["Acceleration z"][0:50]
    ax.plot(xx, 'r')
    ax.plot(yy, 'k')
    ax.plot(zz, 'b')
    ax.set_xlabel('time')
    ax.set_ylabel('acceleration')
    ax.set_title(f'launch {idx}')

    
def hatches_plot2(ax,dat,idx): 
    #inset_ax = inset_axes(ax,
                         # height="100%", # set height
                         # width="100%", # and width
                         # loc=3)
    #inset_ax.yaxis.tick_right()
    #inset_ax.yaxis.set_label_position("right")
    
    aa = dat[idx]["Altitude"][0:-1]   
    filteredaa = aa[~is_outlier(aa)]
    resaa = list(filter(lambda c : c > 0, filteredaa))
    
# insert
    ax.plot(resaa, 'g')
    ax.set_xlabel('Time')
    ax.set_ylabel('Alt (m)')

for ax, h in zip(axs4.flat ,d):
   # hatches_plot(ax,d, h)
    hatches_plot2(ax,d, h)

fig5.suptitle("Altitude of each test rocket launch (smoothed)")

###############################################################################
burntimes = [21,20,20,19,19,18,21,19,19,19,21,19,19,20]
maxalts = [96.70873084,94.08902189,93.44211192,74.01857977,
            86.73086258,85.65221956,94.37268363,94.74369633,
            92.69620384,92.00024657,78.16611766,74.59117215,
            89.28653406,84.91526481]


fig6 = plt.figure()
ax9 = fig6.add_subplot()
#ax9.plot(burntimes)
ax9.scatter(burntimes,maxalts)
ax9.set_xlabel("Burn Time (10ms)")
ax9.set_ylabel("Maximum Altitude (m)")
fig6.suptitle("Burn Time vs Maximum Acceleration")

plt.show()