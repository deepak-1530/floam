# compare error between scan matcher and loam trajectories
# compute difference between values with same timestamps

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


def plotTrajectory(floamTraj, scanMatcherTraj, loamTraj):
    lW = 2

    font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 22}

    plt.rc('font', **font)
    
    plt.plot(floamTraj[:,1], floamTraj[:,2], linewidth=lW)
    plt.plot(loamTraj[:,1], loamTraj[:,2], lineWidth=lW)
    plt.plot(scanMatcherTraj[:,1], scanMatcherTraj[:,2],lineWidth=lW)
    plt.legend(["FLOAM","LOAM","ScanMatcher"])
    plt.xlabel("X coordinate")
    plt.ylabel("Y Coordinate")
    plt.show()
    fig = plt.figure()

    ax = plt.axes(projection='3d')
    ax.plot3D(floamTraj[:,1], floamTraj[:,2], floamTraj[:,3], linewidth=lW)
    ax.plot3D(loamTraj[:,1], loamTraj[:,2], loamTraj[:,3], linewidth=lW)
    ax.plot3D(scanMatcherTraj[:,1], scanMatcherTraj[:,2], scanMatcherTraj[:,3], linewidth=lW)
    plt.legend(["FLOAM","LOAM","ScanMatcher"])
    plt.show()

    

def computeMSE(loamTraj, scanMatcherTraj, floamTraj):

    print(loamTraj.shape)
    err_ = abs(loamTraj[0:862,1:7] - scanMatcherTraj[0:862, 1:7])
    print(f' \n **** Mean Error Values (X,Y,Z,R,P,Y) are : {np.mean(err_[:,0]), np.mean(err_[:,1]), np.mean(err_[:,2]), np.mean(err_[:,3]), np.mean(err_[:,4]), np.mean(err_[:,5])}')

    posErr = 0
    orientErr = 0
    count = 0

    mXLOAM = np.mean(loamTraj[:,1])
    mXSM   = np.mean(scanMatcherTraj[:,1])

    mYLOAM = np.mean(loamTraj[:,2])
    mYSM   = np.mean(scanMatcherTraj[:,2])

    mZLOAM = np.mean(loamTraj[:,3])
    mZSM   = np.mean(scanMatcherTraj[:,3])

    print(f'\n **** Mean values along each axis (x,y,z) [loam, scanMatch] are : {[mXLOAM, mXSM], [mYLOAM, mYSM], [mZLOAM, mZSM]}')

    tLoam = loamTraj[:,0]
    tScanMatcher = scanMatcherTraj[:,0]
    
    for t in tScanMatcher:
        if t in tLoam:
            tIndexSM = np.where(tScanMatcher == t)
            tIndexLoam = np.where(tLoam == t)
            posLoam = loamTraj[tIndexLoam, 1:7].reshape(6)
            posScanMatcher = scanMatcherTraj[tIndexSM, 1:7].reshape(6)
            posErr += abs(posLoam - posScanMatcher)
            count += 1


    print(f'\n **** Mean Absolute Error is : {posErr/count, count} \n')

    plotTrajectory(loamTraj, scanMatcherTraj, floamTraj)

    

if __name__=="__main__":
    loamTraj        = np.load('/home/deepak/IIITD/catkin_ws/src/floam/data/posesFloam.npy')#input("Enter loam trajectory file (npy): ")
    scanMatcherTraj = np.load('/home/deepak/IIITD/catkin_ws/src/loam_velodyne/data/posesScanMatcher.npy')#input("Enter scan matcher trajectory file (npy): ")
    loamTraj2       = np.load('/home/deepak/IIITD/catkin_ws/src/loam_velodyne/data/posesLoam_2.npy')
    computeMSE(loamTraj, scanMatcherTraj, loamTraj2)