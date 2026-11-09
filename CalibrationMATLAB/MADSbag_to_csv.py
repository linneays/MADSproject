"""Export a MADS bag to CSV files that MATLAB can read with readmatrix.
 
    python3 bag_to_csv.py FILE.bag
 
Writes, next to the bag:
  FILE_driver.csv : t [s], enc_left [tic], enc_right [tic]   (cumulative counts)
  FILE_htc.csv    : t [s], x [m], y [m], yaw [rad]            (yaw unwrapped)
  FILE_gyro.csv   : t [s], gyro [rad/s]
Time is in seconds from the first message in the bag.
"""
import sys, math, os
import numpy as np
from read_bag import read_bag
 
def main(path):
    recs = read_bag(path)
    t0 = recs[0][0]
    drv, htc, gyr = [], [], []
    for ts, topic, m, _ in recs:
        t = (ts - t0) / 1e9
        if m is None:
            continue
        if topic == "driver":
            drv.append((t, m["encoders"]["left"], m["encoders"]["right"]))
        elif topic == "htc":
            H = m["Htracker"]["pose"]["H"]
            htc.append((t, H[0][3], H[1][3], math.atan2(H[1][0], H[0][0])))
        elif topic == "calibrated_imu":
            gyr.append((t, m["gyro"]))
    htc = np.array(htc)
    htc[:, 3] = np.unwrap(htc[:, 3])
    base = os.path.splitext(path)[0]
    np.savetxt(base + "_driver.csv", np.array(drv), delimiter=",", fmt="%.6f",
               header="t,enc_left,enc_right", comments="")
    np.savetxt(base + "_htc.csv", htc, delimiter=",", fmt="%.6f",
               header="t,x,y,yaw", comments="")
    np.savetxt(base + "_gyro.csv", np.array(gyr), delimiter=",", fmt="%.6f",
               header="t,gyro", comments="")
    print(f"{len(drv)} driver, {len(htc)} htc, {len(gyr)} gyro rows -> {base}_*.csv")
 
if __name__ == "__main__":
    main(sys.argv[1])