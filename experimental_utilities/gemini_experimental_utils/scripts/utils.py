import numpy as np

def angle_correction(cur_ang, prev_ang):
	correctedAng = np.zeros(len(cur_ang))
	for i in range(len(cur_ang)):
		if (prev_ang[i] >= 0.):
			ang_range = np.floor(prev_ang[i]/np.pi)
			if (cur_ang[i] >= 0):
				correctedAng[i] = cur_ang[i] + ang_range*np.pi
			else:
				correctedAng[i] = cur_ang[i] + (ang_range+1)*np.pi
		elif(prev_ang[i] < 0.):
			ang_range =  np.ceil(prev_ang[i]/np.pi)
			if (cur_ang[i] > 0):
				correctedAng[i] = cur_ang[i] + (ang_range-1)*np.pi
			else:
				correctedAng[i] = cur_ang[i] + ang_range*np.pi
	return correctedAng